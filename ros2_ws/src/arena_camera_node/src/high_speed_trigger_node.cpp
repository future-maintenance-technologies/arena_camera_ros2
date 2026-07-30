// Synchronized-trigger broadcaster (composable component).
//
// Owns no camera. It opens an Arena system and, on a timer, broadcasts a
// PTP-scheduled action command so every camera in `trigger_source:=action`
// mode exposes at the same PTP instant. Because it holds no device and never
// starts a stream, it restarts almost instantly: if it dies, the cameras
// simply stop receiving triggers until it comes back, and resume in sync once
// it does. A heartbeat topic lets a sensor monitor watch and restart it.
//
// Schedule times come from the grandmaster NIC's PTP hardware clock, which is
// the timebase the cameras are slaved to. See ptp_interface below.

#include <fcntl.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "ArenaApi.h"

using namespace std::chrono_literals;

// A PHC file descriptor is turned into a POSIX clockid by this encoding; see
// clock_gettime(3) and linuxptp's clockadj.
static constexpr int kClockFd = 3;
static inline clockid_t fd_to_clockid(int fd)
{
  return static_cast<clockid_t>((~static_cast<unsigned int>(fd) << 3) | kClockFd);
}

// Resolve the PTP hardware clock backing `iface` (the NIC ptp4l disciplines)
// and return an open fd to its /dev/ptpN. Throws if the interface has no PHC,
// which means this host cannot serve the cameras' timebase at all.
static int open_phc_(const std::string& iface)
{
  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    throw std::runtime_error("failed to open socket for PHC lookup: " +
                             std::string(std::strerror(errno)));
  }

  struct ethtool_ts_info tsi = {};
  tsi.cmd = ETHTOOL_GET_TS_INFO;

  struct ifreq ifr = {};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  ifr.ifr_data = reinterpret_cast<char*>(&tsi);

  const bool ok = ::ioctl(sock, SIOCETHTOOL, &ifr) >= 0;
  const int ioctl_errno = errno;
  ::close(sock);

  if (!ok) {
    throw std::runtime_error("SIOCETHTOOL on '" + iface +
                             "' failed: " + std::strerror(ioctl_errno));
  }
  if (tsi.phc_index < 0) {
    throw std::runtime_error("interface '" + iface +
                             "' has no PTP hardware clock");
  }

  const std::string dev = "/dev/ptp" + std::to_string(tsi.phc_index);
  const int fd = ::open(dev.c_str(), O_RDONLY);
  if (fd < 0) {
    throw std::runtime_error("failed to open " + dev + ": " +
                             std::strerror(errno));
  }
  return fd;
}

class HighSpeedTriggerNode : public rclcpp::Node
{
 public:
  explicit HighSpeedTriggerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("high_speed_trigger_node", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    trigger_rate_hz_ = this->declare_parameter("trigger_rate_hz", 10.0);
    // Residual calibration knob (ns) applied on top of the PHC time. Leave at 0
    // unless the cameras' servo settles on a measurable constant bias.
    ptp_offset_ns_ = this->declare_parameter<int64_t>("ptp_offset_ns", 0);

    // Scheduled action commands are expressed in the cameras' PTP timebase,
    // which is the PHC of the NIC ptp4l serves as grandmaster - NOT this host's
    // CLOCK_REALTIME. The two agree only while phc2sys is converged, so reading
    // the PHC directly keeps the schedule correct even when phc2sys is not.
    // Scheduling off CLOCK_REALTIME instead turns any phc2sys fault into silent
    // capture loss: the camera's scheduled-action queue fills with far-future
    // commands, drops every trigger until they execute, and the effective frame
    // rate collapses to queue_depth / clock_offset.
    ptp_interface_ = this->declare_parameter("ptp_interface", std::string(""));
    if (ptp_interface_.empty()) {
      throw std::runtime_error(
          "ptp_interface is required: name the NIC whose PHC ptp4l serves as "
          "grandmaster (e.g. ptp_interface:=enp134s0f0np0)");
    }
    phc_fd_ = open_phc_(ptp_interface_);
    phc_clock_ = fd_to_clockid(phc_fd_);

    m_pSystem_ = Arena::OpenSystem();
    m_pSystem_->UpdateDevices(100);  // initialize the NIC(s) for broadcasting

    // Match the per-device action keys the camera nodes set; target IP
    // 0xFFFFFFFF broadcasts to every camera on the subnet.
    auto nm = m_pSystem_->GetTLSystemNodeMap();
    Arena::SetNodeValue<int64_t>(nm, "ActionCommandDeviceKey", 1);
    Arena::SetNodeValue<int64_t>(nm, "ActionCommandGroupKey", 1);
    Arena::SetNodeValue<int64_t>(nm, "ActionCommandGroupMask", 1);
    Arena::SetNodeValue<int64_t>(nm, "ActionCommandTargetIP", 0xFFFFFFFF);

    m_pub_ = this->create_publisher<std_msgs::msg::UInt64>("~/heartbeat", 10);

    auto period = std::chrono::duration<double>(1.0 / trigger_rate_hz_);
    m_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&HighSpeedTriggerNode::fire_, this));

    RCLCPP_INFO(this->get_logger(),
                "high_speed_trigger_node broadcasting action commands at %.2f Hz "
                "on the %s PHC timebase",
                trigger_rate_hz_, ptp_interface_.c_str());

    check_host_clock_offset_();
    m_offset_timer_ = this->create_wall_timer(
        30s, std::bind(&HighSpeedTriggerNode::check_host_clock_offset_, this));
  }

  ~HighSpeedTriggerNode()
  {
    if (phc_fd_ >= 0) ::close(phc_fd_);
    if (m_pSystem_) Arena::CloseSystem(m_pSystem_);
  }

 private:
  int64_t phc_now_ns_() const
  {
    struct timespec ts = {};
    if (::clock_gettime(phc_clock_, &ts) < 0) {
      throw std::runtime_error("clock_gettime on PHC failed: " +
                               std::string(std::strerror(errno)));
    }
    return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
  }

  // The PHC drives the cameras; CLOCK_REALTIME stamps everything this host
  // records. Capture stays correct when they diverge, but host-recorded data no
  // longer lines up with image timestamps, so surface it loudly.
  void check_host_clock_offset_()
  {
    try {
      int64_t host_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
      int64_t offset_ns = phc_now_ns_() - host_ns;
      if (std::abs(offset_ns) > kMaxHostOffsetNs) {
        RCLCPP_ERROR(this->get_logger(),
                     "PHC (%s) is %.3f s from CLOCK_REALTIME; image timestamps "
                     "will not match host-recorded data. Check phc2sys.",
                     ptp_interface_.c_str(), offset_ns / 1e9);
      }
    } catch (std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to read PHC offset\n%s", e.what());
    }
  }

  void fire_()
  {
    try {
      // Read "now" straight from the grandmaster PHC (the cameras' timebase)
      // and schedule the capture half a period ahead.
      // ponytail: half-period lead works for rates up to ~100 Hz; sub-ms
      // periods would need overlapping/queued scheduling instead.
      int64_t now_ns = phc_now_ns_();
      int64_t period_ns = static_cast<int64_t>(1e9 / trigger_rate_hz_);
      int64_t execute_time_ns = now_ns + period_ns / 2 + ptp_offset_ns_;

      auto nm = m_pSystem_->GetTLSystemNodeMap();
      Arena::SetNodeValue<int64_t>(nm, "ActionCommandExecuteTime", execute_time_ns);
      Arena::ExecuteNode(nm, "ActionCommandFireCommand");

      std_msgs::msg::UInt64 hb;
      hb.data = ++fire_count_;
      m_pub_->publish(hb);
    } catch (std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to fire action command\n%s",
                  e.what());
    }
  }

  static constexpr int64_t kMaxHostOffsetNs = 10000000;  // 10 ms

  double  trigger_rate_hz_;
  int64_t ptp_offset_ns_;
  std::string ptp_interface_;
  int phc_fd_ = -1;
  clockid_t phc_clock_ = 0;
  uint64_t fire_count_ = 0;

  Arena::ISystem* m_pSystem_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_timer_;
  rclcpp::TimerBase::SharedPtr m_offset_timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(HighSpeedTriggerNode)
