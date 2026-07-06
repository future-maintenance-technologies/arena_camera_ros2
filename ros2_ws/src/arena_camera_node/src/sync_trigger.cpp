// Standalone synchronized-trigger broadcaster.
//
// Owns no camera. It opens an Arena system and, on a timer, broadcasts a
// PTP-scheduled action command so every camera in `trigger_source:=action`
// mode exposes at the same PTP instant. Because it holds no device and never
// starts a stream, it restarts almost instantly: if it dies, the cameras
// simply stop receiving triggers until it comes back, and resume in sync once
// it does. A heartbeat topic lets a sensor monitor watch and restart it.

#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "ArenaApi.h"

using namespace std::chrono_literals;

class SyncTriggerNode : public rclcpp::Node
{
 public:
  SyncTriggerNode() : Node("sync_trigger")
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    trigger_rate_hz_ = this->declare_parameter("trigger_rate_hz", 10.0);
    // Calibration knob: constant offset (ns) between this PC's system clock and
    // the cameras' PTP timebase. 0 when the grandmaster serves PC system time;
    // tune on the rig if scheduled times land in the cameras' past/future.
    ptp_offset_ns_ = this->declare_parameter<int64_t>("ptp_offset_ns", 0);

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
        std::bind(&SyncTriggerNode::fire_, this));

    RCLCPP_INFO(this->get_logger(),
                "sync_trigger broadcasting action commands at %.2f Hz",
                trigger_rate_hz_);
  }

  ~SyncTriggerNode()
  {
    if (m_pSystem_) Arena::CloseSystem(m_pSystem_);
  }

 private:
  void fire_()
  {
    try {
      // Read "now" in the cameras' PTP timebase (the grandmaster = this PC's
      // system clock) and schedule the capture half a period ahead.
      // ponytail: half-period lead works for rates up to ~100 Hz; sub-ms
      // periods would need overlapping/queued scheduling instead.
      int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
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

  double  trigger_rate_hz_;
  int64_t ptp_offset_ns_;
  uint64_t fire_count_ = 0;

  Arena::ISystem* m_pSystem_ = nullptr;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncTriggerNode>());
  rclcpp::shutdown();
  return 0;
}
