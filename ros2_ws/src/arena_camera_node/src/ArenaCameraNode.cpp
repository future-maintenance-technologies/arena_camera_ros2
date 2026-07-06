// clang-format off
#include <algorithm>  // std::any_of
#include <cstring>  // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>

// ROS
#include "rmw/types.h"
#include <rclcpp_components/register_node_macro.hpp>

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

// ---------------------------------------------------------------------------
// Process-wide Arena::ISystem singleton
//
// Arena::OpenSystem() must be called only once per process — calling it
// multiple times (e.g. from different composable node instances) throws.
// This helper keeps a weak_ptr so the real shared_ptr is reference-counted
// across all ArenaCameraNode instances; CloseSystem is called only when the
// last instance is destroyed.
// ---------------------------------------------------------------------------
static std::shared_ptr<Arena::ISystem> get_arena_system()
{
  static std::weak_ptr<Arena::ISystem> s_weak;
  static std::mutex s_mutex;

  std::lock_guard<std::mutex> lock(s_mutex);
  auto system = s_weak.lock();
  if (!system) {
    system = std::shared_ptr<Arena::ISystem>(
      Arena::OpenSystem(),
      [](Arena::ISystem* p) {
        if (p) Arena::CloseSystem(p);
      }
    );
    s_weak = system;
  }
  return system;
}

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    nextParameterToDeclare = "serial";
    serial_ = this->declare_parameter("serial", "");
    is_passed_serial_ = serial_ != "";
    log_info(serial_);

    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    log_info("Received gain: " + std::to_string(gain_));
    is_passed_gain_ = gain_ >= 0;

    // log_info(std::to_string(gain_));

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    // log_info(std::to_string(exposure_time_));

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);
    // no need to is_passed_trigger_mode_ because it is already a boolean

    nextParameterToDeclare = "trigger_source";
    trigger_source_ = this->declare_parameter("trigger_source", std::string(""));
    // Back-compat: legacy `trigger_mode:=true` selects the encoder trigger.
    if (trigger_source_.empty()) {
      trigger_source_ = trigger_mode_activated_ ? "encoder" : "continuous";
    }
    if (trigger_source_ != "continuous" && trigger_source_ != "encoder" &&
        trigger_source_ != "action") {
      throw std::invalid_argument(
          "trigger_source must be one of: continuous, encoder, action");
    }

    nextParameterToDeclare = "encoder_divider";
    encoder_divider_ = this->declare_parameter("encoder_divider", 65535);

    nextParameterToDeclare = "ptp_enable";
    ptp_enable_ = this->declare_parameter("ptp_enable", true);

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");
    // no need to is_passed_topic_

    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

    nextParameterToDeclare = "frame_id";
    frame_id_ = this->declare_parameter("frame_id", "camera_frame");

    nextParameterToDeclare = "camera_type";
    camera_type_ = this->declare_parameter("camera_type", "");
    if (camera_type_ == "") {
      throw std::invalid_argument("Camera type must be provided");
    }
    
  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Obtain the process-wide Arena::ISystem singleton.
  // Arena::OpenSystem() may only be called once per process; subsequent
  // composable node instances reuse the same system and CloseSystem is
  // called only when the last shared_ptr owner is destroyed.
  m_pSystem = get_arena_system();

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has ihese defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  pub_qos_ = pub_qos;

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, pub_qos_);
  log_info(std::string("Publishing raw Image on: ") + topic_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // Wait until the *intended* device is present before connecting. When a
  // serial is provided we must poll for that specific device to turn up;
  // otherwise any discovered device will do. Until then we leave the timer
  // running and try again on the next tick (rather than connecting to the
  // wrong camera or aborting because the target serial isn't found yet).
  bool device_available;
  if (is_passed_serial_) {
    device_available = std::any_of(
        device_infos.begin(), device_infos.end(),
        [this](Arena::DeviceInfo& info) {
          return serial_ == std::string(info.SerialNumber().c_str());
        });
    if (!device_available) {
      log_info("Waiting for arena camera with serial " + serial_ +
               " to be connected...");
    }
  } else {
    device_available = device_infos.size() > 0;
    if (!device_available) {
      log_info("No arena camera is connected. Waiting for device(s)...");
    }
  }

  if (device_available) {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();
  m_pDevice->StartStream();

  // Grab images on a dedicated thread so the executor stays free for the
  // trigger service. In action mode the camera waits for the broadcast action
  // commands fired by the separate high_speed_trigger_node.
  m_grab_thread_ = std::thread(&ArenaCameraNode::publish_images_, this);
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok() && m_running_) {
    try {
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      // std::cout << "error 1" << "\n";
      pImage = m_pDevice->GetImage(999999999999);  // time before timeout
      // std::cout << "error 2" << "\n";
      msg_form_image_(pImage, *p_image_msg);
      // std::cout << "error 3" << "\n";

      m_pub_->publish(std::move(p_image_msg));

      log_debug(std::string("image ") + std::to_string(pImage->GetFrameId()) +
                " published to " + topic_);
      this->m_pDevice->RequeueBuffer(pImage);

    } catch (std::exception& e) {
      if (pImage) {
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
        log_warn(std::string("Exception occurred while publishing an image\n") +
                 e.what());
      }
    }
  };
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = frame_id_;
    // image_msg.header.frame_id = std::to_string(pImage->GetFrameId());

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], x,
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000); 
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  // log_info(std::string("here1"));
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {

    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }
  // log_info(std::string("here2"));
  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  // log_info(std::string("here3"));
  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  set_nodes_roi_();
  set_nodes_gain_();
  set_nodes_pixelformat_();
  set_nodes_trigger_mode_();
  set_nodes_ptp_();
  set_nodes_exposure_();

  // Configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  // The encoder trigger needs TriggerSelector LineStart, which is not exposed
  // via the SDK and must come from the camera's onboard UserSet1. Other modes
  // (continuous, action) load the Default profile.
  if (trigger_source_ == "encoder") {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "UserSet1");
  } else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  }

  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {

    auto nodemap = m_pDevice->GetNodeMap();
    log_info(std::string("\tExposureTime set to ") + std::to_string(exposure_time_));
    if (camera_type_ == "high_speed") {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
      log_info("\tExposureAuto set to Off");
    }
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
    log_info(std::string("\tExposureTime set to ") + std::to_string(exposure_time_));
  }
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Trigger mode must be configured before the stream starts; it cannot be
  // toggled while the device is streaming.
  if (trigger_source_ == "encoder") {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    //NEED TO MOVE TO YAML FILE
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "AcquisitionMode", "Continuous");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "EncoderSelector","Encoder0");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "EncoderSourceA","Line3");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "EncoderSourceB","Line2");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "EncoderMode", "FourPhase");
    Arena::SetNodeValue<int64_t>(nodemap, "EncoderDivider", encoder_divider_);
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "EncoderOutputMode","Motion");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Encoder0");
    // Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerActivation","RisingEdge");

    log_warn("\ttrigger_source=encoder (quadrature encoder on Line2/Line3)");
  }
  // PTP-synchronized capture: every camera waits for a broadcast action
  // command and exposes at the same scheduled PTP time. See
  // Cpp_ScheduledActionCommands SDK example.
  else if (trigger_source_ == "action") {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "AcquisitionMode", "Continuous");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector", "FrameStart");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Action0");

    // Accept action commands without holding device control, and match the
    // keys the coordinator broadcasts with.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ActionUnconditionalMode", "On");
    Arena::SetNodeValue<int64_t>(nodemap, "ActionSelector", 0);
    Arena::SetNodeValue<int64_t>(nodemap, "ActionDeviceKey", 1);
    Arena::SetNodeValue<int64_t>(nodemap, "ActionGroupKey", 1);
    Arena::SetNodeValue<int64_t>(nodemap, "ActionGroupMask", 1);

    log_warn("\ttrigger_source=action (PTP scheduled action command)");
  }
  // continuous / free-run
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
    log_warn("\ttrigger_source=continuous (free-run)");
  }
}

void ArenaCameraNode::set_nodes_ptp_()
{
  // Keep PTP enabled by default so device image timestamps share the
  // grandmaster clock and stay comparable across cameras - this is what makes
  // both the synchronized and the free-run baseline recordings measurable.
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<bool>(nodemap, "PtpEnable", ptp_enable_);
  Arena::SetNodeValue<bool>(nodemap, "PtpSlaveOnly", ptp_enable_);
  log_info(std::string("\tPtpEnable set to ") + (ptp_enable_ ? "true" : "false"));
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}

RCLCPP_COMPONENTS_REGISTER_NODE(ArenaCameraNode)
// clang-format on