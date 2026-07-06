#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <atomic>
#include <memory>
#include <string>
#include <thread>

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  explicit ArenaCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("arena_camera_node", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    m_running_ = false;
    // The grab thread parks in a long GetImage() call; stopping the stream
    // makes it return so the thread can observe m_running_ and exit.
    if (m_pDevice) {
      try { m_pDevice->StopStream(); } catch (...) {}
    }
    if (m_grab_thread_.joinable()) m_grab_thread_.join();
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); }
  void log_info(std::string msg)  { RCLCPP_INFO(this->get_logger(), msg.c_str()); }
  void log_warn(std::string msg)  { RCLCPP_WARN(this->get_logger(), msg.c_str()); }
  void log_err(std::string msg)   { RCLCPP_ERROR(this->get_logger(), msg.c_str()); }

 private:
  // ---- Arena SDK -----------------------------------------------------------
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  // ---- Publishers / publisher QoS ----------------------------------------
  rclcpp::QoS                                           pub_qos_{1};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr   m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  // ---- Parameters ----------------------------------------------------------
  std::string serial_;
  bool        is_passed_serial_;

  std::string topic_;

  size_t width_;
  bool   is_passed_width;

  size_t height_;
  bool   is_passed_height;

  double gain_;
  bool   is_passed_gain_;

  double exposure_time_;
  bool   is_passed_exposure_time_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool        is_passed_pixelformat_ros_;

  // Trigger selection: "continuous" (free-run), "encoder", or "action" (PTP sync)
  std::string trigger_source_;
  bool trigger_mode_activated_;  // legacy bool; true maps to encoder
  int encoder_divider_;

  // PTP + scheduled action command synchronization
  bool   ptp_enable_;
  bool   trigger_coordinator_;
  double trigger_rate_hz_;
  rclcpp::TimerBase::SharedPtr m_action_command_timer_;

  // Background image grab loop
  std::thread       m_grab_thread_;
  std::atomic<bool> m_running_{true};

  std::string pub_qos_history_;
  bool        is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool   is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool        is_passed_pub_qos_reliability_;

  std::string frame_id_;

  std::string camera_type_;

  // ---- Private methods -----------------------------------------------------
  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  Arena::IDevice* create_device_ros_();

  // Camera configuration
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_trigger_mode_();
  void set_nodes_ptp_();
  void set_nodes_test_pattern_image_();

  // Scheduled action command coordinator (fires the synchronized trigger)
  void setup_action_command_coordinator_();
  void fire_scheduled_action_command_();

  // Streaming
  void publish_images_();

  // Trigger service (low-rate, not part of the pipeline)
  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
};
