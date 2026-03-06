#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <atomic>
#include <chrono>      // chrono_literals
#include <functional>  // std::bind, std::placeholders
#include <memory>
#include <thread>

#include "frame_queue.hpp"
#include "gpu_compressor.hpp"

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/trigger.hpp>

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info(std::string("Destroying \"") + this->get_name() + "\" node");
    running_.store(false);
    frame_queue_.shutdown();
    if (producer_thread_.joinable()) producer_thread_.join();
    if (consumer_thread_.joinable()) consumer_thread_.join();
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); }
  void log_info(std::string msg)  { RCLCPP_INFO(this->get_logger(), msg.c_str()); }
  void log_warn(std::string msg)  { RCLCPP_WARN(this->get_logger(), msg.c_str()); }
  void log_err(std::string msg)   { RCLCPP_ERROR(this->get_logger(), msg.c_str()); }

 private:
  // ---- Arena SDK -----------------------------------------------------------
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  // ---- ROS publishers / timers / services ----------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         m_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_compressed_;
  rclcpp::TimerBase::SharedPtr   m_wait_for_device_timer_callback_;
  rclcpp::TimerBase::SharedPtr   m_telemetry_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  // ---- GPU compression -----------------------------------------------------
  std::unique_ptr<GpuCompressor> compressor_;

  // ---- Producer / consumer pipeline ----------------------------------------
  std::thread        producer_thread_;
  std::thread        consumer_thread_;
  std::atomic<bool>  running_{true};
  FrameQueue<>       frame_queue_;          // bounded SPSC queue (capacity 4)
  size_t             frame_size_bytes_ = 0; // PayloadSize from device

  // ---- Telemetry (atomic for cross-thread reads) ---------------------------
  std::atomic<uint64_t> frames_received_{0};
  std::atomic<uint64_t> frames_published_{0};
  std::atomic<uint64_t> frames_compressed_{0};
  std::atomic<uint64_t> compression_failures_{0};

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

  bool trigger_mode_activated_;

  std::string pub_qos_history_;
  bool        is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool   is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool        is_passed_pub_qos_reliability_;

  std::string frame_id_;

  // ---- Private methods -----------------------------------------------------
  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();
  void telemetry_timer_callback_();

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
  void set_nodes_test_pattern_image_();

  // Pipeline threads
  void camera_producer_();
  void compress_publish_consumer_();

  // Trigger service (low-rate, not part of the pipeline)
  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Legacy helpers used by the trigger path
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
  void msg_form_compressed_image_(Arena::IImage* pImage,
                                  sensor_msgs::msg::CompressedImage& image_msg);
};
