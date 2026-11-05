#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_vision/srv/set_stream_mode.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

namespace rov_vision {

enum class StreamMode { COLOR = 0, DEPTH = 1, ALTERNATE = 2 };

class Streamer : public rclcpp::Node {
 public:
  using Image = sensor_msgs::msg::Image;
  using SetStreamMode = rov_vision::srv::SetStreamMode;
  using SteadyClock = std::chrono::steady_clock;

  explicit Streamer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct TimedFrame {
    cv::Mat img;
    SteadyClock::time_point arrived;
  };

  template <typename T>
  T P(const std::string& name, const T& def) {
    return declare_parameter<T>(name, def);
  }

  void declare_all_parameters();
  rcl_interfaces::msg::SetParametersResult on_params(
      const std::vector<rclcpp::Parameter>& ps);
  void on_set_mode(const SetStreamMode::Request::SharedPtr req,
                   SetStreamMode::Response::SharedPtr res);
  void publish_depth_request(bool needed);

  void on_color(const Image::ConstSharedPtr& msg);
  void on_depth(const Image::ConstSharedPtr& msg);
  cv::Mat to_output_size(const cv::Mat& src, int interpolation) const;

  cv::Mat fresh(std::mutex& m, const TimedFrame& f) const;
  bool selected_depth() const;
  cv::Mat compose();
  void on_timer();
  void emit_frame();
  std::string build_pipeline() const;
  void open_writer();

  std::string topic_color_, topic_depth_viz_, topic_enable_, topic_out_;
  bool qos_reliable_{true};
  int qos_depth_{2};
  bool event_driven_{true};
  double fps_{30.0};
  cv::Size out_size_;
  double stale_sec_{1.0};
  bool overlay_mode_{true}, publish_ros_image_{false};
  bool gcs_enabled_{true};
  std::string gcs_ip_, gcs_encoder_, gcs_pipeline_;
  int gcs_port_{5600}, gcs_bitrate_{2000}, gcs_keyint_{30};

  std::atomic<StreamMode> mode_{StreamMode::COLOR};
  std::atomic<double> alt_color_sec_{5.0}, alt_depth_sec_{5.0}, alt_start_{0.0};

  std::mutex color_mtx_, depth_mtx_;
  TimedFrame color_, depth_viz_;

  std::mutex writer_mtx_;
  cv::VideoWriter writer_;
  double last_open_try_{0.0};

  rclcpp::CallbackGroup::SharedPtr cb_color_, cb_depth_, cb_out_;
  rclcpp::Subscription<Image>::SharedPtr sub_color_, sub_depth_viz_;
  rclcpp::Publisher<Image>::SharedPtr pub_out_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_;
  rclcpp::Service<SetStreamMode>::SharedPtr srv_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace rov_vision
