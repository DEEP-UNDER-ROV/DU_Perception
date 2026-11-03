#ifndef ROV_VISION__DEPTH_ESTIMATION_HPP_
#define ROV_VISION__DEPTH_ESTIMATION_HPP_

#include <atomic>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

namespace rov_vision {

struct Intrinsics {
  double fx{1}, fy{1}, cx{0}, cy{0}, k1{0}, k2{0}, p1{0}, p2{0};
  cv::Mat K() const {
    return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }
  cv::Mat D() const { return (cv::Mat_<double>(1, 4) << k1, k2, p1, p2); }
  bool has_distortion() const {
    return k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0;
  }
};

class DepthEstimation : public rclcpp::Node {
 public:
  using Image = sensor_msgs::msg::Image;

  explicit DepthEstimation(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  struct IrFrame {
    cv::Mat img;
    double t{0};
    std_msgs::msg::Header header;
    bool valid{false};
  };

  template <typename T>
  T P(const std::string& name, const T& def) {
    return declare_parameter<T>(name, def);
  }

  // parameters / control
  void declare_all_parameters();
  rcl_interfaces::msg::SetParametersResult on_params(
      const std::vector<rclcpp::Parameter>& ps);
  void set_enabled(bool on);

  // stereo
  void setup_stereo();
  void ensure_rectify_maps(const cv::Size& sz);
  void on_infra(bool left, const Image::ConstSharedPtr& msg);
  void process_stereo(const IrFrame& l, const IrFrame& r);

  // visualisation
  cv::Mat colorize(const cv::Mat& depth) const;
  void build_legend();
  void apply_legend(cv::Mat& img) const;

  // params
  std::string topic_ir1_, topic_ir2_, topic_viz_, topic_depth_, topic_enable_;
  bool qos_reliable_{false};
  int qos_depth_{2};
  int cam_w_{848}, cam_h_{480};
  Intrinsics cam_l_, cam_r_;
  double baseline_{0.1};
  double proc_scale_{0.5};
  int num_disp_{208}, min_disp_{0}, block_size_{7}, p1_scale_{8}, p2_scale_{32};
  int uniq_{10}, speckle_win_{100}, speckle_rng_{2}, disp12_{1},
      prefilter_cap_{31}, median_k_{3};
  std::string sgbm_mode_;
  double slop_{0.02};
  bool pad_left_{true}, crop_strip_{false};
  double min_m_{0.1}, max_m_{3.0};
  int cmap_{cv::COLORMAP_JET};
  bool legend_enabled_{true};
  int legend_ticks_{6}, legend_w_{84};
  cv::Size viz_size_;
  bool publish_depth_{false};

  // runtime-changeable
  std::atomic<bool> enabled_{true};

  // stereo state
  cv::Ptr<cv::StereoSGBM> matcher_;
  cv::Size maps_size_;
  cv::Mat map1_l_, map2_l_, map1_r_, map2_r_;
  std::mutex ir_mtx_;
  IrFrame ir_l_, ir_r_;

  // legend
  cv::Rect legend_rect_;
  cv::Mat legend_layer_, legend_mask_;

  // ROS
  rclcpp::CallbackGroup::SharedPtr cb_stereo_, cb_ctrl_;
  rclcpp::Subscription<Image>::SharedPtr sub_ir1_, sub_ir2_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Publisher<Image>::SharedPtr pub_viz_, pub_depth_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace rov_vision

#endif  // ROV_VISION__DEPTH_ESTIMATION_HPP_
