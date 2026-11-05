#include "rov_vision/depth_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace rov_vision {

namespace {

std::string fmt_depth(double m) {
  char b[32];
  if (m < 1.0) {
    std::snprintf(b, sizeof b, "%.0f cm", m * 100.0);
  } else {
    std::snprintf(b, sizeof b, "%.1f m", m);
  }
  return b;
}

}  // namespace

DepthEstimation::DepthEstimation(const rclcpp::NodeOptions& options)
    : Node("depth_estimation", options) {
  declare_all_parameters();
  setup_stereo();
  build_legend();

  cb_stereo_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
  if (qos_reliable_) {
    qos.reliable();
  } else {
    qos.best_effort();
  }

  rclcpp::SubscriptionOptions o_stereo;
  o_stereo.callback_group = cb_stereo_;

  sub_ir1_ = create_subscription<Image>(
      topic_ir1_, qos, [this](Image::ConstSharedPtr m) { on_infra(true, m); },
      o_stereo);
  sub_ir2_ = create_subscription<Image>(
      topic_ir2_, qos, [this](Image::ConstSharedPtr m) { on_infra(false, m); },
      o_stereo);

  pub_viz_ = create_publisher<Image>(topic_viz_, 1);
  if (publish_depth_) {
    pub_depth_ = create_publisher<Image>(topic_depth_, 1);
  }

  param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& ps) {
        return on_params(ps);
      });

  RCLCPP_INFO(get_logger(),
              "depth_estimation up. enabled=%d, lazy=%d, infra1=%s, infra2=%s, "
              "viz=%s (%dx%d)",
              enabled_.load(), lazy_, topic_ir1_.c_str(), topic_ir2_.c_str(),
              topic_viz_.c_str(), viz_size_.width, viz_size_.height);
}

void DepthEstimation::declare_all_parameters() {
  topic_ir1_ =
      P<std::string>("topics.infra1", "/camera/camera/infra1/image_rect_raw");
  topic_ir2_ =
      P<std::string>("topics.infra2", "/camera/camera/infra2/image_rect_raw");
  topic_viz_ = P<std::string>("topics.depth_viz", "/depth_estimation/image_viz");
  topic_depth_ = P<std::string>("topics.depth", "/depth_estimation/depth");
  qos_reliable_ = P<bool>("qos.reliable", false);
  qos_depth_ = std::max<int>(1, P<int>("qos.depth", 2));

  cam_w_ = P<int>("camera.width", 848);
  cam_h_ = P<int>("camera.height", 480);
  cam_l_ = declare_intrinsics("camera.left");
  cam_r_ = declare_intrinsics("camera.right");

  const auto t_lr = P<std::vector<double>>("camera.T_left_right", {});
  if (t_lr.size() != 16) {
    throw std::runtime_error("camera.T_left_right must be a row-major 4x4 (16 values)");
  }
  baseline_ = std::hypot(t_lr[3], t_lr[7], t_lr[11]);
  if (baseline_ <= 0.0) {
    throw std::runtime_error("camera.T_left_right has zero baseline");
  }
  if (std::abs(t_lr[7]) > 0.01 * baseline_ || std::abs(t_lr[11]) > 0.01 * baseline_) {
    RCLCPP_WARN(get_logger(),
                "camera.T_left_right is not a pure x-translation, images may not be rectified");
  }
  if (cam_w_ <= 0 || cam_h_ <= 0) {
    throw std::runtime_error("camera.width/height must be > 0");
  }

  proc_scale_ = std::clamp(P<double>("stereo.processing_scale", 0.5), 0.1, 1.0);
  num_disp_ = P<int>("stereo.num_disparities", 208);
  min_disp_ = P<int>("stereo.min_disparity", 0);
  block_size_ = P<int>("stereo.block_size", 7);
  sgbm_mode_ = P<std::string>("stereo.mode", "sgbm");
  p1_scale_ = P<int>("stereo.p1_scale", 8);
  p2_scale_ = P<int>("stereo.p2_scale", 32);
  uniq_ = P<int>("stereo.uniqueness_ratio", 10);
  speckle_win_ = P<int>("stereo.speckle_window_size", 100);
  speckle_rng_ = P<int>("stereo.speckle_range", 2);
  disp12_ = P<int>("stereo.disp12_max_diff", 1);
  prefilter_cap_ = P<int>("stereo.pre_filter_cap", 31);
  median_k_ = P<int>("stereo.median_ksize", 3);
  slop_ = P<double>("stereo.sync_slop_sec", 0.02);
  pad_left_ = P<bool>("stereo.pad_left", true);
  crop_strip_ = P<bool>("stereo.crop_blank_strip", false);

  min_m_ = P<double>("depth.min_m", 0.10);
  max_m_ = P<double>("depth.max_m", 3.0);
  if (min_m_ < 0.0 || max_m_ <= min_m_) {
    throw std::runtime_error("need 0 <= depth.min_m < depth.max_m");
  }
  const std::string cmap = P<std::string>("depth.colormap", "jet");
  cmap_ = (cmap == "turbo") ? cv::COLORMAP_TURBO : cv::COLORMAP_JET;
  publish_depth_ = P<bool>("depth.publish_raw", false);

  legend_enabled_ = P<bool>("legend.enabled", true);
  legend_ticks_ = std::max(2, P<int>("legend.ticks", 6));
  legend_w_ = std::max(60, P<int>("legend.width", 84));

  viz_size_ = cv::Size(std::max(64, P<int>("viz.width", 848)),
                       std::max(64, P<int>("viz.height", 480)));

  enabled_ = P<bool>("enabled", true);
  lazy_ = P<bool>("lazy", true);
}

Intrinsics DepthEstimation::declare_intrinsics(const std::string& prefix) {
  Intrinsics i;
  i.fx = P<double>(prefix + ".fx", 0.0);
  i.fy = P<double>(prefix + ".fy", 0.0);
  i.cx = P<double>(prefix + ".cx", 0.0);
  i.cy = P<double>(prefix + ".cy", 0.0);
  i.k1 = P<double>(prefix + ".k1", 0.0);
  i.k2 = P<double>(prefix + ".k2", 0.0);
  i.p1 = P<double>(prefix + ".p1", 0.0);
  i.p2 = P<double>(prefix + ".p2", 0.0);
  if (i.fx <= 0.0 || i.fy <= 0.0 || i.cx <= 0.0 || i.cy <= 0.0) {
    throw std::runtime_error(prefix + " intrinsics are not set");
  }
  return i;
}

rcl_interfaces::msg::SetParametersResult DepthEstimation::on_params(
    const std::vector<rclcpp::Parameter>& ps) {
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (const auto& p : ps) {
    const auto& n = p.get_name();
    if (n == "enabled") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        res.successful = false;
        res.reason = "enabled must be a bool";
        return res;
      }
    } else if (n != "use_sim_time") {
      res.successful = false;
      res.reason = "'" + n + "' can only be set at startup";
      return res;
    }
  }
  for (const auto& p : ps) {
    if (p.get_name() == "enabled") {
      set_enabled(p.as_bool());
    }
  }
  return res;
}

void DepthEstimation::set_enabled(bool on) {
  if (enabled_.exchange(on) == on) {
    return;
  }
  if (on) {
    std::lock_guard<std::mutex> lk(ir_mtx_);
    ir_l_.valid = ir_r_.valid = false;
  }
  RCLCPP_INFO(get_logger(), "depth estimation %s", on ? "enabled" : "disabled");
}

void DepthEstimation::setup_stereo() {
  num_disp_ = std::max(16, ((num_disp_ + 15) / 16) * 16);
  if (block_size_ % 2 == 0) {
    ++block_size_;
  }
  block_size_ = std::clamp(block_size_, 1, 21);
  if (median_k_ != 0 && median_k_ != 3 && median_k_ != 5) {
    median_k_ = 3;
  }

  int mode = cv::StereoSGBM::MODE_SGBM;
  if (sgbm_mode_ == "sgbm_3way") {
    mode = cv::StereoSGBM::MODE_SGBM_3WAY;
  } else if (sgbm_mode_ == "hh") {
    mode = cv::StereoSGBM::MODE_HH;
  } else if (sgbm_mode_ == "hh4") {
    mode = cv::StereoSGBM::MODE_HH4;
  }

  const int cn = 1;
  matcher_ = cv::StereoSGBM::create(
      min_disp_, num_disp_, block_size_,
      p1_scale_ * cn * block_size_ * block_size_,
      p2_scale_ * cn * block_size_ * block_size_, disp12_, prefilter_cap_,
      uniq_, speckle_win_, speckle_rng_, mode);

  const double min_depth_meas = cam_l_.fx * proc_scale_ * baseline_ / num_disp_;
  RCLCPP_INFO(
      get_logger(),
      "SGBM: scale=%.2f, numDisp=%d, block=%d -> nearest measurable ~%.0f cm",
      proc_scale_, num_disp_, block_size_, min_depth_meas * 100.0);
  if (min_depth_meas > min_m_) {
    RCLCPP_WARN(get_logger(),
                "depth.min_m=%.2f m is closer than the matcher can see (%.2f "
                "m). Raise "
                "stereo.num_disparities or stereo.processing_scale.",
                min_m_, min_depth_meas);
  }
}

void DepthEstimation::ensure_rectify_maps(const cv::Size& sz) {
  if (sz == maps_size_) {
    return;
  }
  maps_size_ = sz;
  map1_l_.release();
  map2_l_.release();
  map1_r_.release();
  map2_r_.release();

  if (sz.width != cam_w_ || sz.height != cam_h_) {
    RCLCPP_WARN(
        get_logger(),
        "image is %dx%d but camera.width/height is %dx%d: scaling intrinsics",
        sz.width, sz.height, cam_w_, cam_h_);
  }
  const double sx = sz.width / static_cast<double>(cam_w_);
  const double sy = sz.height / static_cast<double>(cam_h_);
  auto scaled = [&](Intrinsics i) {
    i.fx *= sx;
    i.cx *= sx;
    i.fy *= sy;
    i.cy *= sy;
    return i;
  };
  const Intrinsics L = scaled(cam_l_), R = scaled(cam_r_);
  if (L.has_distortion()) {
    cv::initUndistortRectifyMap(L.K(), L.D(), cv::Mat(), L.K(), sz, CV_16SC2,
                                map1_l_, map2_l_);
  }
  if (R.has_distortion()) {
    cv::initUndistortRectifyMap(R.K(), R.D(), cv::Mat(), L.K(), sz, CV_16SC2,
                                map1_r_, map2_r_);
  }
}

bool DepthEstimation::has_consumers() const {
  return pub_viz_->get_subscription_count() > 0 ||
         (pub_depth_ && pub_depth_->get_subscription_count() > 0);
}

void DepthEstimation::on_infra(bool left, const Image::ConstSharedPtr& msg) {
  if (!enabled_ || (lazy_ && !has_consumers())) {
    return;
  }  

  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image.clone();
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "infra convert: %s",
                          e.what());
    return;
  }

  IrFrame l, r;
  bool ready = false;
  {
    std::lock_guard<std::mutex> lk(ir_mtx_);
    IrFrame& slot = left ? ir_l_ : ir_r_;
    slot.img = img;
    slot.t = rclcpp::Time(msg->header.stamp).seconds();
    slot.header = msg->header;
    slot.valid = true;
    if (ir_l_.valid && ir_r_.valid) {
      const double dt = ir_l_.t - ir_r_.t;
      if (std::abs(dt) <= slop_) {
        l = ir_l_;
        r = ir_r_;
        ir_l_.valid = ir_r_.valid = false;
        ready = true;
      } else if (dt < 0) {
        ir_l_.valid = false;  // left is older, drop it
      } else {
        ir_r_.valid = false;
      }
    }
  }
  if (ready) {
    process_stereo(l, r);
  }
}

void DepthEstimation::process_stereo(const IrFrame& l, const IrFrame& r) {
  if (l.img.size() != r.img.size()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                          "infra1/infra2 sizes differ");
    return;
  }
  ensure_rectify_maps(l.img.size());

  cv::Mat left = l.img, right = r.img, tmp;
  if (!map1_l_.empty()) {
    cv::remap(left, tmp, map1_l_, map2_l_, cv::INTER_LINEAR);
    left = tmp;
  }
  if (!map1_r_.empty()) {
    cv::remap(right, tmp, map1_r_, map2_r_, cv::INTER_LINEAR);
    right = tmp;
  }

  if (proc_scale_ < 0.999) {
    cv::resize(left, left, cv::Size(), proc_scale_, proc_scale_,
               cv::INTER_AREA);
    cv::resize(right, right, cv::Size(), proc_scale_, proc_scale_,
               cv::INTER_AREA);
  }

  const int proc_cols = left.cols;

  const int pad = pad_left_ ? num_disp_ : 0;
  if (pad > 0) {
    cv::copyMakeBorder(left, tmp, 0, 0, pad, 0, cv::BORDER_REPLICATE);
    left = tmp;
    cv::Mat tmp2;
    cv::copyMakeBorder(right, tmp2, 0, 0, pad, 0, cv::BORDER_REPLICATE);
    right = tmp2;
  }

  cv::Mat disp16, disp;
  matcher_->compute(left, right, disp16);
  if (pad > 0) {
    disp16 = disp16(cv::Rect(pad, 0, disp16.cols - pad, disp16.rows)).clone();
  }
  disp16.convertTo(disp, CV_32F, 1.0 / 16.0);

  const double f = cam_l_.fx * (static_cast<double>(proc_cols) / cam_w_);
  const float fb = static_cast<float>(f * baseline_);
  cv::Mat depth(disp.size(), CV_32F);
  for (int y = 0; y < disp.rows; ++y) {
    const float* pd = disp.ptr<float>(y);
    float* pz = depth.ptr<float>(y);
    for (int x = 0; x < disp.cols; ++x) {
      const float d = pd[x];
      pz[x] = (d >= static_cast<float>(min_disp_) && d > 0.1f) ? fb / d : 0.f;
    }
  }
  if (median_k_ >= 3) {
    cv::medianBlur(depth, depth, median_k_);
  }

  if (pub_depth_) {
    pub_depth_->publish(
        *cv_bridge::CvImage(l.header, "32FC1", depth).toImageMsg());
  }

  if (crop_strip_ && !pad_left_) {
    const int c = std::min(num_disp_ + min_disp_, depth.cols / 2);
    const int w2 = depth.cols - c;
    const int h2 = std::min(
        depth.rows, static_cast<int>(std::lround(w2 * viz_size_.height /
                                                 double(viz_size_.width))));
    const int y0 = (depth.rows - h2) / 2;
    depth = depth(cv::Rect(c, y0, w2, h2)).clone();
  }

  cv::Mat viz = colorize(depth);
  if (viz.size() != viz_size_) {
    cv::resize(viz, viz, viz_size_, 0, 0, cv::INTER_LINEAR);
  }
  if (legend_enabled_) {
    apply_legend(viz);
  }

  pub_viz_->publish(*cv_bridge::CvImage(l.header, "bgr8", viz).toImageMsg());
}

cv::Mat DepthEstimation::colorize(const cv::Mat& depth) const {
  cv::Mat gray(depth.size(), CV_8U), invalid(depth.size(), CV_8U);
  const float lo = static_cast<float>(min_m_),
              span = static_cast<float>(max_m_ - min_m_);
  for (int y = 0; y < depth.rows; ++y) {
    const float* pz = depth.ptr<float>(y);
    uchar* pg = gray.ptr<uchar>(y);
    uchar* pi = invalid.ptr<uchar>(y);
    for (int x = 0; x < depth.cols; ++x) {
      const float z = pz[x];
      if (z <= 0.f || !std::isfinite(z)) {
        pg[x] = 0;
        pi[x] = 255;
      } else {
        const float t = std::clamp((z - lo) / span, 0.f, 1.f);
        pg[x] = static_cast<uchar>(std::lround(255.f * (1.f - t)));
        pi[x] = 0;
      }
    }
  }
  cv::Mat color;
  cv::applyColorMap(gray, color, cmap_);
  color.setTo(cv::Scalar(0, 0, 0), invalid);
  return color;
}

void DepthEstimation::build_legend() {
  const int pw = std::min(legend_w_, viz_size_.width / 2);
  const int ph = viz_size_.height - 16;
  if (ph < 100) {
    legend_enabled_ = false;
    return;
  }
  legend_rect_ = cv::Rect(viz_size_.width - pw - 8, 8, pw, ph);
  legend_layer_ = cv::Mat::zeros(ph, pw, CV_8UC3);

  cv::putText(legend_layer_, "Depth", cv::Point(8, 16),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);

  const int bx = 8, by = 28, bw = 16, bh = ph - by - 10;
  cv::Mat ramp(bh, 1, CV_8U);
  for (int i = 0; i < bh; ++i) {
    ramp.at<uchar>(i, 0) =
        static_cast<uchar>(std::lround(255.0 * (1.0 - i / (bh - 1.0))));
  }
  cv::Mat ramp_c, bar;
  cv::applyColorMap(ramp, ramp_c, cmap_);
  cv::resize(ramp_c, bar, cv::Size(bw, bh), 0, 0, cv::INTER_NEAREST);
  bar.copyTo(legend_layer_(cv::Rect(bx, by, bw, bh)));
  cv::rectangle(legend_layer_, cv::Rect(bx, by, bw, bh),
                cv::Scalar(255, 255, 255), 1);

  for (int i = 0; i < legend_ticks_; ++i) {
    const double frac = i / static_cast<double>(legend_ticks_ - 1);
    const int y = by + static_cast<int>(std::lround(frac * (bh - 1)));
    const double d = min_m_ + frac * (max_m_ - min_m_);
    cv::line(legend_layer_, cv::Point(bx + bw, y), cv::Point(bx + bw + 4, y),
             cv::Scalar(255, 255, 255), 1);
    cv::putText(legend_layer_, fmt_depth(d), cv::Point(bx + bw + 7, y + 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);
  }

  cv::Mat g;
  cv::cvtColor(legend_layer_, g, cv::COLOR_BGR2GRAY);
  cv::threshold(g, legend_mask_, 0, 255, cv::THRESH_BINARY);
}

void DepthEstimation::apply_legend(cv::Mat& img) const {
  cv::Mat roi = img(legend_rect_);
  roi.convertTo(roi, -1, 0.35, 0);
  legend_layer_.copyTo(roi, legend_mask_);
}

}  // namespace rov_vision
