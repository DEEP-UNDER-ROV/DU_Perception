#include "rov_vision/streamer.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace rov_vision {

namespace {

const char* mode_to_string(StreamMode m) {
  switch (m) {
    case StreamMode::COLOR:
      return "color";
    case StreamMode::DEPTH:
      return "depth";
    case StreamMode::ALTERNATE:
      return "alternate";
  }
  return "color";
}

bool parse_mode(const std::string& s, StreamMode& out) {
  if (s == "color") {
    out = StreamMode::COLOR;
    return true;
  }
  if (s == "depth") {
    out = StreamMode::DEPTH;
    return true;
  }
  if (s == "alternate") {
    out = StreamMode::ALTERNATE;
    return true;
  }
  return false;
}

double now_steady_sec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void draw_label(cv::Mat& img, const std::string& text, cv::Point org) {
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

cv::Mat placeholder(const cv::Size& size, const std::string& text) {
  cv::Mat img = cv::Mat::zeros(size, CV_8UC3);
  int base = 0;
  const cv::Size ts =
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &base);
  cv::putText(img, text, cv::Point((size.width - ts.width) / 2, size.height / 2),
              cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2,
              cv::LINE_AA);
  return img;
}

}  // namespace

Streamer::Streamer(const rclcpp::NodeOptions& options)
    : Node("streamer", options) {
  declare_all_parameters();

  cb_color_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_depth_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_out_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
  if (qos_reliable_) {
    qos.reliable();
  } else {
    qos.best_effort();
  }

  rclcpp::SubscriptionOptions o_color, o_depth;
  o_color.callback_group = cb_color_;
  o_depth.callback_group = cb_depth_;

  sub_color_ = create_subscription<Image>(
      topic_color_, qos, [this](Image::ConstSharedPtr m) { on_color(m); },
      o_color);
  sub_depth_viz_ = create_subscription<Image>(
      topic_depth_viz_, qos, [this](Image::ConstSharedPtr m) { on_depth(m); },
      o_depth);

  if (publish_ros_image_) {
    pub_out_ = create_publisher<Image>(topic_out_, 1);
  }
  if (!topic_enable_.empty()) {
    pub_enable_ = create_publisher<std_msgs::msg::Bool>(
        topic_enable_, rclcpp::QoS(1).reliable().transient_local());
    publish_depth_request(mode_.load() != StreamMode::COLOR);
  }

  srv_mode_ = create_service<SetStreamMode>(
      "~/set_stream_mode",
      [this](const SetStreamMode::Request::SharedPtr req,
             SetStreamMode::Response::SharedPtr res) { on_set_mode(req, res); },
      rclcpp::ServicesQoS(), cb_out_);

  param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& ps) {
        return on_params(ps);
      });

  if (gcs_enabled_) {
    open_writer();
  }

  timer_ = create_wall_timer(
      std::chrono::milliseconds(
          event_driven_ ? 1000 : static_cast<int>(std::lround(1000.0 / fps_))),
      [this]() { on_timer(); }, cb_out_);

  RCLCPP_INFO(get_logger(),
              "streamer up. mode=%s, color=%s, depth=%s, out=%dx%d@%.1f",
              mode_to_string(mode_.load()), topic_color_.c_str(),
              topic_depth_viz_.c_str(), out_size_.width, out_size_.height,
              fps_);
}

void Streamer::declare_all_parameters() {
  topic_color_ =
      P<std::string>("topics.color", "/camera/camera/color/image_raw");
  topic_depth_viz_ =
      P<std::string>("topics.depth_viz", "/depth_estimation/image_viz");
  topic_enable_ = P<std::string>("topics.enable", "/depth_estimation/enable");
  topic_out_ = P<std::string>("topics.output", "/streamer/image");
  qos_reliable_ = P<bool>("qos.reliable", false);
  qos_depth_ = std::max<int>(1, P<int>("qos.depth", 2));

  const std::string mode_s = P<std::string>("stream.mode", "color");
  StreamMode m = StreamMode::COLOR;
  if (!parse_mode(mode_s, m)) {
    RCLCPP_WARN(get_logger(), "unknown stream.mode '%s', using color",
                mode_s.c_str());
  }
  mode_ = m;
  alt_color_sec_ = P<double>("stream.alternate_color_sec", 5.0);
  alt_depth_sec_ = P<double>("stream.alternate_depth_sec", 5.0);
  event_driven_ = P<bool>("stream.event_driven", true);
  fps_ = std::max(1.0, P<double>("stream.fps", 15.0));
  out_size_ = cv::Size(std::max(64, P<int>("stream.output_width", 848)),
                       std::max(64, P<int>("stream.output_height", 480)));
  stale_sec_ = P<double>("stream.stale_timeout_sec", 1.0);
  overlay_mode_ = P<bool>("stream.overlay_mode_text", true);
  publish_ros_image_ = P<bool>("stream.publish_ros_image", false);

  gcs_enabled_ = P<bool>("gcs.enabled", true);
  gcs_ip_ = P<std::string>("gcs.ip", "127.0.0.1");
  gcs_port_ = P<int>("gcs.port", 5600);
  gcs_encoder_ = P<std::string>("gcs.encoder", "x264");
  gcs_bitrate_ = P<int>("gcs.bitrate_kbps", 2000);
  gcs_pipeline_ = P<std::string>("gcs.pipeline", "");

  alt_start_ = now_steady_sec();
}

// Only the stream mode and the alternate durations can change while running.
rcl_interfaces::msg::SetParametersResult Streamer::on_params(
    const std::vector<rclcpp::Parameter>& ps) {
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (const auto& p : ps) {
    const auto& n = p.get_name();
    if (n == "stream.mode") {
      StreamMode m = StreamMode::COLOR;
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
          !parse_mode(p.as_string(), m)) {
        res.successful = false;
        res.reason = "stream.mode must be one of: color, depth, alternate";
        return res;
      }
    } else if (n == "stream.alternate_color_sec" ||
               n == "stream.alternate_depth_sec") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
          p.as_double() <= 0.0) {
        res.successful = false;
        res.reason = n + " must be a double > 0";
        return res;
      }
    } else if (n != "use_sim_time") {
      res.successful = false;
      res.reason = "'" + n + "' can only be set at startup";
      return res;
    }
  }

  // Apply durations first so a mode change to "alternate" starts with the new
  // timings.
  for (const auto& p : ps) {
    if (p.get_name() == "stream.alternate_color_sec") {
      alt_color_sec_ = p.as_double();
    }
    if (p.get_name() == "stream.alternate_depth_sec") {
      alt_depth_sec_ = p.as_double();
    }
  }
  for (const auto& p : ps) {
    if (p.get_name() == "stream.mode") {
      StreamMode m = StreamMode::COLOR;
      parse_mode(p.as_string(), m);
      alt_start_ = now_steady_sec();
      mode_ = m;
      publish_depth_request(m != StreamMode::COLOR);
      RCLCPP_INFO(get_logger(), "stream mode -> %s", mode_to_string(m));
    }
  }
  return res;
}

void Streamer::on_set_mode(const SetStreamMode::Request::SharedPtr req,
                           SetStreamMode::Response::SharedPtr res) {
  StreamMode m = StreamMode::COLOR;
  switch (req->mode) {
    case SetStreamMode::Request::COLOR:
      m = StreamMode::COLOR;
      break;
    case SetStreamMode::Request::DEPTH:
      m = StreamMode::DEPTH;
      break;
    case SetStreamMode::Request::ALTERNATE:
      m = StreamMode::ALTERNATE;
      break;
    default:
      res->success = false;
      res->message = "invalid mode " + std::to_string(req->mode) +
                     " (0=color 1=depth 2=alternate)";
      return;
  }
  std::vector<rclcpp::Parameter> ps;
  if (req->color_duration > 0.0f) {
    ps.emplace_back("stream.alternate_color_sec",
                    static_cast<double>(req->color_duration));
  }
  if (req->depth_duration > 0.0f) {
    ps.emplace_back("stream.alternate_depth_sec",
                    static_cast<double>(req->depth_duration));
  }
  ps.emplace_back("stream.mode", std::string(mode_to_string(m)));

  const auto r = set_parameters_atomically(ps);
  res->success = r.successful;
  res->message = r.successful ? std::string("mode set to ") + mode_to_string(m)
                              : r.reason;
}

void Streamer::publish_depth_request(bool needed) {
  if (!pub_enable_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = needed;
  pub_enable_->publish(msg);
}

// ------------------------------------------------------------------- input
cv::Mat Streamer::to_output_size(const cv::Mat& src, int interpolation) const {
  if (src.size() == out_size_) {
    return src.clone();
  }
  cv::Mat out;
  cv::resize(src, out, out_size_, 0, 0, interpolation);
  return out;
}

void Streamer::on_color(const Image::ConstSharedPtr& msg) {
  cv::Mat out;
  try {
    out = to_output_size(cv_bridge::toCvShare(msg, "bgr8")->image,
                         cv::INTER_AREA);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                          "colour convert: %s", e.what());
    return;
  }
  {
    std::lock_guard<std::mutex> lk(color_mtx_);
    color_ = {out, SteadyClock::now()};
  }
  if (event_driven_ && !selected_depth()) {
    emit_frame();
  }
}

void Streamer::on_depth(const Image::ConstSharedPtr& msg) {
  cv::Mat out;
  try {
    out = to_output_size(cv_bridge::toCvShare(msg, "bgr8")->image,
                         cv::INTER_LINEAR);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                          "depth convert: %s", e.what());
    return;
  }
  {
    std::lock_guard<std::mutex> lk(depth_mtx_);
    depth_viz_ = {out, SteadyClock::now()};
  }
  if (event_driven_ && selected_depth()) {
    emit_frame();
  }
}

// ------------------------------------------------------------------ output
cv::Mat Streamer::fresh(std::mutex& m, const TimedFrame& f) const {
  std::lock_guard<std::mutex> lk(m);
  if (f.img.empty()) {
    return cv::Mat();
  }
  const double age =
      std::chrono::duration<double>(SteadyClock::now() - f.arrived).count();
  if (stale_sec_ > 0.0 && age > stale_sec_) {
    return cv::Mat();
  }
  return f.img.clone();
}

bool Streamer::selected_depth() const {
  const StreamMode m = mode_.load();
  if (m == StreamMode::ALTERNATE) {
    const double cs = alt_color_sec_.load(), ds = alt_depth_sec_.load();
    const double t = std::fmod(now_steady_sec() - alt_start_.load(), cs + ds);
    return t >= cs;
  }
  return m == StreamMode::DEPTH;
}

cv::Mat Streamer::compose() {
  const bool show_depth = selected_depth();

  cv::Mat frame;
  if (show_depth) {
    frame = fresh(depth_mtx_, depth_viz_);
    if (frame.empty()) {
      frame = placeholder(out_size_, "NO DEPTH SIGNAL");
    }
  } else {
    frame = fresh(color_mtx_, color_);
    if (frame.empty()) {
      frame = placeholder(out_size_, "NO COLOR SIGNAL");
    }
  }
  if (overlay_mode_) {
    draw_label(frame, show_depth ? "DEPTH" : "COLOR", cv::Point(10, 26));
  }
  return frame;
}

// Fixed-rate mode: compose + send every tick. Event-driven mode: the timer (1
// Hz) only retries opening the GStreamer pipeline; frames are sent from the
// image callbacks.
void Streamer::on_timer() {
  if (event_driven_) {
    std::lock_guard<std::mutex> lk(writer_mtx_);
    if (gcs_enabled_ && !writer_.isOpened() &&
        now_steady_sec() - last_open_try_ > 5.0) {
      open_writer();
    }
    return;
  }
  emit_frame();
}

void Streamer::emit_frame() {
  cv::Mat frame = compose();

  std::lock_guard<std::mutex> lk(writer_mtx_);
  if (gcs_enabled_) {
    if (!writer_.isOpened() && now_steady_sec() - last_open_try_ > 5.0) {
      open_writer();
    }
    if (writer_.isOpened()) {
      writer_.write(frame);
    }
  }
  if (pub_out_) {
    std_msgs::msg::Header h;
    h.stamp = now();
    h.frame_id = "streamer";
    pub_out_->publish(*cv_bridge::CvImage(h, "bgr8", frame).toImageMsg());
  }
}

std::string Streamer::build_pipeline() const {
  if (gcs_encoder_ == "custom") {
    return gcs_pipeline_;
  }
  const int fps = static_cast<int>(std::lround(fps_));
  std::string enc;
  if (gcs_encoder_ == "jetson") {
    enc =
        "appsrc is-live=true do-timestamp=true format=time ! queue ! "
        "videoconvert ! video/x-raw,format=I420 ! nvvidconv ! "
        "video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h264enc bitrate=" +
        std::to_string(gcs_bitrate_ * 1000) +

        " ! h264parse";
  } else if (gcs_encoder_ == "nvenc") {
    enc =
        "appsrc is-live=true do-timestamp=true format=time ! queue ! "
        "videoconvert ! nvh264enc bitrate=" +
        std::to_string(gcs_bitrate_) + " zerolatency=true ! h264parse";
  } else {
    enc =
        "appsrc is-live=true do-timestamp=true format=time ! queue ! "
        "videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency "
        "speed-preset=ultrafast bframes=0 bitrate=" +
        std::to_string(gcs_bitrate_) + " key-int-max=" + std::to_string(fps) +
        " ! video/x-h264,profile=baseline ! h264parse";
  }
  return enc +
         " config-interval=1 ! rtph264pay pt=96 config-interval=1 ! udpsink "
         "host=" +
         gcs_ip_ + " port=" + std::to_string(gcs_port_) +
         " sync=false async=false";
}

void Streamer::open_writer() {
  last_open_try_ = now_steady_sec();
  const std::string pipe = build_pipeline();
  if (pipe.empty()) {
    RCLCPP_ERROR(get_logger(), "gcs.encoder=custom but gcs.pipeline is empty");
    return;
  }
  RCLCPP_INFO(get_logger(), "GStreamer: %s", pipe.c_str());
  if (!writer_.open(pipe, cv::CAP_GSTREAMER, 0, fps_, out_size_, true)) {
    RCLCPP_ERROR(get_logger(),
                 "could not open GStreamer pipeline (OpenCV built without "
                 "GStreamer, or missing plugins?). "
                 "Retrying every 5 s.");
    return;
  }
  RCLCPP_INFO(
      get_logger(),
      "streaming to %s:%d. Receive with:\n  gst-launch-1.0 udpsrc port=%d "
      "caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
      "! rtph264depay ! "
      "avdec_h264 ! videoconvert ! autovideosink sync=false",
      gcs_ip_.c_str(), gcs_port_, gcs_port_);
}

}  // namespace rov_vision
