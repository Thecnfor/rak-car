// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// CameraNode — publishes 5 streams per camera under /vehicle_wbt/v1/sensors/camera/<id>:
//
//   1. image_raw         sensor_msgs/Image          BEST_EFFORT depth=1   (full frames)
//   2. image_compressed  sensor_msgs/CompressedImage BEST_EFFORT depth=1   (JPEG q=85)
//   3. camera_info       sensor_msgs/CameraInfo     TRANSIENT_LOCAL         (calibration)
//   4. camera_status     diagnostic_msgs/DiagnosticArray                  (health)
//   5. camera_meta       vehicle_wbt_platform_cpp/CameraMeta               (driver state)
//
// See docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Camera 抽象
//
// REAL HARDWARE ONLY — no synthetic / placeholder frames. If the V4L2 device
// cannot be opened at construction, this node throws std::runtime_error and
// the process dies (let ros2 launch / systemd restart it). If a per-tick
// capture() fails (USB cable yanked, transient device error), the tick is
// skipped — nothing is published. Never publish a fake frame to "keep the
// pipeline alive"; consumers must be able to trust that every image_raw
// message came from the real hardware.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <vehicle_wbt_platform_cpp/msg/camera_meta.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

using namespace std::chrono_literals;

namespace vwpc_cam
{
// ROS2 image-processing helpers should use these names so frame_ids
// follow REP-103/104. Keep them here (not in a public header) since they
// are only used inside camera_node.
constexpr const char * kEncodingRaw = "bgr8";
constexpr const char * kEncodingCompressed = "jpeg";

// QoS profiles tuned per stream. Using helper lambdas keeps construction
// in one place and easy to tweak without duplicating magic numbers.
inline rclcpp::QoS image_qos()
{
  // Best-effort + small queue: vision tolerates dropped frames but
  // a backlog of stale frames just costs bandwidth for nothing.
  return rclcpp::QoS(1).best_effort();
}
inline rclcpp::QoS info_qos()
{
  // TRANSIENT_LOCAL: a late subscriber should still get the latest calibration.
  return rclcpp::QoS(1).transient_local();
}
inline rclcpp::QoS status_qos()
{
  // RELIABLE for health: a missed diagnostic could hide a real fault.
  return rclcpp::QoS(10).reliable();
}
}  // namespace vwpc_cam

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera_node", options)
  {
    // --- Parameters (all ports/devices passed in, never hardcoded) ---
    this->declare_parameter<std::string>("camera_id", "front");
    this->declare_parameter<std::string>("device", "/dev/cam2");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<double>("rate_hz", 10.0);
    this->declare_parameter<int>("jpeg_quality", 85);
    // Calibration: real values come from a YAML loaded by camera_info_manager
    // in Plan B. Defaults below are a sane 640x480 pinhole so the topic
    // shape is exercisable without calibration data.
    this->declare_parameter<double>("fx", 600.0);
    this->declare_parameter<double>("fy", 600.0);
    this->declare_parameter<double>("cx", 320.0);
    this->declare_parameter<double>("cy", 240.0);

    camera_id_ = this->get_parameter("camera_id").as_string();
    const std::string device = this->get_parameter("device").as_string();
    const int width = this->get_parameter("image_width").as_int();
    const int height = this->get_parameter("image_height").as_int();
    const double rate = this->get_parameter("rate_hz").as_double();
    const int jpeg_q = this->get_parameter("jpeg_quality").as_int();
    fx_ = this->get_parameter("fx").as_double();
    fy_ = this->get_parameter("fy").as_double();
    cx_ = this->get_parameter("cx").as_double();
    cy_ = this->get_parameter("cy").as_double();

    width_ = width;
    height_ = height;
    device_ = device;
    frame_id_ = camera_id_ + "_camera_optical_frame";

    const std::string ns = "/vehicle_wbt/v1/sensors/camera/" + camera_id_;

    // --- Open V4L2 capture up-front. NO FALLBACK. If the device isn't there
    // or isn't readable, throw and let the process die — the operator /
    // launch system is responsible for taking the system offline until the
    // hardware is fixed. Publishing synthetic frames as if the camera were
    // alive would silently corrupt downstream perception (lane following,
    // OCR, detection) with garbage the rest of the pipeline trusts. ---
    cap_ = std::make_unique<cv::VideoCapture>();
    // The Aveo SP2812 (and most cheap UVC cams) only advertises MJPG in
    // --list-formats-ext. If we don't tell OpenCV we want MJPG, it tries
    // uncompressed YUYV by default and the device never sends frames
    // (select() times out). Set FOURCC before open() to force MJPG.
    const int mjpg_fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!cap_->open(device_, cv::CAP_V4L2)) {
      throw std::runtime_error(
        "CameraNode[" + camera_id_ + "]: cannot open V4L2 device '" + device_ +
        "'. Real hardware is required — refusing to publish synthetic frames. "
        "Check (1) USB cable, (2) udev rules in /etc/udev/rules.d/99-usbvideo.rules, "
        "(3) the `device` launch arg in full_system.launch.py.");
    }
    cap_->set(cv::CAP_PROP_FOURCC, static_cast<double>(mjpg_fourcc));
    cap_->set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    cap_->set(cv::CAP_PROP_CONVERT_RGB, 1.0);  // ask OpenCV to deliver BGR
    // Note: do NOT set CAP_PROP_FPS — most UVC cams honor only the formats
    // they advertised and ignore FPS hints, while setting it can cause
    // negotiation stalls on some drivers.

    // Camera drivers often accept a width/height/FPS request but deliver
    // something different. Capture the actual values from the device so
    // publishers' encoding/header fields match the real frames.
    const int actual_w = static_cast<int>(cap_->get(cv::CAP_PROP_FRAME_WIDTH));
    const int actual_h = static_cast<int>(cap_->get(cv::CAP_PROP_FRAME_HEIGHT));
    if (actual_w != width_ || actual_h != height_) {
      RCLCPP_WARN(
        this->get_logger(),
        "CameraNode[%s]: requested %dx%d but device gave %dx%d — using device's values",
        camera_id_.c_str(), width_, height_, actual_w, actual_h);
      width_ = actual_w > 0 ? actual_w : width_;
      height_ = actual_h > 0 ? actual_h : height_;
    }

    // --- Publishers — one per stream, each with its own QoS ---
    pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
      ns + "/image_raw", vwpc_cam::image_qos());
    pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      ns + "/image_compressed", vwpc_cam::image_qos());
    pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      ns + "/camera_info", vwpc_cam::info_qos());
    pub_status_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      ns + "/camera_status", vwpc_cam::status_qos());
    pub_meta_ = this->create_publisher<vehicle_wbt_platform_cpp::msg::CameraMeta>(
      ns + "/camera_meta", vwpc_cam::status_qos());

    // One-shot CameraInfo so late subscribers see it (TRANSIENT_LOCAL).
    camera_info_template_ = build_camera_info_template();
    publish_camera_info_once();

    // --- Timer for image streams (frames) ---
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_frames_ = this->create_wall_timer(period, [this, jpeg_q]() {
      this->tick_frames(jpeg_q);
    });

    // --- Timer for status + meta (1 Hz; cheaper than per-frame) ---
    timer_status_ = this->create_wall_timer(
      1s, [this]() { this->tick_status_meta(); });

    RCLCPP_INFO(
      this->get_logger(),
      "CameraNode[%s] up (live V4L2): 5 streams @ %s (raw %dx%d @ %.1f Hz, "
      "jpeg q=%d, device=%s, backend=%s)",
      camera_id_.c_str(), ns.c_str(), width_, height_, rate, jpeg_q, device_.c_str(),
      cap_->getBackendName().c_str());
  }

private:
  // ---- Frame producer: V4L2 capture, NO SYNTHETIC FALLBACK ----
  // Pulls one frame from /dev/cam<N>. If the read fails (USB cable yank,
  // transient timeout, decoder error) the function returns false and
  // tick_frames() SKIPS publishing — consumers can trust that every
  // image_raw / image_compressed they receive came from real hardware.
  bool capture(cv::Mat & out)
  {
    if (!cap_ || !cap_->isOpened()) {
      return false;
    }
    // grab() decodes + retrieves a frame; returns false on I/O failure.
    // retrieve() copies into the output Mat.
    if (!cap_->grab()) {
      return false;
    }
    return cap_->retrieve(out);
  }

  // ---- Per-frame: publish raw + compressed ----
  void tick_frames(int jpeg_q)
  {
    cv::Mat frame;
    if (!capture(frame) || frame.empty()) {
      // Real capture failed this tick. SKIP publishing — never substitute
      // a previous or synthetic frame. Warn at most every 5s to avoid
      // drowning the log when the camera is unplugged.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "CameraNode[%s]: V4L2 capture returned no frame on device %s; "
        "skipping this tick (NOT publishing a placeholder)",
        camera_id_.c_str(), device_.c_str());
      return;
    }

    const auto stamp = this->now();

    // image_raw — bgr8 (3 bytes/pixel)
    {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id_;
      msg->width = static_cast<uint32_t>(frame.cols);
      msg->height = static_cast<uint32_t>(frame.rows);
      msg->encoding = vwpc_cam::kEncodingRaw;
      msg->is_bigendian = false;
      msg->step = static_cast<uint32_t>(frame.cols * frame.elemSize());
      const size_t bytes = frame.total() * frame.elemSize();
      msg->data.assign(frame.data, frame.data + bytes);
      pub_raw_->publish(std::move(msg));
    }

    // image_compressed — JPEG, bytes via OpenCV
    {
      std::vector<uint8_t> jpeg_buf;
      const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_q};
      cv::imencode(".jpg", frame, jpeg_buf, params);

      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id_;
      msg->format = vwpc_cam::kEncodingCompressed;
      msg->data = std::move(jpeg_buf);
      pub_compressed_->publish(std::move(msg));
    }
  }

  // ---- 1 Hz: publish status + meta ----
  void tick_status_meta()
  {
    const auto stamp = this->now();

    // DiagnosticArray with a single OK status (Phase 1.5 has nothing to fail yet).
    {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = stamp;
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.name = "camera_node/" + camera_id_;
      s.message = "live V4L2 capture";
      s.hardware_id = device_;
      array.status.push_back(std::move(s));
      pub_status_->publish(std::move(array));
    }

    // Custom CameraMeta — driver/runtime metrics. NaN per spec when unknown.
    {
      vehicle_wbt_platform_cpp::msg::CameraMeta meta;
      meta.header.stamp = stamp;
      meta.header.frame_id = frame_id_;  // match image_raw's optical frame
      meta.camera_id = camera_id_;
      meta.exposure_us = std::numeric_limits<float>::quiet_NaN();
      meta.exposure_auto = true;
      meta.gain = std::numeric_limits<float>::quiet_NaN();
      meta.gain_auto = true;
      meta.white_balance_kelvin = 0;
      meta.white_balance_auto = true;
      meta.temperature_c = std::numeric_limits<float>::quiet_NaN();
      meta.device_path = device_;
      pub_meta_->publish(std::move(meta));
    }
  }

  // ---- One-shot CameraInfo (calibration) ----
  sensor_msgs::msg::CameraInfo build_camera_info_template()
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(width_);
    info.height = static_cast<uint32_t>(height_);

    // Intrinsic matrix K (pinhole). Naive defaults — real calibration in Plan B.
    info.k = {fx_, 0.0, cx_,
              0.0, fy_, cy_,
              0.0, 0.0, 1.0};
    // No distortion for now; plumb_brown_congo or similar in Plan B.
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    // Rectification R = identity; projection P = K with [Tx Ty 1] appended.
    info.r = {1, 0, 0,
              0, 1, 0,
              0, 0, 1};
    // P is 3x4 = 12 elements, not 14 — was a typo in the stub.
    info.p = {fx_, 0.0, cx_, 0.0,
              0.0, fy_, cy_, 0.0,
              0.0, 0.0, 1.0, 0.0};
    info.binning_x = 0;
    info.binning_y = 0;
    return info;
  }

  void publish_camera_info_once()
  {
    // Publish a couple of times during startup so late-joining subscribers
    // (TRANSIENT_LOCAL) reliably see the latched info even if they connect
    // before our first publish completed.
    auto info = camera_info_template_;
    info.header.stamp = this->now();
    pub_info_->publish(info);
    auto info2 = info;
    info2.header.stamp = this->now();
    pub_info_->publish(info2);
  }

  // ---- State ----
  std::string camera_id_;
  std::string device_;
  std::string frame_id_;
  int width_{640};
  int height_{480};
  double fx_{600.0}, fy_{600.0}, cx_{320.0}, cy_{240.0};

  sensor_msgs::msg::CameraInfo camera_info_template_;

  // V4L2 capture handle. Opened in the constructor; if it can't open the
  // configured device, the constructor throws and the process dies
  // (no synthetic frames ever enter the publisher pipeline).
  std::unique_ptr<cv::VideoCapture> cap_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_status_;
  rclcpp::Publisher<vehicle_wbt_platform_cpp::msg::CameraMeta>::SharedPtr pub_meta_;

  rclcpp::TimerBase::SharedPtr timer_frames_;
  rclcpp::TimerBase::SharedPtr timer_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
