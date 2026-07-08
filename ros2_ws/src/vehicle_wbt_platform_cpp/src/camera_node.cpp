// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// CameraNode — production camera publisher for any UVC device.
//
// Publishes 5 streams under /vehicle_wbt/v1/sensors/camera/<id>/:
//
//   1. image_raw         sensor_msgs/Image          BEST_EFFORT depth=1
//   2. image_compressed  sensor_msgs/CompressedImage BEST_EFFORT depth=1 (jpeg)
//   3. camera_info       sensor_msgs/CameraInfo     TRANSIENT_LOCAL
//                            (only if calibration YAML has real data;
//                             otherwise NOT published — see "no-mocks" rule)
//   4. camera_status     diagnostic_msgs/DiagnosticArray      1 Hz
//   5. camera_meta       vehicle_wbt_platform_cpp/CameraMeta   1 Hz
//
// Plus /tf_static: <id>_camera_optical_frame → base_link (from launch params).
//
// REAL HARDWARE ONLY. No synthetic frames, no fake intrinsics, no fake
// white-balance temperature. The no-mocks rule applies:
//   https://github.com/.../CLAUDE.md (rule #6) +
//   docs/coding-rules-no-mocks.md
//
// Status semantics (camera_status):
//   OK    : capture OK, no recent failures
//   WARN  : < kReopenAfterFailStreak consecutive failures (transient)
//   ERROR : >= kReopenAfterFailStreak failures (we are trying to reopen)
//
// Reconnect: on consecutive failures = kReopenAfterFailStreak, the node
// releases the V4L2 handle and opens it again. If reopen fails the node
// keeps trying — it does NOT crash, so OS-level tooling (udev, cable) can
// heal it without process supervision.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <vehicle_wbt_platform_cpp/msg/camera_meta.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

using namespace std::chrono_literals;

namespace vwpc_cam
{
// QoS profiles, centralised so a tweak here changes every publisher.
constexpr const char * kEncodingRaw = "bgr8";
constexpr const char * kEncodingCompressed = "jpeg";

inline rclcpp::QoS image_qos() { return rclcpp::QoS(1).best_effort(); }
inline rclcpp::QoS info_qos() { return rclcpp::QoS(1).transient_local(); }
inline rclcpp::QoS status_qos() { return rclcpp::QoS(10).reliable(); }

// Health-state tuning. Consecutive V4L2 failures trigger reopen attempt.
// 5 missed frames at 10 Hz = 0.5s; long enough to absorb a transient
// USB hiccup, short enough that operators don't wait long when the
// cable is broken.
constexpr int kReopenAfterFailStreak = 5;
}  // namespace vwpc_cam

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera_node", options)
  {
    // ---- Parameters (all knobs overridable; nothing hardcoded) ----
    this->declare_parameter<std::string>("camera_id", "front");
    this->declare_parameter<std::string>("device", "/dev/cam2");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<double>("rate_hz", 30.0);
    this->declare_parameter<int>("jpeg_quality", 85);
    this->declare_parameter<std::string>("calibration_url", "");
    // camera_optical_frame pose in robot base_link (radians, meters).
    this->declare_parameter<std::string>("tf_parent_frame", "base_link");
    this->declare_parameter<double>("tf_x", 0.0);
    this->declare_parameter<double>("tf_y", 0.0);
    this->declare_parameter<double>("tf_z", 0.0);
    this->declare_parameter<double>("tf_roll", 0.0);
    this->declare_parameter<double>("tf_pitch", 0.0);
    this->declare_parameter<double>("tf_yaw", 0.0);

    camera_id_ = this->get_parameter("camera_id").as_string();
    device_ = this->get_parameter("device").as_string();
    width_requested_ = this->get_parameter("image_width").as_int();
    height_requested_ = this->get_parameter("image_height").as_int();
    rate_hz_ = this->get_parameter("rate_hz").as_double();
    jpeg_q_ = this->get_parameter("jpeg_quality").as_int();
    calibration_url_ = this->get_parameter("calibration_url").as_string();

    tf_parent_frame_ = this->get_parameter("tf_parent_frame").as_string();
    tf_x_ = this->get_parameter("tf_x").as_double();
    tf_y_ = this->get_parameter("tf_y").as_double();
    tf_z_ = this->get_parameter("tf_z").as_double();
    tf_roll_ = this->get_parameter("tf_roll").as_double();
    tf_pitch_ = this->get_parameter("tf_pitch").as_double();
    tf_yaw_ = this->get_parameter("tf_yaw").as_double();

    frame_id_ = camera_id_ + "_camera_optical_frame";
    ns_ = "/vehicle_wbt/v1/sensors/camera/" + camera_id_;

    // ---- V4L2: throw on open failure (no silent fallback) ----
    open_capture();

    // ---- CameraInfoManager: load YAML if URL given, no fake intrinsics ----
    cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_id_);
    if (!calibration_url_.empty()) {
      // loadCameraInfo() returns true on success. The K matrix in the
      // returned CameraInfo is all-zero if the YAML was the empty
      // sentinel — has_real_calibration() below detects that.
      const bool ok = cam_info_manager_->loadCameraInfo(calibration_url_);
      RCLCPP_INFO(
        this->get_logger(),
        "CameraNode[%s]: calibration load '%s' -> %s",
        camera_id_.c_str(), calibration_url_.c_str(), ok ? "OK" : "FAILED");
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "CameraNode[%s]: no calibration_url → camera_info not published",
        camera_id_.c_str());
    }

    // ---- Publishers (camera_info only created if calibration has real data) ----
    pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
      ns_ + "/image_raw", vwpc_cam::image_qos());
    pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      ns_ + "/image_compressed", vwpc_cam::image_qos());
    pub_status_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      ns_ + "/camera_status", vwpc_cam::status_qos());
    pub_meta_ = this->create_publisher<vehicle_wbt_platform_cpp::msg::CameraMeta>(
      ns_ + "/camera_meta", vwpc_cam::status_qos());
    if (has_real_calibration()) {
      pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        ns_ + "/camera_info", vwpc_cam::info_qos());
      // Publish once for TRANSIENT_LOCAL late subscribers.
      publish_camera_info_once(this->now());
    }

    // ---- /tf_static: camera optical frame in robot base_link ----
    publish_static_tf();

    // ---- Timers ----
    const auto frame_period = std::chrono::microseconds(
      static_cast<int64_t>(1'000'000.0 / rate_hz_));
    timer_frames_ = this->create_wall_timer(
      frame_period, [this]() { this->tick_frames(jpeg_q_); });
    timer_status_ = this->create_wall_timer(1s, [this]() {
      this->tick_status_meta();
    });

    RCLCPP_INFO(
      this->get_logger(),
      "CameraNode[%s] up: device=%s, %dx%d@%.1fHz, jpeg q=%d, "
      "calibration='%s', frame_id='%s' under %s",
      camera_id_.c_str(), device_.c_str(), actual_width_, actual_height_,
      rate_hz_, jpeg_q_, calibration_url_.c_str(), frame_id_.c_str(),
      ns_.c_str());
  }

  ~CameraNode() override
  {
    if (cap_ && cap_->isOpened()) {
      cap_->release();
    }
  }

private:
  // ============================================================
  // V4L2 capture
  // ============================================================
  void open_capture()
  {
    cap_ = std::make_unique<cv::VideoCapture>();
    const int mjpg = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!cap_->open(device_, cv::CAP_V4L2)) {
      throw std::runtime_error(
        "CameraNode[" + camera_id_ + "]: cannot open V4L2 '" + device_ +
        "'. Real hardware is required — refusing to publish synthetic frames. "
        "Check USB cable + udev rules + the device launch arg.");
    }
    cap_->set(cv::CAP_PROP_FOURCC, static_cast<double>(mjpg));
    cap_->set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_requested_));
    cap_->set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_requested_));
    // FPS: ask the device explicitly. Default without this call is
    // driver-dependent — Aveo SP2812 defaults to ~10 fps when not
    // requested, even though --list-formats-ext advertises 30. We
    // request the rate_hz launch arg (clamped to what the device
    // supports; cap_->set tolerates unsupported values by ignoring).
    cap_->set(cv::CAP_PROP_FPS, rate_hz_);
    cap_->set(cv::CAP_PROP_CONVERT_RGB, 1.0);

    actual_width_ = static_cast<int>(cap_->get(cv::CAP_PROP_FRAME_WIDTH));
    actual_height_ = static_cast<int>(cap_->get(cv::CAP_PROP_FRAME_HEIGHT));
    if (actual_width_ <= 0) {
      actual_width_ = width_requested_;
    }
    if (actual_height_ <= 0) {
      actual_height_ = height_requested_;
    }
  }

  // Pull one frame. Returns false on any failure (USB yank, decoder
  // error, etc.); caller is responsible for updating health counters.
  bool capture(cv::Mat & out)
  {
    if (!cap_ || !cap_->isOpened()) {
      return false;
    }
    if (!cap_->grab()) {
      return false;
    }
    return cap_->retrieve(out);
  }

  // After kReopenAfterFailStreak consecutive failures, drop the V4L2
  // handle and re-open. The OS / udev / cable can heal, then we resume.
  // Loop continues forever — connection-loss is normal for a robot.
  void maybe_reopen()
  {
    if (consecutive_failures_ < vwpc_cam::kReopenAfterFailStreak) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "CameraNode[%s]: %d consecutive V4L2 failures — reopening '%s'",
      camera_id_.c_str(), consecutive_failures_, device_.c_str());
    cap_->release();
    try {
      open_capture();
      consecutive_failures_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "CameraNode[%s]: reopened '%s' (backend %s)",
        camera_id_.c_str(), device_.c_str(),
        cap_ ? cap_->getBackendName().c_str() : "?");
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "CameraNode[%s]: reopen failed: %s — retrying on next tick",
        camera_id_.c_str(), e.what());
    }
  }

  // ============================================================
  // Per-frame: image_raw + image_compressed (+ camera_info if valid)
  // ============================================================
  void tick_frames(int jpeg_q)
  {
    cv::Mat frame;
    if (!capture(frame) || frame.empty()) {
      ++consecutive_failures_;
      ++total_drop_count_;
      maybe_reopen();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "CameraNode[%s]: V4L2 capture failed (streak=%d, drops=%llu); "
        "skipping tick — NEVER publishing a placeholder",
        camera_id_.c_str(), consecutive_failures_,
        static_cast<unsigned long long>(total_drop_count_));
      return;
    }

    consecutive_failures_ = 0;
    ++frames_published_;
    const auto stamp = this->now();

    // image_raw (bgr8)
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

    // image_compressed (JPEG)
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

    // camera_info — only when YAML had real data; mirror with frame stamp
    if (pub_info_) {
      publish_camera_info_with_stamp(stamp);
    }
  }

  // ============================================================
  // 1 Hz: camera_status + camera_meta (real driver queries)
  // ============================================================
  void tick_status_meta()
  {
    const auto stamp = this->now();
    const auto health = compute_health_level();

    // ---- camera_status ----
    {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = stamp;
      diagnostic_msgs::msg::DiagnosticStatus ds;
      ds.level = static_cast<uint8_t>(health);
      ds.name = "camera_node/" + camera_id_;
      ds.hardware_id = device_;
      ds.message = health_message(health);
      add_kv(ds.values, "frame_id", frame_id_);
      add_kv(ds.values, "device_path", device_);
      add_kv(ds.values, "tf_parent_frame", tf_parent_frame_);
      add_kv(ds.values, "resolution", std::to_string(actual_width_) + "x" +
        std::to_string(actual_height_));
      add_kv(ds.values, "rate_hz_target", std::to_string(rate_hz_));
      add_kv(ds.values, "jpeg_quality", std::to_string(jpeg_q_));
      // Achieved FPS: delta since last status (1 Hz timer), bumped per
      // captured frame above. Should match rate_hz_target when the
      // device is keeping up; lower means the V4L2 pipeline is
      // struggling (USB saturation or decoder hiccup).
      const uint64_t delta =
        frames_published_ - frames_published_last_status_;
      frames_published_last_status_ = frames_published_;
      add_kv(ds.values, "frames_published", std::to_string(frames_published_));
      add_kv(
        ds.values, "achieved_rate_hz", std::to_string(
          static_cast<double>(delta)));
      add_kv(ds.values, "rate_hz_target", std::to_string(rate_hz_));
      add_kv(ds.values, "total_drops", std::to_string(total_drop_count_));
      add_kv(ds.values, "consecutive_failures",
        std::to_string(consecutive_failures_));
      add_kv(ds.values, "calibration_loaded",
        has_real_calibration() ? "true" : "false");
      add_kv(ds.values, "v4l2_backend", backend_name());
      array.status.push_back(std::move(ds));
      pub_status_->publish(std::move(array));
    }

    // ---- camera_meta: real V4L2 control queries via cap_->get() ----
    {
      vehicle_wbt_platform_cpp::msg::CameraMeta meta;
      meta.header.stamp = stamp;
      meta.header.frame_id = frame_id_;
      meta.camera_id = camera_id_;
      meta.device_path = device_;

      // OpenCV's CAP_PROP_* → NaN-or-value pattern. UVC cams commonly
      // report 0 for controls they don't implement — per OpenCV docs:
      // "Value 0 is returned when querying a property that is not
      // supported by the backend." We treat <= 0 as "unknown" and
      // emit NaN per the no-mocks rule (no real value → no
      // plausible-looking fake).
      meta.exposure_us = safe_cap_get<float>(cv::CAP_PROP_EXPOSURE);
      meta.exposure_auto = !cap_exposes(cv::CAP_PROP_EXPOSURE);
      meta.gain = safe_cap_get<float>(cv::CAP_PROP_GAIN);
      meta.gain_auto = !cap_exposes(cv::CAP_PROP_GAIN);
      meta.white_balance_kelvin =
        static_cast<uint16_t>(safe_cap_get<double>(cv::CAP_PROP_WB_TEMPERATURE));
      meta.white_balance_auto =
        !cap_exposes(cv::CAP_PROP_WB_TEMPERATURE);
      // CAP_PROP_TEMPERATURE always returns 0 on UVC. Per the no-mocks
      // rule, we explicitly do not synthesize a value — NaN per spec.
      meta.temperature_c = std::numeric_limits<float>::quiet_NaN();

      pub_meta_->publish(std::move(meta));
    }
  }

  // ============================================================
  // camera_info_manager helpers
  // ============================================================
  bool has_real_calibration() const
  {
    if (!cam_info_manager_) {
      return false;
    }
    // loadCameraInfo(url) may have failed; isCalibrated() is the
    // package's own verdict and is the safest gate.
    return cam_info_manager_->isCalibrated();
  }

  void publish_camera_info_once(const rclcpp::Time & stamp)
  {
    publish_camera_info_with_stamp(stamp);
    publish_camera_info_with_stamp(stamp);  // belt-and-braces for TRANSIENT_LOCAL
  }

  void publish_camera_info_with_stamp(const rclcpp::Time & stamp)
  {
    if (!pub_info_ || !cam_info_manager_) {
      return;
    }
    auto info = cam_info_manager_->getCameraInfo();
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(actual_width_);
    info.height = static_cast<uint32_t>(actual_height_);
    pub_info_->publish(std::move(info));
  }

  // ============================================================
  // /tf_static: optical frame pose in robot base
  // ============================================================
  void publish_static_tf()
  {
    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = tf_parent_frame_;
    tf.child_frame_id = frame_id_;
    tf.transform.translation.x = tf_x_;
    tf.transform.translation.y = tf_y_;
    tf.transform.translation.z = tf_z_;

    // RPY → quaternion (ZYX intrinsic). When all three are 0 this
    // produces w=1, x=y=z=0 — the identity rotation, which is the
    // documented "unknown mount orientation" choice. Operators must
    // override via launch args for their physical rig.
    const double cr = std::cos(tf_roll_ * 0.5);
    const double sr = std::sin(tf_roll_ * 0.5);
    const double cp = std::cos(tf_pitch_ * 0.5);
    const double sp = std::sin(tf_pitch_ * 0.5);
    const double cy = std::cos(tf_yaw_ * 0.5);
    const double sy = std::sin(tf_yaw_ * 0.5);
    tf.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    tf.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    tf.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    tf.transform.rotation.z = cr * cp * sy - sr * sp * cy;

    static_tf_broadcaster_->sendTransform(tf);

    RCLCPP_INFO(
      this->get_logger(),
      "Published /tf_static: %s -> %s [t=(%.3f, %.3f, %.3f) "
      "rpy=(%.3f, %.3f, %.3f)]",
      tf_parent_frame_.c_str(), frame_id_.c_str(),
      tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_);
  }

  // ============================================================
  // Helper accessors
  // ============================================================
  diagnostic_msgs::msg::DiagnosticStatus::_level_type compute_health_level() const
  {
    if (consecutive_failures_ == 0) {
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    }
    if (consecutive_failures_ < vwpc_cam::kReopenAfterFailStreak) {
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  static const char * health_message(
    diagnostic_msgs::msg::DiagnosticStatus::_level_type level)
  {
    using D = diagnostic_msgs::msg::DiagnosticStatus;
    switch (level) {
      case D::OK:    return "live V4L2 capture";
      case D::WARN:  return "degraded — recent V4L2 read failures (transient)";
      case D::ERROR: return "capturing is failing — V4L2 reopen in progress";
      default:       return "unknown";
    }
  }

  static void add_kv(
    std::vector<diagnostic_msgs::msg::KeyValue> & kvs,
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    kvs.push_back(std::move(kv));
  }

  // cap_->get(CAP_PROP_X) returns 0 when the driver doesn't expose
  // that control; treat as "unknown" → NaN. Returns std::optional for
  // callers that want raw access.
  std::optional<double> cap_exposes(cv::VideoCaptureProperties p) const
  {
    if (!cap_ || !cap_->isOpened()) {
      return std::nullopt;
    }
    const double v = cap_->get(p);
    return (v > 0.0) ? std::optional<double>{v} : std::nullopt;
  }

  template <typename T>
  T safe_cap_get(cv::VideoCaptureProperties p) const
  {
    auto v = cap_exposes(p);
    return v.has_value() ? static_cast<T>(*v)
                         : std::numeric_limits<T>::quiet_NaN();
  }

  std::string backend_name() const
  {
    return (cap_ && cap_->isOpened()) ? cap_->getBackendName() : "released";
  }

  // ============================================================
  // State
  // ============================================================
  std::string camera_id_;
  std::string device_;
  std::string frame_id_;
  std::string ns_;
  std::string tf_parent_frame_;
  std::string calibration_url_;
  double tf_x_{0.0}, tf_y_{0.0}, tf_z_{0.0};
  double tf_roll_{0.0}, tf_pitch_{0.0}, tf_yaw_{0.0};

  int width_requested_{640};
  int height_requested_{480};
  int actual_width_{640};
  int actual_height_{480};
  double rate_hz_{30.0};
  int jpeg_q_{85};

  std::unique_ptr<cv::VideoCapture> cap_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  uint64_t frames_published_{0};
  uint64_t frames_published_last_status_{0};
  uint64_t total_drop_count_{0};
  int consecutive_failures_{0};

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
