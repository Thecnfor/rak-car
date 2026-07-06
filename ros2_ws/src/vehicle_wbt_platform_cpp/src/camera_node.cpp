// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// CameraNode — publishes sensor_msgs/Image to /vehicle_wbt/v1/sensors/camera/<id>.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Camera 抽象
//
// Phase 1.5: stub. Real impl opens /dev/cam* via V4L2 in Plan B. This stub
// publishes a synthetic black image at the configured rate so downstream
// perception nodes can be developed without real hardware.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("camera_node", options)
  {
    // Parameters
    this->declare_parameter<std::string>("camera_id", "front");
    this->declare_parameter<std::string>("device", "/dev/cam2");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<double>("rate_hz", 10.0);

    camera_id_ = this->get_parameter("camera_id").as_string();
    const std::string device = this->get_parameter("device").as_string();
    const int width = this->get_parameter("image_width").as_int();
    const int height = this->get_parameter("image_height").as_int();
    const double rate = this->get_parameter("rate_hz").as_double();

    topic_ = "/vehicle_wbt/v1/sensors/camera/" + camera_id_ + "/image_raw";
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_, 10);

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this, width, height, device]() {
      this->publish_synthetic_frame(width, height, device);
    });

    RCLCPP_INFO(
      this->get_logger(), "CameraNode[%s] publishing to %s (%.1f Hz, %dx%d, device=%s)",
      camera_id_.c_str(), topic_.c_str(), rate, width, height, device.c_str());
  }

private:
  void publish_synthetic_frame(int width, int height, const std::string & device)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = camera_id_ + "_camera_optical_frame";
    msg->width = static_cast<uint32_t>(width);
    msg->height = static_cast<uint32_t>(height);
    msg->encoding = "rgb8";
    msg->is_bigendian = false;
    msg->step = static_cast<uint32_t>(width * 3);
    msg->data.assign(static_cast<size_t>(width * height * 3), 0);

    // Phase 1.5: real impl reads from V4L2 device. Stub publishes zeroed frame.
    (void)device;
    pub_->publish(std::move(msg));
  }

  std::string camera_id_;
  std::string topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
