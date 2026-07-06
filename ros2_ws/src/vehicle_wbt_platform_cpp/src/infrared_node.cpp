// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// InfraredNode — publishes sensor_msgs/Range to /vehicle_wbt/v1/sensors/ir/<id>.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Camera 抽象
//
// Phase 1.5: stub. Real impl reads MC602 Infrared() over MC602Adapter in
// Plan B. Stub publishes a constant range so navigation/PID can be tested.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

class InfraredNode : public rclcpp::Node
{
public:
  explicit InfraredNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("infrared_node", options)
  {
    this->declare_parameter<std::string>("ir_id", "left");
    this->declare_parameter<int>("mc602_port", 8);  // P8 per hardware-port-mapping.md
    this->declare_parameter<double>("min_range_m", 0.02);
    this->declare_parameter<double>("max_range_m", 0.3);
    this->declare_parameter<double>("rate_hz", 20.0);

    ir_id_ = this->get_parameter("ir_id").as_string();
    const int port = this->get_parameter("mc602_port").as_int();
    const double rate = this->get_parameter("rate_hz").as_double();
    const double min_r = this->get_parameter("min_range_m").as_double();
    const double max_r = this->get_parameter("max_range_m").as_double();

    topic_ = "/vehicle_wbt/v1/sensors/ir/" + ir_id_;
    pub_ = this->create_publisher<sensor_msgs::msg::Range>(topic_, 10);

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this, port, min_r, max_r]() {
      this->publish_synthetic_range(port, min_r, max_r);
    });

    RCLCPP_INFO(
      this->get_logger(), "InfraredNode[%s] publishing to %s (%.1f Hz, P%d, [%.2f, %.2f] m)",
      ir_id_.c_str(), topic_.c_str(), rate, port, min_r, max_r);
  }

private:
  void publish_synthetic_range(int port, double min_r, double max_r)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Range>();
    msg->header.stamp = this->now();
    msg->header.frame_id = ir_id_ + "_ir_frame";
    msg->radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg->field_of_view = 0.1f;
    msg->min_range = static_cast<float>(min_r);
    msg->max_range = static_cast<float>(max_r);
    msg->range = 0.0f;  // stub: no obstacle
    (void)port;
    pub_->publish(std::move(msg));
  }

  std::string ir_id_;
  std::string topic_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InfraredNode>());
  rclcpp::shutdown();
  return 0;
}
