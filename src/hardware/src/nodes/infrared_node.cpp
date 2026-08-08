// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// InfraredNode — reads MC602 IR sensors via MC602Adapter::read_infrared
// and publishes sensor_msgs/Range to /rak/sensors/ir/<id>.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §传感器抽象
// Hardware: docs/hardware-port-mapping.md §P7/P8 红外测距
//
// Each IR sensor is an MC602 Infrared peripheral on a specific port:
//   P7 → right IR  (port 7)
//   P8 → left IR   (port 8)

#include "hardware/mc602_adapter.hpp"
#include "hardware/transport_factory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace vw = hardware;

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
    this->declare_parameter<std::string>("mc602_serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("mc602_baud", 1000000);
    this->declare_parameter<std::string>("mc602_transport", "direct");

    ir_id_ = this->get_parameter("ir_id").as_string();
    const int port = this->get_parameter("mc602_port").as_int();
    min_range_ = this->get_parameter("min_range_m").as_double();
    max_range_ = this->get_parameter("max_range_m").as_double();
    const double rate = this->get_parameter("rate_hz").as_double();
    const std::string serial_port = this->get_parameter("mc602_serial_port").as_string();
    const int baud = this->get_parameter("mc602_baud").as_int();
    const std::string transport_mode = this->get_parameter("mc602_transport").as_string();

    if (port < 1 || port > 16) {
      RCLCPP_FATAL(this->get_logger(),
        "mc602_port %d out of range [1, 16]", port);
      throw std::invalid_argument("mc602_port out of range");
    }
    port_id_ = static_cast<uint8_t>(port);

    topic_ = "/rak/sensors/ir/" + ir_id_;
    pub_ = this->create_publisher<sensor_msgs::msg::Range>(topic_, 10);

    // --- MC602 hardware interface ---
    adapter_ = std::make_unique<vw::MC602Adapter>(
      vw::make_mc602_transport(this, transport_mode, serial_port,
                               static_cast<uint32_t>(baud)));
    adapter_->open();
    RCLCPP_INFO(this->get_logger(),
      "MC602Adapter opened: %s @ %d baud via %s", serial_port.c_str(), baud,
      transport_mode.c_str());

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this]() { this->publish_range(); });

    RCLCPP_INFO(
      this->get_logger(), "InfraredNode[%s] publishing to %s (%.1f Hz, P%d, [%.2f, %.2f] m)",
      ir_id_.c_str(), topic_.c_str(), rate, port, min_range_, max_range_);
  }

  ~InfraredNode() override
  {
    try {
      if (adapter_) {
        adapter_->close();
      }
    } catch (...) {}
  }

private:
  void publish_range()
  {
    float dist_m = 0.0f;

    try {
      dist_m = adapter_->read_ir(port_id_);  // meters
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "read_ir(P%d) failed: %s", port_id_, e.what());
      return;
    }

    // Clamp to configured range.
    dist_m = std::clamp(dist_m, static_cast<float>(min_range_), static_cast<float>(max_range_));

    auto msg = std::make_unique<sensor_msgs::msg::Range>();
    msg->header.stamp = this->now();
    msg->header.frame_id = ir_id_ + "_ir_frame";
    msg->radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg->field_of_view = 0.1f;
    msg->min_range = static_cast<float>(min_range_);
    msg->max_range = static_cast<float>(max_range_);
    msg->range = dist_m;

    pub_->publish(std::move(msg));
  }

  std::string ir_id_;
  std::string topic_;
  uint8_t port_id_{0};
  double min_range_{0.02};
  double max_range_{0.3};

  std::unique_ptr<vw::MC602Adapter> adapter_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded: bridge mode blocks the callback on a service round-trip;
  // extra threads keep subscriptions/timers serviced while it waits.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(std::make_shared<InfraredNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
