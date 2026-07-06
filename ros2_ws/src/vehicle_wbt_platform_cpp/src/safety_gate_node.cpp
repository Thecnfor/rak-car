// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SafetyGateNode — implements the 4-layer safety gate per
// docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §安全设计
//
// Layer 1: mode check (only MANUAL/DEV_TEST accepts /cmd/*)
// Layer 2: rate limit (0.3 m/s velocity cap)
// Layer 3: deadman switch (500 ms heartbeat timeout)
// Layer 4: physical estop priority (BUTTON3 always wins)
//
// Output: /vehicle_wbt/v1/state/safety (safety_msgs/msg/SupervisoryControl) —
// placeholder for now. Subscribes:
//   /vehicle_wbt/v1/safety/heartbeat (std_msgs/Empty)
//   /vehicle_wbt/v1/safety/estop (std_msgs/Bool)
//   /vehicle_wbt/v1/safety/mode_cmd (std_msgs/String)
// Publishes (after gating):
//   /vehicle_wbt/v1/cmd/vel_safe (geometry_msgs/Twist)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

enum class Mode { AUTO, MANUAL, DEV_TEST, E_STOP };

class SafetyGateNode : public rclcpp::Node
{
public:
  explicit SafetyGateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("safety_gate_node", options)
  {
    this->declare_parameter<double>("max_linear_velocity", 0.3);
    this->declare_parameter<double>("max_angular_velocity", 0.5);
    this->declare_parameter<int>("deadman_ms", 500);

    max_v_ = this->get_parameter("max_linear_velocity").as_double();
    max_w_ = this->get_parameter("max_angular_velocity").as_double();
    const int deadman_ms = this->get_parameter("deadman_ms").as_int();

    heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/vehicle_wbt/v1/safety/heartbeat", 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_heartbeat_ = this->now();
      });

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vehicle_wbt/v1/safety/estop", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        estop_pressed_ = msg->data;
        if (estop_pressed_) {
          mode_ = Mode::E_STOP;
        }
      });

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vehicle_wbt/v1/safety/mode_cmd", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (msg->data == "AUTO") mode_ = Mode::AUTO;
        else if (msg->data == "MANUAL") mode_ = Mode::MANUAL;
        else if (msg->data == "DEV_TEST") mode_ = Mode::DEV_TEST;
        else if (msg->data == "E_STOP") mode_ = Mode::E_STOP;
      });

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/vehicle_wbt/v1/cmd/vel_raw", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::Twist gated;
        if (gate(*msg, gated)) {
          safe_pub_->publish(gated);
        }
      });

    safe_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/vehicle_wbt/v1/cmd/vel_safe", 10);

    deadman_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(deadman_ms / 4),
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto now = this->now();
        if (mode_ != Mode::AUTO && (now - last_heartbeat_).seconds() * 1000.0 > deadman_ms_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Deadman timeout in mode MANUAL/DEV_TEST — publishing stop");
          geometry_msgs::msg::Twist stop;
          safe_pub_->publish(stop);
        }
      });

    RCLCPP_INFO(
      this->get_logger(),
      "SafetyGateNode ready: max_v=%.2f m/s max_w=%.2f rad/s deadman=%d ms",
      max_v_, max_w_, deadman_ms);
  }

private:
  bool gate(const geometry_msgs::msg::Twist & in, geometry_msgs::msg::Twist & out) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Layer 4: physical estop always wins.
    if (estop_pressed_) {
      out = geometry_msgs::msg::Twist();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "E-STOP active");
      return true;  // publish stop, do NOT pass-through
    }

    // Layer 1: mode gate. AUTO blocks all /cmd/*.
    if (mode_ == Mode::AUTO) {
      return false;  // drop
    }

    // Layer 2: rate limit (clamp, not drop).
    out = in;
    const double vx = std::clamp(in.linear.x, -max_v_, max_v_);
    const double vy = std::clamp(in.linear.y, -max_v_, max_v_);
    const double wz = std::clamp(in.angular.z, -max_w_, max_w_);
    out.linear.x = vx;
    out.linear.y = vy;
    out.angular.z = wz;

    return true;
  }

  mutable std::mutex state_mutex_;
  Mode mode_{Mode::AUTO};
  bool estop_pressed_{false};
  rclcpp::Time last_heartbeat_;
  int deadman_ms_{500};
  double max_v_{0.3};
  double max_w_{0.5};

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_;
  rclcpp::TimerBase::SharedPtr deadman_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
