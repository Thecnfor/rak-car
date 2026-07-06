// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SafetyGateNode — rclcpp::Node wrapper for the 4-layer safety gate.
//
// The actual gate logic lives in include/.../safety_gate_logic.hpp as a
// pure function (apply_safety_gate) that this node calls. Keeping the
// logic pure means it can be unit-tested with gtest without a ROS2
// runtime (see test/test_safety_gate_logic.cpp).
//
// 4 layers (handled by apply_safety_gate):
//   1. Physical estop: if pressed, output zero twist + always publish stop
//   2. Mode: if AUTO, drop the message (don't publish)
//   3. Rate limit: clamp linear/angular velocity to max_linear/max_angular
//   4. Heartbeat: tracked here via the deadman_timer_ (not in pure fn)
//
// Subscribes:
//   /vehicle_wbt/v1/safety/heartbeat (std_msgs/Empty)
//   /vehicle_wbt/v1/safety/estop (std_msgs/Bool)
//   /vehicle_wbt/v1/safety/mode_cmd (std_msgs/String)
//   /vehicle_wbt/v1/cmd/vel_raw (geometry_msgs/Twist)
// Publishes (after gating):
//   /vehicle_wbt/v1/cmd/vel_safe (geometry_msgs/Twist)

#include "vehicle_wbt_platform_cpp/safety_gate_logic.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace vwpc = vehicle_wbt_platform_cpp;

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
          mode_ = vwpc::GateMode::E_STOP;
        }
      });

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vehicle_wbt/v1/safety/mode_cmd", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (msg->data == "AUTO") mode_ = vwpc::GateMode::AUTO;
        else if (msg->data == "MANUAL") mode_ = vwpc::GateMode::MANUAL;
        else if (msg->data == "DEV_TEST") mode_ = vwpc::GateMode::DEV_TEST;
        else if (msg->data == "E_STOP") mode_ = vwpc::GateMode::E_STOP;
      });

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/vehicle_wbt/v1/cmd/vel_raw", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->apply_and_publish(*msg);
      });

    safe_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/vehicle_wbt/v1/cmd/vel_safe", 10);

    deadman_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(deadman_ms / 4),
      [this]() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto now = this->now();
        if (mode_ != vwpc::GateMode::AUTO &&
            (now - last_heartbeat_).seconds() * 1000.0 > deadman_ms_) {
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
  void apply_and_publish(const geometry_msgs::msg::Twist & raw_cmd)
  {
    vwpc::GateInput in;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      in.mode = mode_;
      in.estop_pressed = estop_pressed_;
      in.max_linear = max_v_;
      in.max_angular = max_w_;
    }

    const auto decision = vwpc::apply_safety_gate(
      in, raw_cmd.linear.x, raw_cmd.linear.y, raw_cmd.angular.z);

    if (static_cast<bool>(decision.reason & vwpc::GateDecision::Reason::ESTOP)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "E-STOP active");
    }
    if (static_cast<bool>(decision.reason & vwpc::GateDecision::Reason::MODE_DROPPED)) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Dropped cmd (AUTO mode)");
    }
    if (static_cast<bool>(decision.reason & vwpc::GateDecision::Reason::RATE_LIMITED)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Rate limited: vx=%.2f vy=%.2f wz=%.2f",
        decision.linear_x, decision.linear_y, decision.angular_z);
    }

    if (decision.publish) {
      geometry_msgs::msg::Twist out;
      out.linear.x = decision.linear_x;
      out.linear.y = decision.linear_y;
      out.angular.z = decision.angular_z;
      safe_pub_->publish(out);
    }
  }

  mutable std::mutex state_mutex_;
  vwpc::GateMode mode_{vwpc::GateMode::AUTO};
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
