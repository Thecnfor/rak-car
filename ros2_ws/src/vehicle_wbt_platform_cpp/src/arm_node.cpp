// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// ArmNode — subscribes /vehicle_wbt/v1/cmd/arm/trajectory (JointTrajectory),
// publishes /vehicle_wbt/v1/state/actuators (sensor_msgs/JointState — standard
// ROS2 message; we previously used a custom ActuatorState.msg but skipped
// rosidl_generate_interfaces for Lyrical 2.8.7 compatibility).
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §机械臂抽象
//
// Phase 1.5: stub. Real impl wires StepperWrap + ServoBus + PoutD via
// MC602Adapter in Plan B.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <string>

using namespace std::chrono_literals;

class ArmNode : public rclcpp::Node
{
public:
  explicit ArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("arm_node", options)
  {
    this->declare_parameter<std::string>("arm_id", "main");
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    arm_id_ = this->get_parameter("arm_id").as_string();
    const double rate = this->get_parameter("publish_rate_hz").as_double();

    joint_names_ = {"horiz_m6", "vert_stepper3", "rotate_s3", "grip_s7"};

    const std::string cmd_topic = "/vehicle_wbt/v1/cmd/arm/" + arm_id_ + "/trajectory";
    const std::string state_topic = "/vehicle_wbt/v1/state/actuators/" + arm_id_;

    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      cmd_topic, 10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        last_traj_ = *msg;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Arm[%s] received trajectory with %zu points",
          arm_id_.c_str(), msg->points.size());
      });

    state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic, 10);

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this]() { this->publish_state(); });

    RCLCPP_INFO(
      this->get_logger(), "ArmNode[%s] publishing to %s (%.1f Hz)",
      arm_id_.c_str(), state_topic.c_str(), rate);
  }

private:
  void publish_state()
  {
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = joint_names_;
    msg->position.assign(joint_names_.size(), 0.0);
    msg->velocity.assign(joint_names_.size(), 0.0);
    msg->effort.assign(joint_names_.size(), 0.0);
    state_pub_->publish(std::move(msg));
  }

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  trajectory_msgs::msg::JointTrajectory last_traj_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmNode>());
  rclcpp::shutdown();
  return 0;
}
