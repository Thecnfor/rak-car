// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MecanumChassisNode — subscribes /vehicle_wbt/v1/cmd/vel_safe (Twist),
// publishes /vehicle_wbt/v1/state/odom (Odometry) + /tf (TF).
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象
//
// Phase 1.5: stub. Real impl wires MecanumChassis to MC602Adapter via
// ros2_control in Plan B. Stub does the kinematics math and publishes
// odom but does not actually move motors.

#include "vehicle_wbt_platform_cpp/mecanum_chassis.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MecanumChassisNode : public rclcpp::Node
{
public:
  explicit MecanumChassisNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mecanum_chassis_node", options),
    tf_broadcaster_(this),
    last_cmd_time_(this->now())
  {
    this->declare_parameter<double>("chassis_Lx", 0.15);
    this->declare_parameter<double>("chassis_Ly", 0.10);
    this->declare_parameter<double>("wheel_radius", 0.03);
    this->declare_parameter<double>("publish_rate_hz", 50.0);

    const double Lx = this->get_parameter("chassis_Lx").as_double();
    const double Ly = this->get_parameter("chassis_Ly").as_double();
    const double r = this->get_parameter("wheel_radius").as_double();
    const double rate = this->get_parameter("publish_rate_hz").as_double();

    chassis_ = std::make_unique<vehicle_wbt_platform_cpp::MecanumChassis>("mec1", Lx, Ly, r);
    chassis_->reset_odometry();

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/vehicle_wbt/v1/cmd/vel_safe", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        chassis_->set_velocity(msg->linear.x, msg->linear.y, msg->angular.z);
        last_cmd_time_ = this->now();
      });

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/vehicle_wbt/v1/state/odom", 10);

    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this]() { this->publish_odometry(); });

    RCLCPP_INFO(
      this->get_logger(),
      "MecanumChassisNode ready: Lx=%.3f Ly=%.3f r=%.3f rate=%.1f Hz",
      Lx, Ly, r, rate);
  }

private:
  void publish_odometry()
  {
    const auto p = chassis_->get_pose();
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = p.x;
    odom.pose.pose.position.y = p.y;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p.theta);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);

    // Broadcast TF (odom -> base_link)
    geometry_msgs::msg::TransformStamped t;
    t.header = odom.header;
    t.child_frame_id = "base_link";
    t.transform.translation.x = p.x;
    t.transform.translation.y = p.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(t);
  }

  std::unique_ptr<vehicle_wbt_platform_cpp::MecanumChassis> chassis_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumChassisNode>());
  rclcpp::shutdown();
  return 0;
}
