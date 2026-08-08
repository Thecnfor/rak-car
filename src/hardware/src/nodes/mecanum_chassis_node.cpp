// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MecanumChassisNode — subscribes /rak/cmd/vel_safe (Twist),
// commands MC602Adapter::write_motor4 for real wheel speeds, reads
// MC602Adapter::read_encoder4 for encoder-based odometry, and publishes
// /rak/state/odom (Odometry) + /tf (TF).
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象
//
// Architecture (nav2/moveit split):
//   - MecanumChassis: pure kinematics (no ROS2, no hardware) — unchanged
//   - MC602Adapter: protocol layer — handles serial I/O
//   - MecanumChassisNode: ROS2 wiring — this file
//
// Wheel order (top-down, car facing +x):
//     M2(front-left)   M1(front-right)
//     M3(rear-left)    M4(rear-right)
//
// Encoder order from read_encoder4: [FL(M2), FR(M1), RL(M3), RR(M4)]

#include "hardware/mecanum_chassis.hpp"
#include "hardware/mc602_adapter.hpp"
#include "hardware/base_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace vw = hardware;

// ---------------------------------------------------------------------------
// OdomHelper — encoder-based odometry via forward kinematics
// ---------------------------------------------------------------------------

class OdomHelper
{
public:
  void init(double Lx, double Ly, double wheel_radius)
  {
    Lx_ = Lx;
    Ly_ = Ly;
    wheel_radius_ = wheel_radius;
    prev_counts_ = {0, 0, 0, 0};
    prev_time_ = rclcpp::Time(0);
    pose_.x = pose_.y = pose_.theta = 0.0;
  }

  void reset()
  {
    prev_counts_ = {0, 0, 0, 0};
    prev_time_ = rclcpp::Time(0);
    pose_.x = pose_.y = pose_.theta = 0.0;
  }

  bool update(const std::array<int32_t, 4> & counts, rclcpp::Time now,
              double & vx_out, double & vy_out, double & omega_out,
              vw::Pose2D & pose_out)
  {
    if (prev_time_.nanoseconds() == 0) {
      prev_counts_ = counts;
      prev_time_ = now;
      return false;
    }

    const double dt = (now - prev_time_).seconds();
    if (dt <= 0.0 || !std::isfinite(dt)) {
      prev_counts_ = counts;
      prev_time_ = now;
      return false;
    }

    // Wheel linear speeds (m/s) from encoder deltas.
    const double counts_per_rev = vw::MC602Adapter::ENCODER_COUNTS_PER_REV;
    const double circumference = 2.0 * vw::MC602_PI * wheel_radius_;
    std::array<double, 4> ws;
    for (int i = 0; i < 4; ++i) {
      const double delta_rev = static_cast<double>(counts[i] - prev_counts_[i]) / counts_per_rev;
      ws[i] = (delta_rev * circumference) / dt;
    }

    // Forward kinematics: wheel speeds → body velocity.
    const double k = Lx_ + Ly_;
    vx_out   = (ws[0] + ws[1] + ws[2] + ws[3]) / 4.0;
    vy_out   = (-ws[0] + ws[1] + ws[2] - ws[3]) / 4.0;
    omega_out = (-ws[0] + ws[1] - ws[2] + ws[3]) / (4.0 * k);

    // Integrate pose.
    pose_.x     += vx_out * dt;
    pose_.y     += vy_out * dt;
    pose_.theta += omega_out * dt;

    // Wrap theta to [-pi, pi].
    while (pose_.theta > vw::MC602_PI)  pose_.theta -= 2.0 * vw::MC602_PI;
    while (pose_.theta < -vw::MC602_PI) pose_.theta += 2.0 * vw::MC602_PI;

    pose_out = pose_;
    prev_counts_ = counts;
    prev_time_ = now;
    return true;
  }

  const vw::Pose2D & pose() const { return pose_; }

private:
  double Lx_{0.0};
  double Ly_{0.0};
  double wheel_radius_{0.03};
  std::array<int32_t, 4> prev_counts_;
  rclcpp::Time prev_time_;
  vw::Pose2D pose_;
};

// ---------------------------------------------------------------------------
// MecanumChassisNode
// ---------------------------------------------------------------------------

class MecanumChassisNode : public rclcpp::Node
{
public:
  explicit MecanumChassisNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mecanum_chassis_node", options),
    tf_broadcaster_(this),
    last_cmd_time_(this->now())
  {
    // --- Parameters ---
    this->declare_parameter<double>("chassis_Lx", 0.15);
    this->declare_parameter<double>("chassis_Ly", 0.10);
    this->declare_parameter<double>("wheel_radius", 0.03);
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<std::string>("mc602_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("mc602_baud", 1000000);

    const double Lx = this->get_parameter("chassis_Lx").as_double();
    const double Ly = this->get_parameter("chassis_Ly").as_double();
    const double r = this->get_parameter("wheel_radius").as_double();
    const double rate = this->get_parameter("publish_rate_hz").as_double();
    const std::string port = this->get_parameter("mc602_port").as_string();
    const int baud = this->get_parameter("mc602_baud").as_int();

    // --- Pure kinematics (independent of ROS2 / hardware) ---
    chassis_ = std::make_unique<vw::MecanumChassis>("mec1", Lx, Ly, r);

    // --- MC602 hardware interface ---
    adapter_ = std::make_unique<vw::MC602Adapter>(port, static_cast<uint32_t>(baud));
    adapter_->open();
    odom_.init(Lx, Ly, r);

    RCLCPP_INFO(this->get_logger(),
      "MC602Adapter opened: %s @ %d baud", port.c_str(), baud);

    // --- Subscriptions ---
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/rak/cmd/vel_safe", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_cmd_ = *msg;
        last_cmd_time_ = this->now();
      });

    // --- Publications ---
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/rak/state/odom", 10);

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/rak/state/joint_states", 10);

    // --- Main loop timer ---
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this]() { this->publish_odometry(); });

    RCLCPP_INFO(this->get_logger(),
      "MecanumChassisNode ready: Lx=%.3f Ly=%.3f r=%.3f rate=%.1f Hz",
      Lx, Ly, r, rate);
  }

  ~MecanumChassisNode() override
  {
    try {
      adapter_->close();
    } catch (...) {
      // destructor must not throw
    }
  }

private:
  void publish_odometry()
  {
    // 1. Read encoders + integrate odometry.
    std::array<int32_t, 4> counts;
    try {
      counts = adapter_->read_encoder4();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "read_encoder4 failed: %s", e.what());
      return;
    }

    const auto now = this->now();
    double vx_body, vy_body, omega_body;
    vw::Pose2D pose;
    const bool updated = odom_.update(counts, now, vx_body, vy_body, omega_body, pose);

    // 2. Publish Odometry.
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    if (updated) {
      q.setRPY(0.0, 0.0, pose.theta);
    } else {
      // First cycle: use current commanded omega for heading direction.
      double cmd_omega = 0.0;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cmd_omega = last_cmd_.angular.z;
      }
      q.setRPY(0.0, 0.0, pose.theta + cmd_omega * 0.02);
    }
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    if (updated) {
      odom.twist.twist.linear.x = vx_body;
      odom.twist.twist.linear.y = vy_body;
      odom.twist.twist.angular.z = omega_body;
    }
    odom_pub_->publish(odom);

    // 3. Broadcast TF (odom -> base_link).
    geometry_msgs::msg::TransformStamped t;
    t.header = odom.header;
    t.child_frame_id = "base_link";
    t.transform.translation.x = pose.x;
    t.transform.translation.y = pose.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(t);

    // 4. Publish joint states (wheel joints for diff_drive_controller compat).
    publish_joint_states(counts, now);

    // 5. Command motors from last Twist (if recently received).
    command_motors();
  }

  void publish_joint_states(const std::array<int32_t, 4> & counts, rclcpp::Time now)
  {
    // Publish wheel joint positions (radians) derived from encoder counts.
    // Joint names follow URDF convention: wheel_fl, wheel_fr, wheel_rl, wheel_rr.
    static sensor_msgs::msg::JointState js;
    static bool initialized = false;

    if (!initialized) {
      js.name = {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};
      js.position.resize(4, 0.0);
      js.velocity.resize(4, 0.0);
      initialized = true;
    }

    const double counts_per_rev = vw::MC602Adapter::ENCODER_COUNTS_PER_REV;
    const double wheel_r = chassis_->wheel_radius();

    for (int i = 0; i < 4; ++i) {
      js.position[i] = static_cast<double>(counts[i]) / counts_per_rev * 2.0 * vw::MC602_PI;
      js.velocity[i] = 0.0;  // velocities set by encoder delta in a full impl
    }
    js.header.stamp = now;
    joint_pub_->publish(js);
  }

  void command_motors()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Deadman: stop motors if no cmd received within 200 ms.
    const auto since_cmd = (this->now() - last_cmd_time_).nanoseconds() / 1e6;
    if (since_cmd > 200.0) {
      // Only publish stop if we were previously non-zero (avoid serial spam).
      if (last_cmd_.linear.x != 0.0 || last_cmd_.linear.y != 0.0 || last_cmd_.angular.z != 0.0) {
        try {
          adapter_->write_motor4(0, 0, 0, 0);
        } catch (...) {}
      }
      return;
    }

    // Inverse kinematics: Twist → wheel virtual speeds.
    const double vx = last_cmd_.linear.x;
    const double vy = last_cmd_.linear.y;
    const double omega = last_cmd_.angular.z;
    const auto ws = chassis_->inverse(vx, vy, omega);

    // Convert rad/s → virtual int8 via MC602Adapter.
    const double r = chassis_->wheel_radius();
    const int8_t v_fl = static_cast<int8_t>(
      std::clamp(static_cast<int>(std::round(vw::MC602Adapter::meters_to_virtual(ws.values[0], r))),
                 -100, 100));
    const int8_t v_fr = static_cast<int8_t>(
      std::clamp(static_cast<int>(std::round(vw::MC602Adapter::meters_to_virtual(ws.values[1], r))),
                 -100, 100));
    const int8_t v_rl = static_cast<int8_t>(
      std::clamp(static_cast<int>(std::round(vw::MC602Adapter::meters_to_virtual(ws.values[2], r))),
                 -100, 100));
    const int8_t v_rr = static_cast<int8_t>(
      std::clamp(static_cast<int>(std::round(vw::MC602Adapter::meters_to_virtual(ws.values[3], r))),
                 -100, 100));

    try {
      adapter_->write_motor4(v_fl, v_fr, v_rl, v_rr);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "write_motor4 failed: %s", e.what());
    }
  }

  // --- Members ---
  std::unique_ptr<vw::MecanumChassis> chassis_;
  std::unique_ptr<vw::MC602Adapter> adapter_;
  OdomHelper odom_;

  std::mutex state_mutex_;
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumChassisNode>());
  rclcpp::shutdown();
  return 0;
}
