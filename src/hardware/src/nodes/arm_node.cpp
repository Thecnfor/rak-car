// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// ArmNode — subscribes /rak/cmd/arm/<id>/trajectory (JointTrajectory),
// commands MC602Adapter for real hardware (M6 motor, stepper 3, S3/S7 servos,
// P2 vacuum pump, P3 solenoid valve), and publishes joint state feedback.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §机械臂抽象
// Hardware: docs/hardware-port-mapping.md §机械臂
//
// Joint layout (matches trajectory message order):
//   [0] horiz_m6   — M6 horizontal lead-screw motor (port 6, dev_id=0x02)
//   [1] vert_stepper3 — stepper 3 vertical lead-screw (port 3, dev_id=0x11)
//   [2] rotate_s3  — S3 bus servo rotation (port 3, dev_id=0x06)
//   [3] grip_s7    — S7 PWM servo hand (port 7, dev_id=0x05, 270° mode)

#include "hardware/mc602_adapter.hpp"
#include "hardware/transport_factory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace vw = hardware;

// ---------------------------------------------------------------------------
// Constants from hardware-port-mapping.md
// ---------------------------------------------------------------------------

namespace
{
// Joint indices (must match joint_names_ order).
constexpr size_t J_HORIZ = 0;
constexpr size_t J_VERT  = 1;
constexpr size_t J_ROTATE = 2;
constexpr size_t J_GRIP  = 3;
constexpr size_t NUM_JOINTS = 4;

// Stepper 3 vertical: perimeter = 0.008 m/rev (from arm_cfg.yaml).
constexpr double STEPPER3_PERIMETER = 0.008;
// Max vertical speed: ±0.04 m/s (from arm_cfg.yaml PID tuning).
constexpr double VERT_MAX_SPEED_MPS = 0.04;
// Steps per meter: ENCODER_COUNTS_PER_REV / perimeter.
inline double stepper3_steps_per_meter()
{
  return vw::MC602Adapter::ENCODER_COUNTS_PER_REV / STEPPER3_PERIMETER;
}

// M6 horizontal lead screw: perimeter = 0.032 m (from arm_cfg.yaml).
constexpr double M6_PERIMETER = 0.032;
// Max horizontal speed: ±0.2 m/s.
constexpr double HORIZ_MAX_SPEED_MPS = 0.2;

// S3 rotation servo angle map (from arm_base.py).
// {-1: -93°, 0: 0°, 1: +93°}
constexpr double S3_ANGLE_MAP[3] = {-93.0, 0.0, 93.0};

// S7 hand servo angle map (from arm_base.py, 270° mode).
// {-1: -45°, 1: +46°}
constexpr double S7_ANGLE_MAP[3] = {-45.0, 0.0, 46.0};

// Vacuum pump: ON = 1 (suck), OFF = 0.
constexpr uint8_t PUMP_ON  = 2;  // MC602 DOUT connect
constexpr uint8_t PUMP_OFF = 1;  // MC602 DOUT disconnect

// Solenoid valve: 1 = closed (hold vacuum), 0 = open (release).
constexpr uint8_t VALVE_CLOSE = 2;
constexpr uint8_t VALVE_OPEN  = 1;

// P6 analog limit sensor threshold: value > 1000 = touch bottom.
constexpr uint16_t VERT_LIMIT_THRESHOLD = 1000;
}  // anonymous namespace

// ---------------------------------------------------------------------------
// ArmNode
// ---------------------------------------------------------------------------

class ArmNode : public rclcpp::Node
{
public:
  explicit ArmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("arm_node", options)
  {
    // --- Parameters ---
    this->declare_parameter<std::string>("arm_id", "main");
    this->declare_parameter<double>("publish_rate_hz", 50.0);
    this->declare_parameter<std::string>("mc602_serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("mc602_baud", 1000000);
    this->declare_parameter<std::string>("mc602_transport", "direct");

    arm_id_ = this->get_parameter("arm_id").as_string();
    const double rate = this->get_parameter("publish_rate_hz").as_double();
    const std::string port = this->get_parameter("mc602_serial_port").as_string();
    const int baud = this->get_parameter("mc602_baud").as_int();
    const std::string transport_mode = this->get_parameter("mc602_transport").as_string();

    // Joint names must match trajectory message joint_names order.
    joint_names_ = {"horiz_m6", "vert_stepper3", "rotate_s3", "grip_s7"};

    // --- MC602 hardware interface ---
    adapter_ = std::make_unique<vw::MC602Adapter>(
      vw::make_mc602_transport(this, transport_mode, port,
                               static_cast<uint32_t>(baud)));
    adapter_->open();
    RCLCPP_INFO(this->get_logger(),
      "MC602Adapter opened: %s @ %d baud via %s", port.c_str(), baud,
      transport_mode.c_str());

    // --- Subscriptions ---
    const std::string cmd_topic = "/rak/cmd/arm/" + arm_id_ + "/trajectory";
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      cmd_topic, 10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        this->on_trajectory(*msg);
      });

    // --- Publications ---
    const std::string state_topic = "/rak/state/actuators/" + arm_id_;
    state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic, 10);

    // --- Main loop timer ---
    const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    timer_ = this->create_wall_timer(period, [this]() { this->publish_state(); });

    RCLCPP_INFO(this->get_logger(),
      "ArmNode[%s] joints=%s rate=%.1f Hz",
      arm_id_.c_str(), cmd_topic.c_str(), rate);
  }

  ~ArmNode() override
  {
    try {
      // Safety: release pump + valve on shutdown.
      if (adapter_) {
        adapter_->write_actuator(2, "dout", 0);  // pump off
        adapter_->write_actuator(3, "dout", 0);  // valve open (release)
        adapter_->close();
      }
    } catch (...) {}
  }

private:
  // -----------------------------------------------------------------------
  // Trajectory callback: execute the last point immediately.
  // -----------------------------------------------------------------------
  void on_trajectory(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (traj.points.empty()) return;

    // Use the last point (most recent target).
    const auto & point = traj.points.back();

    // Expand positions to match NUM_JOINTS if trajectory has fewer.
    std::vector<double> positions(NUM_JOINTS, 0.0);
    for (size_t i = 0; i < std::min(point.positions.size(), NUM_JOINTS); ++i) {
      positions[i] = point.positions[i];
    }

    // --- Joint 0: M6 horizontal motor (position → m/s via perimeter) ---
    // Arm control uses position commands; we convert to speed for the motor.
    // Speed = position_delta / dt. Here we use a fixed dt = 1.0s per step,
    // so velocity = position (the caller sets velocity in position field).
    const double horiz_speed = std::clamp(positions[J_HORIZ],
      -HORIZ_MAX_SPEED_MPS, HORIZ_MAX_SPEED_MPS);
    const double horiz_virtual = vw::MC602Adapter::meters_to_virtual(horiz_speed, M6_PERIMETER);
    const int8_t horiz_cmd = static_cast<int8_t>(
      std::clamp(static_cast<int>(std::round(horiz_virtual)), -100, 100));

    // Pack all joint writes for this trajectory tick into ONE burst: a single
    // bridge service call, so the several frames hit the bus atomically and
    // cost one DDS round-trip. commit_burst() surfaces transport errors.
    try {
      adapter_->begin_burst();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "begin_burst failed: %s", e.what());
    }

    try {
      adapter_->write_actuator(6, "motor", horiz_speed);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "M6 motor write failed: %s", e.what());
    }

    // --- Joint 1: Stepper 3 vertical (position → steps) ---
    // Vertical position is in meters; convert to angle_deg for stepper.
    // steps = angle_deg / STEPPER_RAD_PER_STEP → angle_deg = steps * STEPPER_RAD_PER_STEP
    // We use position (m) → steps directly.
    const double vert_pos_m = std::clamp(positions[J_VERT], 0.0, 0.3);  // 0~0.3m range
    const double vert_steps = vert_pos_m * stepper3_steps_per_meter();
    const double vert_angle_deg = vert_steps * vw::MC602Adapter::STEPPER_RAD_PER_STEP
                                  * 180.0 / vw::MC602_PI;

    try {
      adapter_->write_actuator(3, "stepper", vert_angle_deg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Stepper3 write failed: %s", e.what());
    }

    // --- Joint 2: S3 rotation servo (position → angle_deg) ---
    // Position is -1 / 0 / +1 → mapped to -93° / 0° / +93°.
    double s3_angle = 0.0;
    const int side_idx = static_cast<int>(std::round(positions[J_ROTATE]));
    if (side_idx >= -1 && side_idx <= 1) {
      s3_angle = S3_ANGLE_MAP[side_idx + 1];
    }
    try {
      adapter_->write_actuator(3, "servo_bus", s3_angle);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "S3 servo write failed: %s", e.what());
    }

    // --- Joint 3: S7 hand servo (position → angle_deg, 270° mode) ---
    // Position is -1 / 0 / +1 → mapped to -45° / 0° / +46°.
    double s7_angle = 0.0;
    const int grip_idx = static_cast<int>(std::round(positions[J_GRIP]));
    if (grip_idx >= -1 && grip_idx <= 1) {
      s7_angle = S7_ANGLE_MAP[grip_idx + 1];
    }
    try {
      adapter_->write_actuator(7, "servo_pwm", s7_angle);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "S7 servo write failed: %s", e.what());
    }

    // --- Effort field: effort > 0 → pump ON, effort < 0 → pump OFF ---
    if (!point.effort.empty() && point.effort.size() > J_GRIP) {
      const uint8_t pump_cmd = (point.effort[J_GRIP] > 0.0) ? PUMP_ON : PUMP_OFF;
      try {
        adapter_->write_actuator(2, "dout", static_cast<double>(pump_cmd));
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Pump write failed: %s", e.what());
      }
      // Valve stays closed (1=hold vacuum) while pump is on.
      try {
        adapter_->write_actuator(3, "dout",
          static_cast<double>((point.effort[J_GRIP] > 0.0) ? VALVE_CLOSE : VALVE_OPEN));
      } catch (...) {}
    }

    // Send the whole tick's writes as one transaction.
    try {
      adapter_->commit_burst();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "burst commit failed: %s", e.what());
    }

    // Store last commanded positions for state publishing.
    last_positions_ = positions;
  }

  // -----------------------------------------------------------------------
  // Publish current joint state feedback (position + velocity = 0 for now).
  // Real impl reads back from encoders/servos; stub zeros until Plan B adds
  // feedback reads.
  // -----------------------------------------------------------------------
  void publish_state()
  {
    std::vector<double> positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      positions = last_positions_;
    }

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = joint_names_;
    msg->position = positions;
    msg->velocity.assign(NUM_JOINTS, 0.0);
    msg->effort.assign(NUM_JOINTS, 0.0);

    state_pub_->publish(std::move(msg));
  }

  // --- Members ---
  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::vector<double> last_positions_{NUM_JOINTS, 0.0};

  std::mutex state_mutex_;

  std::unique_ptr<vw::MC602Adapter> adapter_;

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded: bridge mode blocks the callback on a service round-trip;
  // extra threads keep subscriptions/timers serviced while it waits.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(std::make_shared<ArmNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
