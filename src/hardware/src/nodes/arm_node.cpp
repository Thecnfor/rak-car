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
  constexpr double kN_COUNTS_PER_REV = 2015.13;  // hardware-port-mapping.md
  return kN_COUNTS_PER_REV / STEPPER3_PERIMETER;
}

// M6 horizontal lead screw: perimeter = 0.032 m (from arm_cfg.yaml).
constexpr double M6_PERIMETER = 0.032;
// Max horizontal speed: ±0.2 m/s.
constexpr double HORIZ_MAX_SPEED_MPS = 0.2;

// Continuous servo command limits. The MC602 conversion helpers map these
// physical angles into each servo's raw command range.
constexpr double SERVO_ANGLE_MIN_DEG = -150.0;
constexpr double SERVO_ANGLE_MAX_DEG = 150.0;

// Analog P5 magnetic bottom limit: raw value > 1000 means z = 0.0 m.
constexpr uint8_t VERT_LIMIT_PORT = 5;

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
        adapter_->set_dout(2, 0);  // pump off
        adapter_->set_dout(3, 0);  // valve open (release)
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

    // --- Joint 0: M6 horizontal motor (position → m/s → virtual speed) ---
    // Arm control uses position commands; we convert to speed for the motor.
    // Speed = position_delta / dt. Here we use a fixed dt = 1.0s per step,
    // so velocity = position (the caller sets velocity in position field).
    const double horiz_speed = std::clamp(positions[J_HORIZ],
      -HORIZ_MAX_SPEED_MPS, HORIZ_MAX_SPEED_MPS);
    const int8_t horiz_cmd = adapter_->mps_to_virtual(horiz_speed, M6_PERIMETER);

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
      adapter_->set_motor(6, horiz_cmd);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "M6 motor write failed: %s", e.what());
    }

    // --- Joint 1: Stepper 3 vertical (position + configurable velocity) ---
    const double vert_pos_m = std::clamp(positions[J_VERT], 0.0, 0.3);
    double vert_speed_mps = VERT_MAX_SPEED_MPS;
    if (point.velocities.size() > J_VERT && std::isfinite(point.velocities[J_VERT])) {
      vert_speed_mps = std::clamp(std::abs(point.velocities[J_VERT]), 0.0,
                                  VERT_MAX_SPEED_MPS);
    }
    const bool moving_down = vert_pos_m < last_positions_[J_VERT];
    try {
      const uint16_t p5 = adapter_->read_analog(VERT_LIMIT_PORT);
      if (p5 > VERT_LIMIT_THRESHOLD) {
        vert_zeroed_ = true;
        last_positions_[J_VERT] = 0.0;
        if (moving_down) {
          vert_speed_mps = 0.0;
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "P5 magnetic limit read failed: %s", e.what());
    }
    const int32_t vert_velocity = static_cast<int32_t>(
      std::round(vert_speed_mps * stepper3_steps_per_meter()));
    const int32_t vert_steps = static_cast<int32_t>(
      std::round(vert_pos_m * stepper3_steps_per_meter()));

    try {
      adapter_->set_stepper(3, vert_velocity, vert_steps);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Stepper3 write failed: %s", e.what());
    }

    // --- Joint 2: S3 rotation servo (continuous physical angle) ---
    const double s3_angle = std::clamp(positions[J_ROTATE],
      SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    try {
      adapter_->set_servo_bus(3, vw::MC602Adapter::deg_to_servo_bus(
        s3_angle, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "S3 servo write failed: %s", e.what());
    }

    // --- Joint 3: S7 hand servo (continuous physical angle) ---
    const double s7_angle = std::clamp(positions[J_GRIP],
      SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    try {
      adapter_->set_servo_pwm(7, vw::MC602Adapter::deg_to_servo_pwm(
        s7_angle, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "S7 servo write failed: %s", e.what());
    }

    // --- Effort field: effort > 0 → pump ON, effort < 0 → pump OFF ---
    if (!point.effort.empty() && point.effort.size() > J_GRIP) {
      const int8_t pump_cmd = (point.effort[J_GRIP] > 0.0) ? PUMP_ON : PUMP_OFF;
      try {
        adapter_->set_dout(2, pump_cmd);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Pump write failed: %s", e.what());
      }
      // Valve stays closed (hold vacuum) while pump is on.
      try {
        adapter_->set_dout(3,
          (point.effort[J_GRIP] > 0.0) ? VALVE_CLOSE : VALVE_OPEN);
      } catch (...) {}
    }

    // Send the whole tick's writes as one transaction.
    try {
      adapter_->commit_burst();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "burst commit failed: %s", e.what());
    }

    last_positions_ = positions;
    last_positions_[J_VERT] = vert_pos_m;
    last_velocities_ = std::vector<double>(NUM_JOINTS, 0.0);
    if (point.velocities.size() > J_VERT) {
      last_velocities_[J_VERT] = vert_speed_mps;
    }
  }

  // -----------------------------------------------------------------------
  // Publish current joint state feedback (position + velocity = 0 for now).
  // Real impl reads back from encoders/servos; stub zeros until Plan B adds
  // feedback reads.
  // -----------------------------------------------------------------------
  void publish_state()
  {
    std::vector<double> positions;
    std::vector<double> velocities;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      positions = last_positions_;
      velocities = last_velocities_;
    }

    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = joint_names_;
    msg->position = positions;
    msg->velocity = velocities;
    msg->effort.assign(NUM_JOINTS, 0.0);

    state_pub_->publish(std::move(msg));
  }

  // --- Members ---
  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::vector<double> last_positions_{NUM_JOINTS, 0.0};
  std::vector<double> last_velocities_{NUM_JOINTS, 0.0};
  bool vert_zeroed_{false};

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
  // Keep the node alive for the entire executor lifetime. Executor stores
  // node references weakly; a temporary shared_ptr would destroy the node
  // immediately after add_node(), leaving a spinning but empty process.
  auto node = std::make_shared<ArmNode>();
  // Multi-threaded: bridge mode blocks the callback on a service round-trip;
  // extra threads keep subscriptions/timers serviced while it waits.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
