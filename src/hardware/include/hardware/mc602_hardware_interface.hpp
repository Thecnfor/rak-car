// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602HardwareInterface — ros2_control SystemInterface for the MC602
// motor controller. Lets ros2_control manage the MC602 as part of a
// controller_manager-driven control loop.
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §7.2
//
// Use in URDF:
//   <ros2_control name="MC602" type="system">
//     <hardware>
//       <plugin>hardware/MC602HardwareInterface</plugin>
//       <param name="mc602_transport">bridge</param>
//       <param name="serial_port">/dev/ttyUSB0</param>
//       <param name="baud">1000000</param>
//     </hardware>
//     <joint name="wheel_fl"><command_interface name="velocity"/>
//                            <state_interface name="position"/>
//                            <state_interface name="velocity"/></joint>
//     ...
//   </ros2_control>
//
// Joint kinds are inferred from URDF joint names:
//   wheel_m<N>            → 4-wheel encoder4 (velocity cmd, position/velocity state)
//   arm_horiz_joint       → M6 lead-screw (position cmd via servo, encoder state)
//   arm_vert_joint        → stepper3 (position cmd, encoder state)
//   arm_hand_rotate_joint → S3 bus servo (position cmd, open-loop state)
//   arm_hand_grip_joint   → S7 PWM servo (position cmd, open-loop state)
//
// TRANSPORT: default "bridge" — ALL MC602 traffic goes through the
// mc602_bridge_node service (single-bus owner). The plugin never opens a
// serial fd directly. Direct transport is only for dev/tests.

#pragma once

#include "hardware/mc602_adapter.hpp"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hardware
{

// Kind of MC602 joint handled by this interface.
enum class Mc602JointKind : uint8_t
{
  WHEEL,    // velocity cmd + encoder4 state
  ARM_X,    // M6 lead-screw: position cmd (servoed) + encoder state
  ARM_Z,    // stepper3: position cmd + encoder state
  ARM_YAW,  // S3 bus servo: position cmd (deg) + open-loop state
  ARM_GRIP, // S7 PWM servo: position cmd (deg) + open-loop state
};

class MC602HardwareInterface : public hardware_interface::SystemInterface
{
public:
  MC602HardwareInterface();
  ~MC602HardwareInterface() override;

  // --- SystemInterface ---
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // --- Test seam (same pattern as MC602Adapter) ---
  void set_adapter(std::unique_ptr<MC602Adapter> adapter) { adapter_ = std::move(adapter); }

  // Per-joint configuration, parsed from URDF joint names + hardware params.
  struct JointConfig
  {
    std::string name;
    Mc602JointKind kind = Mc602JointKind::WHEEL;
    uint8_t port = 0;          // MC602 device port
    int wheel_index = -1;      // WHEEL: 0..3 (encoder4 order FL,FR,RL,RR)
    bool has_encoder = false;  // ARM_X/ARM_Z: encoder feedback
    // runtime command/state
    double cmd = 0.0;          // command (rad wheel / m arm_x / m arm_z / deg servos)
    double pos = 0.0;          // state position (same units as cmd)
    double vel = 0.0;          // state velocity (rad/s wheels / m/s arm)
  };

private:
  hardware_interface::CallbackReturn parse_hardware_params(const hardware_interface::HardwareInfo & info);
  hardware_interface::CallbackReturn add_joint_config(const hardware_interface::ComponentInfo & joint);
  void read_wheels(std::chrono::steady_clock::time_point now);
  void read_arm_encoder(JointConfig & j, std::chrono::steady_clock::time_point now);
  void write_arm_x(const JointConfig & j);
  void write_arm_z(const JointConfig & j);
  void log_throttled(const std::string & msg);

  std::unique_ptr<MC602Adapter> adapter_;

  std::map<std::string, JointConfig> joints_;

  // --- wheel encoder bookkeeping ---
  std::array<int32_t, 4> last_counts_{};
  std::chrono::steady_clock::time_point last_stamp_{};
  bool have_prev_{false};
  double counts_per_rev_{2015.13};   // hardware-port-mapping.md (48 × 41.98)
  double wheel_radius_{0.03};        // m (mecanum_chassis_node default)
  std::chrono::steady_clock::time_point last_err_stamp_{};

  // --- arm ports + physics (defaults from arm_node / hardware-port-mapping.md) ---
  uint8_t arm_x_port_{6};
  uint8_t arm_z_port_{3};
  uint8_t arm_yaw_port_{3};
  uint8_t arm_grip_port_{7};
  double arm_x_perimeter_{0.032};        // m / rev (M6 lead screw)
  double arm_x_max_speed_{0.2};          // m/s
  double arm_x_gain_{4.0};               // position servo kp [1/s]
  double arm_x_counts_per_rev_{2015.13}; // M6 encoder (must verify)
  double arm_z_steps_per_meter_{2015.13 / 0.008};  // stepper3 encoder/rev ÷ perimeter
  int32_t arm_z_velocity_{50};           // raw stepper velocity (arm_node default)
  double arm_yaw_deg_min_{-93.0};        // S3 angle range
  double arm_yaw_deg_max_{93.0};
  double arm_grip_deg_min_{-45.0};       // S7 angle range
  double arm_grip_deg_max_{46.0};
};

}  // namespace hardware
