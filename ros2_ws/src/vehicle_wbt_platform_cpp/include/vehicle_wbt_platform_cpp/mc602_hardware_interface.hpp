// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602HardwareInterface — ros2_control SystemInterface for the MC602
// motor controller. Lets ros2_control manage the MC602 as part of a
// controller_manager-driven control loop.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件
//
// Use in URDF:
//   <ros2_control name="MC602" type="system">
//     <hardware>
//       <plugin>vehicle_wbt_platform_cpp/MC602HardwareInterface</plugin>
//       <param name="serial_port">/dev/ttyUSB0</param>
//       <param name="baud">1000000</param>
//     </hardware>
//     <joint name="wheel_fl"><command_interface name="velocity"/>
//                            <state_interface name="position"/>
//                            <state_interface name="velocity"/></joint>
//     ...
//   </ros2_control>
//
// ros2_control then loads this plugin via pluginlib, calls on_init() with
// the URDF params, exposes 4 wheel velocity command interfaces, and reads
// wheel encoder state every cycle.

#pragma once

#include "vehicle_wbt_platform_cpp/mc602_adapter.hpp"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>

#include <map>
#include <string>
#include <vector>

namespace vehicle_wbt_platform_cpp
{

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

private:
  std::unique_ptr<MC602Adapter> adapter_;

  // Per-joint velocity command + position/velocity state, indexed by joint name.
  struct JointState
  {
    double cmd_velocity{0.0};
    double pos{0.0};
    double vel{0.0};
  };
  std::map<std::string, JointState> joints_;

  // Map URDF joint name -> physical wheel port_id (from <param name="port_map" ...>).
  std::map<std::string, uint8_t> port_map_;
};

}  // namespace vehicle_wbt_platform_cpp
