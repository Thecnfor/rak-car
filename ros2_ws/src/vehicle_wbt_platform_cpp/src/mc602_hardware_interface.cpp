// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602HardwareInterface — ros2_control SystemInterface wrapper around
// MC602Adapter. The ros2_control controller_manager calls export_*_interfaces()
// once at activation, then read() each cycle to populate state, then
// write() to push command values to hardware.

#include "vehicle_wbt_platform_cpp/mc602_hardware_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace vehicle_wbt_platform_cpp
{

MC602HardwareInterface::MC602HardwareInterface() = default;
MC602HardwareInterface::~MC602HardwareInterface() = default;

hardware_interface::CallbackReturn MC602HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Required params (from URDF <hardware><param .../></hardware>).
  const auto & hw = info.hardware_parameters;
  auto it_port = hw.find("serial_port");
  auto it_baud = hw.find("baud");
  if (it_port == hw.end() || it_baud == hw.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("MC602HardwareInterface"),
                 "missing required params: serial_port and baud");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string serial_port = it_port->second;
  const uint32_t baud = static_cast<uint32_t>(std::stoul(it_baud->second));

  adapter_ = std::make_unique<MC602Adapter>(serial_port, baud);

  // Optional: per-joint port map (joint name -> physical port_id on MC602).
  // Default: assume joint name suffix is the port_id (e.g. "wheel_m1" -> 1).
  for (const auto & joint : info.joints) {
    JointState js;
    joints_[joint.name] = js;

    // Parse port_id from joint name suffix "_m<N>" or fallback to 0.
    const auto & name = joint.name;
    auto pos = name.rfind("_m");
    if (pos != std::string::npos && pos + 2 < name.size()) {
      try {
        port_map_[name] = static_cast<uint8_t>(std::stoul(name.substr(pos + 2)));
      } catch (...) {
        port_map_[name] = 0;
      }
    } else {
      port_map_[name] = 0;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MC602HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!adapter_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    adapter_->open();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MC602HardwareInterface"), "on_activate: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MC602HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (adapter_) {
    adapter_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MC602HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & [name, js] : joints_) {
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &js.pos);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &js.vel);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> MC602HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & [name, js] : joints_) {
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &js.cmd_velocity);
  }
  return out;
}

hardware_interface::return_type MC602HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Phase 1.5: stub — populate zero state. Real impl reads encoders via
  // adapter_->read_sensor(port_id, "encoder") per wheel in Plan B.
  for (auto & [name, js] : joints_) {
    js.pos = 0.0;
    js.vel = 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MC602HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Phase 1.5: stub — record commanded velocities. Real impl dispatches
  // each js.cmd_velocity to adapter_->write_actuator(port_id, "motor", val)
  // in Plan B.
  for (const auto & [name, js] : joints_) {
    (void)name;
    (void)js;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace vehicle_wbt_platform_cpp
