// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602HardwareInterface — ros2_control SystemInterface wrapper around
// MC602Adapter. The ros2_control controller_manager calls export_*_interfaces()
// once at activation, then read() each cycle to populate state, then
// write() to push command values to hardware.
//
// Transport is bridge-first (mc602_bridge_node owns the serial fd). All MC602
// traffic from ros2_control flows through the bridge service via an
// executor-safe BridgeTransport (dedicated client node + worker thread).

#include "hardware/mc602_hardware_interface.hpp"
#include "hardware/bridge_transport.hpp"
#include "hardware/serial_transport.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace hardware
{

namespace
{

// Wheel joint name → encoder4 index. Encoder order (mecanum_chassis_node):
// [FL(M2), FR(M1), RL(M3), RR(M4)].
inline int wheel_encoder_index(uint8_t m)
{
  switch (m) {
    case 1: return 1;  // M1 = front-right
    case 2: return 0;  // M2 = front-left
    case 3: return 2;  // M3 = rear-left
    case 4: return 3;  // M4 = rear-right
    default: return static_cast<int>(m) - 1;
  }
}

constexpr double kTwoPi = 2.0 * MC602_PI;

}  // namespace

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

  if (parse_hardware_params(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info.joints) {
    if (add_joint_config(joint) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MC602HardwareInterface::parse_hardware_params(
  const hardware_interface::HardwareInfo & info)
{
  const auto & hw = info.hardware_parameters;
  const std::string transport_mode = [&]() {
    auto it = hw.find("mc602_transport");
    return (it != hw.end()) ? it->second : std::string("bridge");
  }();

  if (transport_mode != "bridge" && transport_mode != "direct") {
    RCLCPP_ERROR(rclcpp::get_logger("MC602HardwareInterface"),
      "mc602_transport must be 'bridge' or 'direct', got '%s'", transport_mode.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!adapter_) {
    std::shared_ptr<SerialTransport> transport;
    if (transport_mode == "bridge") {
      // nullptr node → dedicated client node from the global default context.
      transport = std::make_shared<BridgeTransport>(
        nullptr, "/rak/hw/mc602/transaction");
    } else {
      auto it_port = hw.find("serial_port");
      auto it_baud = hw.find("baud");
      if (it_port == hw.end() || it_baud == hw.end()) {
        RCLCPP_ERROR(rclcpp::get_logger("MC602HardwareInterface"),
          "direct transport requires params: serial_port and baud");
        return hardware_interface::CallbackReturn::ERROR;
      }
      transport = std::make_shared<DirectSerialTransport>(
        it_port->second, static_cast<uint32_t>(std::stoul(it_baud->second)));
    }
    adapter_ = std::make_unique<MC602Adapter>(transport);
  }

  auto get_d = [&](const char * key, double dflt) {
    auto it = hw.find(key);
    return (it != hw.end()) ? std::stod(it->second) : dflt;
  };
  auto get_u8 = [&](const char * key, uint8_t dflt) {
    auto it = hw.find(key);
    return (it != hw.end()) ? static_cast<uint8_t>(std::stoul(it->second)) : dflt;
  };

  counts_per_rev_ = get_d("wheel_counts_per_rev", counts_per_rev_);
  wheel_radius_ = get_d("wheel_radius", wheel_radius_);
  arm_x_port_ = get_u8("arm_x_port", arm_x_port_);
  arm_z_port_ = get_u8("arm_z_port", arm_z_port_);
  arm_yaw_port_ = get_u8("arm_yaw_port", arm_yaw_port_);
  arm_grip_port_ = get_u8("arm_grip_port", arm_grip_port_);
  arm_x_perimeter_ = get_d("arm_x_perimeter", arm_x_perimeter_);
  arm_x_max_speed_ = get_d("arm_x_max_speed", arm_x_max_speed_);
  arm_x_gain_ = get_d("arm_x_gain", arm_x_gain_);
  arm_x_counts_per_rev_ = get_d("arm_x_counts_per_rev", arm_x_counts_per_rev_);
  arm_z_steps_per_meter_ = get_d("arm_z_steps_per_meter", arm_z_steps_per_meter_);
  arm_z_velocity_ = static_cast<int32_t>(get_d("arm_z_velocity", arm_z_velocity_));
  arm_yaw_deg_min_ = get_d("arm_yaw_deg_min", arm_yaw_deg_min_);
  arm_yaw_deg_max_ = get_d("arm_yaw_deg_max", arm_yaw_deg_max_);
  arm_grip_deg_min_ = get_d("arm_grip_deg_min", arm_grip_deg_min_);
  arm_grip_deg_max_ = get_d("arm_grip_deg_max", arm_grip_deg_max_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MC602HardwareInterface::add_joint_config(
  const hardware_interface::ComponentInfo & joint)
{
  JointConfig cfg;
  cfg.name = joint.name;
  const auto & name = joint.name;

  if (name.rfind("wheel_m", 0) == 0) {
    cfg.kind = Mc602JointKind::WHEEL;
    const uint8_t m = static_cast<uint8_t>(
      std::stoul(name.substr(std::strlen("wheel_m"))));
    cfg.port = m;
    cfg.wheel_index = wheel_encoder_index(m);
  } else if (name == "arm_horiz_joint") {
    cfg.kind = Mc602JointKind::ARM_X;
    cfg.port = arm_x_port_;
    cfg.has_encoder = true;
  } else if (name == "arm_vert_joint") {
    cfg.kind = Mc602JointKind::ARM_Z;
    cfg.port = arm_z_port_;
    cfg.has_encoder = true;
  } else if (name == "arm_hand_rotate_joint") {
    cfg.kind = Mc602JointKind::ARM_YAW;
    cfg.port = arm_yaw_port_;
  } else if (name == "arm_hand_grip_joint") {
    cfg.kind = Mc602JointKind::ARM_GRIP;
    cfg.port = arm_grip_port_;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MC602HardwareInterface"),
      "unsupported joint '%s' for MC602HardwareInterface", name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_[name] = cfg;
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
    if (js.kind == Mc602JointKind::WHEEL) {
      out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &js.cmd);
    } else {
      out.emplace_back(name, hardware_interface::HW_IF_POSITION, &js.cmd);
    }
  }
  return out;
}

hardware_interface::return_type MC602HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto now = std::chrono::steady_clock::now();
  try {
    read_wheels(now);
  } catch (const std::exception & e) {
    log_throttled(std::string("encoder4 read failed: ") + e.what());
  }
  for (auto & [name, js] : joints_) {
    if (js.has_encoder) {
      try {
        read_arm_encoder(js, now);
      } catch (const std::exception & e) {
        log_throttled("arm encoder read failed on '" + name + "': " + e.what());
      }
    } else if (js.kind == Mc602JointKind::ARM_YAW ||
               js.kind == Mc602JointKind::ARM_GRIP) {
      // Open-loop axes: state = last commanded (confidence handled upstream).
      js.vel = 0.0;
    }
  }
  return hardware_interface::return_type::OK;
}

void MC602HardwareInterface::read_wheels(std::chrono::steady_clock::time_point now)
{
  bool has_wheels = false;
  for (const auto & [name, js] : joints_) {
    if (js.kind == Mc602JointKind::WHEEL) {
      has_wheels = true;
      break;
    }
  }
  if (!has_wheels) {
    return;
  }

  const auto counts = adapter_->read_encoder4();
  const double dt = std::chrono::duration<double>(now - last_stamp_).count();
  for (auto & [name, js] : joints_) {
    if (js.kind != Mc602JointKind::WHEEL) {
      continue;
    }
    const int i = js.wheel_index;
    if (i < 0 || i >= 4) {
      continue;
    }
    js.pos = static_cast<double>(counts[i]) / counts_per_rev_ * kTwoPi;
    if (have_prev_ && dt > 0.0) {
      const double delta = static_cast<double>(counts[i] - last_counts_[i]);
      js.vel = delta / counts_per_rev_ * kTwoPi / dt;
    } else {
      js.vel = 0.0;
    }
  }
  last_counts_ = counts;
  last_stamp_ = now;
  have_prev_ = true;
}

void MC602HardwareInterface::read_arm_encoder(
  JointConfig & j, std::chrono::steady_clock::time_point /*now*/)
{
  const int32_t c = adapter_->read_encoder(j.port);
  if (j.kind == Mc602JointKind::ARM_X) {
    // counts → carriage position (m)
    j.pos = static_cast<double>(c) / arm_x_counts_per_rev_ * arm_x_perimeter_;
  } else if (j.kind == Mc602JointKind::ARM_Z) {
    // counts (=steps for the stepper) → height (m)
    j.pos = static_cast<double>(c) / arm_z_steps_per_meter_;
  }
  j.vel = 0.0;  // velocity derived upstream (odom/trajectory), not here
}

hardware_interface::return_type MC602HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // --- 4-wheel velocity command → one motor4 frame ---
  bool has_wheels = false;
  int8_t s[4] = {0, 0, 0, 0};
  for (const auto & [name, js] : joints_) {
    if (js.kind != Mc602JointKind::WHEEL) {
      continue;
    }
    has_wheels = true;
    const int i = js.wheel_index;
    if (i < 0 || i >= 4) {
      continue;
    }
    // Command interface is wheel angular velocity (rad/s) → virtual int8.
    const double linear_mps = js.cmd * wheel_radius_;
    s[i] = adapter_->mps_to_virtual(linear_mps, wheel_radius_);
  }
  if (has_wheels) {
    try {
      adapter_->set_motor4(s[0], s[1], s[2], s[3]);
    } catch (const std::exception & e) {
      log_throttled(std::string("set_motor4 failed: ") + e.what());
    }
  }

  // --- Arm joints (position command) ---
  for (const auto & [name, js] : joints_) {
    try {
      switch (js.kind) {
        case Mc602JointKind::ARM_X:
          write_arm_x(js);
          break;
        case Mc602JointKind::ARM_Z:
          write_arm_z(js);
          break;
        case Mc602JointKind::ARM_YAW: {
          const int raw = MC602Adapter::deg_to_servo_bus(
            js.cmd, arm_yaw_deg_min_, arm_yaw_deg_max_);
          adapter_->set_servo_bus(js.port, raw);
          break;
        }
        case Mc602JointKind::ARM_GRIP: {
          const int raw = MC602Adapter::deg_to_servo_pwm(
            js.cmd, arm_grip_deg_min_, arm_grip_deg_max_);
          adapter_->set_servo_pwm(js.port, raw);
          break;
        }
        case Mc602JointKind::WHEEL:
          break;
      }
    } catch (const std::exception & e) {
      log_throttled("arm write failed on '" + name + "': " + e.what());
    }
  }
  return hardware_interface::return_type::OK;
}

void MC602HardwareInterface::write_arm_x(const JointConfig & j)
{
  // M6 is a velocity-controlled lead-screw motor. Position command (m) is
  // servoed with a P-controller using the encoder-measured carriage position.
  const double err = j.cmd - j.pos;
  double vel_mps = err * arm_x_gain_;
  vel_mps = std::clamp(vel_mps, -arm_x_max_speed_, arm_x_max_speed_);
  const int8_t virt = adapter_->mps_to_virtual(vel_mps, arm_x_perimeter_);
  adapter_->set_motor(j.port, virt);
}

void MC602HardwareInterface::write_arm_z(const JointConfig & j)
{
  // Stepper3 takes absolute (velocity, position-in-steps).
  const int32_t steps = static_cast<int32_t>(
    std::round(j.cmd * arm_z_steps_per_meter_));
  adapter_->set_stepper(j.port, arm_z_velocity_, steps);
}

void MC602HardwareInterface::log_throttled(const std::string & msg)
{
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_err_stamp_).count() >= 1.0) {
    RCLCPP_ERROR(rclcpp::get_logger("MC602HardwareInterface"), "%s", msg.c_str());
    last_err_stamp_ = now;
  }
}

}  // namespace hardware
