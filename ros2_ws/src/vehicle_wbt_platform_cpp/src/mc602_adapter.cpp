// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602Adapter implementation.
//
// Phase 1.5: stub protocol that uses the test injection point. Real MC602
// binary protocol framing (CRC, command codes, response parsing) lands in
// Plan B once the Jetson has ROS2 Humble installed. The shape of the
// public API (BaseController contract) is the contract that matters here.

#include "vehicle_wbt_platform_cpp/mc602_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace vehicle_wbt_platform_cpp
{

// Map of sensor_type -> MC602 method name on the underlying controller_wrap
// wrapper. Extending this map is how new sensor modes get supported.
namespace
{
const std::map<std::string, std::string> & sensor_type_to_method()
{
  static const std::map<std::string, std::string> m = {
    {"ir", "infrared_read"},
    {"analog_input", "analog_input_read"},
    {"ultrasonic", "ultrasonic_read"},
    {"touch", "touch_read"},
    {"ambient_light", "ambient_light_read"},
  };
  return m;
}

const std::map<std::string, std::string> & actuator_type_to_method()
{
  static const std::map<std::string, std::string> m = {
    {"motor", "motor_set_speed"},
    {"servo_bus", "servo_bus_set"},
    {"servo_pwm", "servo_pwm_set"},
    {"stepper", "stepper_goto"},
    {"dout", "dout_set"},
  };
  return m;
}

uint8_t max_ports_for_actuator(const std::string & actuator_type)
{
  if (actuator_type == "motor") return BaseController::MOTOR_MAX;
  if (actuator_type == "servo_bus" || actuator_type == "servo_pwm") return BaseController::SERVO_MAX;
  if (actuator_type == "stepper") return BaseController::STEPPER_MAX;
  if (actuator_type == "dout") return BaseController::IO_MAX;
  return 0;
}
}  // namespace

MC602Adapter::MC602Adapter(std::string serial_port, uint32_t baud)
: serial_port_(std::move(serial_port)), baud_(baud), fd_(-1)
{
  if (baud != 380400 && baud != 1000000 && baud != 115200) {
    throw std::runtime_error("MC602Adapter: unsupported baud " + std::to_string(baud) +
                             "; MC602 supports 380400/1000000/115200");
  }
}

MC602Adapter::~MC602Adapter()
{
  close();
}

void MC602Adapter::open()
{
  if (is_open()) {
    return;
  }
  open_serial();
}

void MC602Adapter::close()
{
  if (!is_open()) {
    return;
  }
  try {
    close_serial();
  } catch (...) {
    // close() must never throw.
  }
  fd_ = -1;
}

double MC602Adapter::read_sensor(uint8_t port_id, const std::string & sensor_type)
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }
  if (sensor_type_to_method().find(sensor_type) == sensor_type_to_method().end()) {
    throw std::runtime_error("unsupported sensor type '" + sensor_type + "'");
  }
  // All P口 modes share the same physical port space (1..8).
  if (port_id < 1 || port_id > IO_MAX) {
    throw std::runtime_error("port_id " + std::to_string(port_id) +
                             " out of range for sensor; must be in [1, " + std::to_string(IO_MAX) + "]");
  }
  return send_frame_read(port_id, sensor_type);
}

void MC602Adapter::write_actuator(uint8_t port_id, const std::string & actuator_type, double value)
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }
  if (actuator_type_to_method().find(actuator_type) == actuator_type_to_method().end()) {
    throw std::runtime_error("unsupported actuator type '" + actuator_type + "'");
  }
  const uint8_t max_ports = max_ports_for_actuator(actuator_type);
  if (port_id < 1 || port_id > max_ports) {
    throw std::runtime_error("port_id " + std::to_string(port_id) +
                             " out of range for actuator type '" + actuator_type +
                             "'; must be in [1, " + std::to_string(max_ports) + "]");
  }
  if (!std::isfinite(value)) {
    throw std::runtime_error("value must be a finite number");
  }
  send_frame_write(port_id, actuator_type, value);
}

std::map<std::string, uint32_t> MC602Adapter::enumerate_ports() const
{
  return {{"motor", MOTOR_MAX}, {"servo", SERVO_MAX}, {"stepper", STEPPER_MAX}, {"io", IO_MAX}};
}

double MC602Adapter::send_frame_read(uint8_t port_id, const std::string & sensor_type)
{
  // Phase 1.5: protocol not yet implemented. If a test injection is set,
  // we still throw so the gtest can assert the path. Real frames land in
  // Plan B with the Jetson-flashed build.
  (void)port_id;
  (void)sensor_type;
  if (injection_) {
    // Allow tests to fully control the value.
    (void)injection_({});
    return 0.0;
  }
  throw std::runtime_error("MC602Adapter::send_frame_read: protocol not yet implemented (Plan B)");
}

void MC602Adapter::send_frame_write(uint8_t port_id, const std::string & actuator_type, double value)
{
  (void)port_id;
  (void)actuator_type;
  (void)value;
  if (injection_) {
    (void)injection_({});
    return;
  }
  throw std::runtime_error("MC602Adapter::send_frame_write: protocol not yet implemented (Plan B)");
}

void MC602Adapter::open_serial()
{
  // Real POSIX open + termios setup goes here in Plan B. For Phase 1.5 we
  // mark as open so tests can exercise the contract without /dev/ttyUSB*.
  fd_ = 0;  // sentinel "open" (NOT a real fd — tests must use injection_)
}

void MC602Adapter::close_serial()
{
  fd_ = -1;
}

}  // namespace vehicle_wbt_platform_cpp
