// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BaseController — pure virtual interface for any motor controller adapter.
// C++ counterpart of vehicle_wbt_platform.controller_base.BaseControllerHardwareInterface.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件
//
// All concrete controller adapters (MC601, MC602, future MCxxx) implement this
// interface. Sidecar components depend only on BaseController, not on
// controller-specific classes. Tests substitute a fake without touching real
// hardware.

#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace vehicle_wbt_platform_cpp
{

class BaseController
{
public:
  virtual ~BaseController() = default;

  // --- Lifecycle ---
  virtual void open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual std::string serial_port() const = 0;
  virtual uint32_t baud() const = 0;

  // --- I/O ---
  // Read a sensor value (returns value in physical units: m, V, ...).
  // Throws std::runtime_error on protocol error or unsupported type/port.
  virtual double read_sensor(uint8_t port_id, const std::string & sensor_type) = 0;

  // Write a value to an actuator. Idempotent under repeated calls.
  // Throws std::runtime_error on protocol error or invalid port/value.
  virtual void write_actuator(uint8_t port_id, const std::string & actuator_type, double value) = 0;

  // --- Metadata ---
  // Return port-type -> count, e.g. {{"motor", 6}, {"servo", 7}, ...}.
  virtual std::map<std::string, uint32_t> enumerate_ports() const = 0;

  // Per-controller port caps; used by derived classes to validate port_id.
  // These constants match hardware-port-mapping.md §MC602 控制器.
  static constexpr uint8_t MOTOR_MAX = 6;
  static constexpr uint8_t SERVO_MAX = 7;
  static constexpr uint8_t STEPPER_MAX = 3;
  static constexpr uint8_t IO_MAX = 8;

protected:

};  // class BaseController

}  // namespace vehicle_wbt_platform_cpp
