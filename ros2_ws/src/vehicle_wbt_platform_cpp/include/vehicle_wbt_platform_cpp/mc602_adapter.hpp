// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602Adapter — concrete controller adapter for Waveshare MC602.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §硬件接口插件
//
// Wraps a POSIX termios serial handle (/dev/ttyUSB*) and exposes the
// BaseController interface. Real MC602 protocol framing is implemented in
// src/mc602_adapter.cpp using write()/read() syscalls (no rclcpp here —
// this class is independent of ROS2 so it can be unit-tested in isolation
// and reused by both sidecar nodes and ros2_control hardware interface).

#pragma once

#include "vehicle_wbt_platform_cpp/base_controller.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>

namespace vehicle_wbt_platform_cpp
{

class MC602Adapter : public BaseController
{
public:
  // baud must be one of: 380400 (MC601), 1000000 (MC602 USB), 115200 (MC602 wireless).
  MC602Adapter(std::string serial_port, uint32_t baud);
  ~MC602Adapter() override;

  MC602Adapter(const MC602Adapter &) = delete;
  MC602Adapter & operator=(const MC602Adapter &) = delete;

  // --- BaseController ---
  void open() override;
  void close() override;
  bool is_open() const override { return fd_ >= 0; }
  std::string serial_port() const override { return serial_port_; }
  uint32_t baud() const override { return baud_; }
  double read_sensor(uint8_t port_id, const std::string & sensor_type) override;
  void write_actuator(uint8_t port_id, const std::string & actuator_type, double value) override;
  std::map<std::string, uint32_t> enumerate_ports() const override;

  // --- Test seam ---
  // When set, read/write use the injection point instead of the real fd.
  // Used by gmock tests; no-op in production.
  void set_injection(std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> responder)
  {
    injection_ = std::move(responder);
  }

private:
  std::string serial_port_;
  uint32_t baud_;
  int fd_;  // -1 when closed

  // Test injection: replaces the real read() call. When set, send_frame()
  // returns whatever the lambda returns. Production sets injection_ = nullptr.
  std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> injection_;

  // Low-level protocol: send a request frame, block-read response, return
  // parsed value. Throws on protocol error.
  double send_frame_read(uint8_t port_id, const std::string & sensor_type);
  void send_frame_write(uint8_t port_id, const std::string & actuator_type, double value);

  // POSIX helpers (split out so tests can override injection_ without
  // touching the real fd).
  void open_serial();
  void close_serial();
};

}  // namespace vehicle_wbt_platform_cpp
