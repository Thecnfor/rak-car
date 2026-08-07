// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602Adapter — concrete controller adapter for Waveshare MC602.
//
// Implements the real MC602 binary protocol (docs/hardware-comm.md §MC602):
//   Frame: 0x77 0x68 | length | dev_id mode port params... | 0x0A
//   Response payload is res[3:-1] (strips header, length, footer).
//
// The adapter wraps a SerialPort (POSIX termios) and exposes the
// BaseController interface. Independent of ROS2 — testable in isolation.

#pragma once

#include "vehicle_wbt_platform_cpp/base_controller.hpp"
#include "vehicle_wbt_platform_cpp/serial_port.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace vehicle_wbt_platform_cpp
{

// C++17-safe π (std::numbers::pi requires C++20).
inline constexpr double MC602_PI = 3.14159265358979323846;

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

  // --- Extended read/write (not in BaseController but needed by C++ nodes) ---

  // Read 4 motor encoders (dev_id=0x03, mode=1) → [FL(M2), FR(M1), RL(M3), RR(M4)]
  // Returns raw int32 counts per wheel.
  std::array<int32_t, 4> read_encoder4();

  // Read single IR sensor (dev_id=0x07, mode=SENSOR_INFRARED) → meters.
  float read_infrared(uint8_t port_id);

  // Write 4 motors simultaneously (dev_id=0x01, mode=2) with int8 virtual speeds.
  void write_motor4(int8_t v_fl, int8_t v_fr, int8_t v_rl, int8_t v_rr);

  // --- Test seam ---
  // When set, read/write use the injection point instead of the real fd.
  void set_injection(std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> responder);

  // MC602 protocol constants (per docs/hardware-comm.md)
  static constexpr uint8_t DEV_MOTOR4       = 0x01;
  static constexpr uint8_t DEV_MOTOR        = 0x02;
  static constexpr uint8_t DEV_ENCODER4     = 0x03;
  static constexpr uint8_t DEV_ENCODER      = 0x04;
  static constexpr uint8_t DEV_SERVO_PWM    = 0x05;
  static constexpr uint8_t DEV_SERVO_BUS    = 0x06;
  static constexpr uint8_t DEV_SENSOR_MULTI = 0x07;
  static constexpr uint8_t DEV_DOUT         = 0x10;
  static constexpr uint8_t DEV_STEPPER      = 0x11;

  static constexpr uint8_t MODE_GET   = 1;
  static constexpr uint8_t MODE_SET   = 2;
  static constexpr uint8_t MODE_RESET = 3;

  static constexpr uint8_t SENSOR_ANALOG     = 0;
  static constexpr uint8_t SENSOR_INFRARED   = 1;
  static constexpr uint8_t SENSOR_TOUCH      = 2;
  static constexpr uint8_t SENSOR_ULTRASONIC = 3;
  static constexpr uint8_t SENSOR_AMBIENT    = 4;

  // Encoder calibration constants
  static constexpr double ENCODER_COUNTS_PER_REV = 2015.13;  // 48 * 41.98
  static constexpr double ENCODER_2_RAD = 2.0 * MC602_PI / ENCODER_COUNTS_PER_REV;  // ~0.003118

  // Wheel kinematics constants
  static constexpr double RAD_2_VIRTUAL = ENCODER_COUNTS_PER_REV / (2.0 * MC602_PI * 100.0);  // ~3.207

  // Stepper conversion
  static constexpr double STEPPER_RAD_PER_STEP = MC602_PI / 180.0 * 1.8 / 16.0;  // ~0.001963

  // MC602-specific port limits (override BaseController generic values).
  static constexpr uint8_t MC602_MOTOR_PORTS = 4;   // MC602: ports 1-4
  static constexpr uint8_t MC602_SERVO_PORTS = 7;   // MC602: ports 1-7
  static constexpr uint8_t MC602_STEPPER_PORTS = 4; // MC602: ports 1-4
  static constexpr uint8_t MC602_IO_PORTS = 16;     // MC602: ports 1-16

  // Frame format bytes
  static constexpr uint8_t FRAME_HEADER_0 = 0x77;
  static constexpr uint8_t FRAME_HEADER_1 = 0x68;
  static constexpr uint8_t FRAME_FOOTER   = 0x0A;

  // --- Protocol helpers (public for test verification) ---

  // Build a GET command frame: 0x77 0x68 | len | dev_id mode port params... | 0x0A
  static std::vector<uint8_t> build_get(uint8_t dev_id, uint8_t port,
                                         std::vector<uint8_t> params = {});

  // Build a SET command frame (mode = MODE_SET).
  static std::vector<uint8_t> build_set(uint8_t dev_id, uint8_t port,
                                         std::vector<uint8_t> params);

  // Send frame via serial_, read response, return payload (strips header/footer/length).
  // Throws std::runtime_error on timeout or protocol error.
  std::vector<uint8_t> exchange(const std::vector<uint8_t> & frame);

  // --- Unit conversion (public for unit tests) ---
  static int8_t clamp_virtual(double v_mps, double wheel_radius = 0.03);  // m/s → int8 [-100,100]
  static double counts_to_meters(int32_t counts, double wheel_radius);  // encoder → m
  static double meters_to_virtual(double v_mps, double wheel_radius);   // m/s → int8
  static int32_t angle_to_servo_bus(double angle_deg);  // deg → int32 LE
  static uint16_t angle_to_servo_pwm(double angle_deg, bool mode_270);  // deg → uint16 (0-270 for 270° mode)
  static int32_t angle_to_stepper_steps(double angle_deg);  // deg → step count

private:
  std::string serial_port_;
  uint32_t baud_;
  int fd_;  // -1 when closed
  SerialPort serial_;

  // Test injection: replaces the real read() call.
  std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> injection_;

  // Low-level POSIX helpers (delegated to serial_).
  void open_serial();
  void close_serial();
};

} // namespace vehicle_wbt_platform_cpp
