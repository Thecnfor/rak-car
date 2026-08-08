// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602Adapter — advanced MC602 device driver (RoboMaster-style HAL).
//
// Encapsulates EVERY direct operation of the MC602 controller behind clean,
// meaningful APIs — the full device inventory from controller_lab's
// DEVICE_DEFS. The driver owns protocol framing (mc602_protocol), unit
// conversions (mm→m, raw→volts, deg→servo byte) and the burst/priority
// scheduling hints. Business code (nodes/tasks) just calls these and gets
// typed results — feature development stays in the nodes.
//
// Layers:
//   nodes (business)          → MC602Adapter (driver: this)
//   MC602Adapter              → mc602::Device (frame builder)
//   MC602Adapter              → SerialTransport (Direct fd / Bridge service)
//
// All angle/speed scales are FREE (nothing hard-coded to a fixed range):
// the driver converts any input to the protocol domain; callers choose the
// numbers. Independent of ROS2 — testable in isolation.

#pragma once

#include "hardware/base_controller.hpp"
#include "hardware/mc602_protocol.hpp"
#include "hardware/serial_transport.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hardware
{

// C++17-safe π.
inline constexpr double MC602_PI = 3.14159265358979323846;

class MC602Adapter : public BaseController
{
public:
  // Direct transport over a local serial fd (tests, mock, non-bridged use).
  MC602Adapter(std::string serial_port, uint32_t baud);

  // Any transport — e.g. BridgeTransport via mc602_bridge.
  explicit MC602Adapter(std::shared_ptr<SerialTransport> transport);
  ~MC602Adapter() override;

  MC602Adapter(const MC602Adapter &) = delete;
  MC602Adapter & operator=(const MC602Adapter &) = delete;

  // --- BaseController (delegated to transport) ---
  void open() override;
  void close() override;
  bool is_open() const override;
  std::string serial_port() const override;
  uint32_t baud() const override;

  // --- Encoders (raw counts) ---
  std::array<int32_t, 4> read_encoder4();   // [FL, FR, RL, RR]
  int32_t read_encoder(uint8_t port);
  void reset_encoder4();                    // mode = reset
  void reset_encoder(uint8_t port);

  // --- Motors (virtual speed int8 [-100, 100]) ---
  void set_motor4(int8_t s1, int8_t s2, int8_t s3, int8_t s4);
  void set_motor(uint8_t port, int8_t speed);
  int8_t mps_to_virtual(double mps, double radius) const;  // m/s → virtual

  // --- Servos ---
  // set_servo_* take the RAW protocol angle (servo_pwm: 0..255 unsigned;
  // servo_bus: -128..127 signed). deg_to_* map degrees → raw with a FREE
  // scale (caller passes the degree range; nothing is hard-coded).
  void set_servo_pwm(uint8_t port, int raw_angle, uint8_t speed = 100);
  void set_servo_bus(uint8_t port, int raw_angle, int16_t speed = 100);
  static uint8_t deg_to_servo_pwm(double deg, double deg_min, double deg_max);
  static int8_t deg_to_servo_bus(double deg, double deg_min, double deg_max);

  // --- Stepper (velocity + position, raw int32) ---
  void set_stepper(uint8_t port, int32_t velocity, int32_t position);

  // --- Digital outputs (raw int8; e.g. relay/valve/pump) ---
  void set_dout(uint8_t port, int8_t value);

  // --- Sensors (dev 0x07; sub_mode selects type) ---
  uint16_t read_sensor_raw(uint8_t port, uint8_t sub_mode);  // raw H
  float read_ir(uint8_t port);            // meters (raw mm / 1000)
  float read_ultrasonic(uint8_t port);    // meters (raw /1000)
  int16_t read_analog(uint8_t port);      // raw H (0..65535)
  // Analog → volts, FREE scale: raw/analog_ref * analog_volts_max.
  float raw_to_volts(uint16_t raw, uint16_t raw_ref = 4096,
                     float volts_max = 3.3f) const;

  // --- Power (dev 0x0C) / analog-a (dev 0x08) — voltage readouts ---
  float read_power_voltage();             // battery voltage, volts
  uint16_t read_analog_a(uint8_t port);   // raw H

  // --- Misc devices ---
  void beep(int freq, float duration_s);
  std::vector<int64_t> read_board_key();
  std::vector<int64_t> read_bluetooth_pad();
  void set_led_light(uint8_t led_id, int r, int g, int b);
  void set_nixie(int value);                          // 数码管显示值
  void set_led_show(const std::string & text);        // 点阵屏文本(≤100 字符)

  // --- Test seam ---
  void set_injection(
    std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> responder);

  // --- Control-cycle packing ---
  void begin_burst();
  void commit_burst();

  // BaseController legacy (thin mappings onto the typed driver API).
  double read_sensor(uint8_t port_id, const std::string & sensor_type) override;
  void write_actuator(uint8_t port_id, const std::string & actuator_type,
                      double value) override;
  std::map<std::string, uint32_t> enumerate_ports() const override;

  // Mode + sensor constants.
  static constexpr uint8_t MODE_GET = 1;
  static constexpr uint8_t MODE_SET = 2;
  static constexpr uint8_t MODE_RESET = 3;
  static constexpr uint8_t SENSOR_ANALOG = 0;
  static constexpr uint8_t SENSOR_INFRARED = 1;
  static constexpr uint8_t SENSOR_TOUCH = 2;
  static constexpr uint8_t SENSOR_ULTRASONIC = 3;
  static constexpr uint8_t SENSOR_AMBIENT = 4;

private:
  // Frame + transaction core.
  std::vector<uint8_t> build_frame(const mc602::Device & dev, uint8_t mode,
                                   std::optional<uint8_t> port,
                                   std::vector<int64_t> args);
  std::vector<uint8_t> transact(const std::vector<uint8_t> & frame,
                                const ExchangeOpts & opts = {});
  void send_write(std::vector<uint8_t> frame, const ExchangeOpts & opts = {});
  std::vector<int64_t> query(const mc602::Device & dev, uint8_t mode,
                             std::optional<uint8_t> port,
                             std::vector<int64_t> args,
                             const std::string & share_key);

  std::shared_ptr<SerialTransport> transport_;
  bool opened_ = false;
  std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> injection_;
  bool burst_active_ = false;
  std::vector<std::vector<uint8_t>> burst_frames_;
};

}  // namespace hardware
