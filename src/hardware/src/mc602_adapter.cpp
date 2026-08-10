// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602Adapter — see header. Advanced MC602 device driver: full device
// inventory, meaningful units, FREE (parameterized) angle/voltage scales.

#include "hardware/mc602_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hardware
{

namespace
{

ExchangeOpts read_opts(const std::string & share_key)
{
  ExchangeOpts o;
  o.priority = ExchangeOpts::READ;
  o.share_key = share_key;
  return o;
}

ExchangeOpts write_opts(const std::string & coalesce_key, bool urgent = false)
{
  ExchangeOpts o;
  o.priority = urgent ? ExchangeOpts::URGENT : ExchangeOpts::NORMAL;
  o.coalesce_key = coalesce_key;
  return o;
}

}  // namespace

MC602Adapter::MC602Adapter(std::string serial_port, uint32_t baud)
: MC602Adapter(std::make_shared<DirectSerialTransport>(std::move(serial_port), baud))
{
  // Preserve legacy validation: MC602 supports 380400/1000000/115200.
  if (baud != 380400 && baud != 1000000 && baud != 115200) {
    throw std::runtime_error("MC602Adapter: unsupported baud " + std::to_string(baud) +
                             "; MC602 supports 380400/1000000/115200");
  }
}

MC602Adapter::MC602Adapter(std::shared_ptr<SerialTransport> transport)
: transport_(std::move(transport))
{
}

MC602Adapter::~MC602Adapter()
{
  close();
}

void MC602Adapter::open()
{
  if (!injection_) {
    transport_->open();
  }
  opened_ = true;
}

void MC602Adapter::close()
{
  try {
    transport_->close();
  } catch (...) {
    // close() must never throw.
  }
  opened_ = false;
}

bool MC602Adapter::is_open() const
{
  return opened_;
}

std::string MC602Adapter::serial_port() const
{
  return transport_->serial_port();
}

uint32_t MC602Adapter::baud() const
{
  return transport_->baud();
}

void MC602Adapter::set_injection(
  std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> responder)
{
  injection_ = std::move(responder);
}

// ---- Frame + transaction core ----

std::vector<uint8_t> MC602Adapter::build_frame(
  const mc602::Device & dev, uint8_t mode, std::optional<uint8_t> port,
  std::vector<int64_t> args)
{
  auto payload = dev.build_payload(mode, port, args);
  std::vector<uint8_t> frame;
  frame.reserve(3 + payload.size() + 1);
  frame.push_back(0x77);
  frame.push_back(0x68);
  frame.push_back(static_cast<uint8_t>(payload.size() + 4));  // len = payload+4
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(0x0A);
  return frame;
}

std::vector<uint8_t> MC602Adapter::transact(
  const std::vector<uint8_t> & frame, const ExchangeOpts & opts)
{
  if (injection_) {
    return injection_(frame);
  }
  auto response = transport_->exchange(frame, opts);

  // Validate: 77 68 <len> <payload> 0A with len == total frame size.
  if (response.size() < 6 || response[0] != 0x77 || response[1] != 0x68 ||
      response[2] != response.size() || response.back() != 0x0A) {
    throw std::runtime_error(
      "MC602 response invalid (" + std::to_string(response.size()) + " bytes)");
  }
  return std::vector<uint8_t>(response.begin() + 3, response.end() - 1);
}

void MC602Adapter::send_write(std::vector<uint8_t> frame, const ExchangeOpts & opts)
{
  if (burst_active_) {
    burst_frames_.push_back(std::move(frame));
    return;
  }
  if (injection_) {
    injection_(frame);
    return;
  }
  transact(frame, opts);  // response ignored for writes
}

std::vector<int64_t> MC602Adapter::query(
  const mc602::Device & dev, uint8_t mode, std::optional<uint8_t> port,
  std::vector<int64_t> args, const std::string & share_key)
{
  auto payload = transact(build_frame(dev, mode, port, std::move(args)),
                          read_opts(share_key));
  return dev.parse_payload(payload);
}

// ---- Encoders ----

std::array<int32_t, 4> MC602Adapter::read_encoder4()
{
  auto vals = query(mc602::encoder4(), MODE_GET, std::nullopt, {}, "encoder4");
  std::array<int32_t, 4> out{};
  for (int i = 0; i < 4; ++i) {
    out[i] = (2 + i < vals.size()) ? static_cast<int32_t>(vals[2 + i]) : 0;
  }
  return out;
}

int32_t MC602Adapter::read_encoder(uint8_t port)
{
  auto vals = query(mc602::encoder(), MODE_GET, port, {},
                    "encoder:" + std::to_string(port));
  return (vals.size() >= 4) ? static_cast<int32_t>(vals[3]) : 0;
}

void MC602Adapter::reset_encoder4()
{
  send_write(build_frame(mc602::encoder4(), MODE_RESET, std::nullopt, {}),
             write_opts("encoder4"));
}

void MC602Adapter::reset_encoder(uint8_t port)
{
  send_write(build_frame(mc602::encoder(), MODE_RESET, port, {}),
             write_opts("encoder:" + std::to_string(port)));
}

// ---- Motors ----

void MC602Adapter::set_motor4(int8_t s1, int8_t s2, int8_t s3, int8_t s4)
{
  const bool stop = (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0);
  send_write(build_frame(mc602::motor4(), MODE_SET, std::nullopt, {s1, s2, s3, s4}),
             write_opts("motor4", /*urgent=*/stop));
}

void MC602Adapter::set_motor(uint8_t port, int8_t speed)
{
  send_write(build_frame(mc602::motor(), MODE_SET, port, {speed}),
             write_opts("motor:" + std::to_string(port)));
}

int8_t MC602Adapter::mps_to_virtual(double mps, double radius) const
{
  // ω = v/r ; virtual = ω × (counts_per_rev / (2π × 100)) ; clamp [-100,100].
  const double omega = mps / radius;
  const double virt = omega * (2015.13 / (2.0 * MC602_PI * 100.0));
  return static_cast<int8_t>(std::clamp<int16_t>(
    static_cast<int16_t>(std::round(virt)), -100, 100));
}

// ---- Servos ----

void MC602Adapter::set_servo_pwm(uint8_t port, int raw_angle, uint8_t speed)
{
  send_write(build_frame(mc602::servo_pwm(), MODE_SET, port, {raw_angle, speed}),
             write_opts("servo_pwm:" + std::to_string(port)));
}

void MC602Adapter::set_servo_bus(uint8_t port, int raw_angle, int16_t speed)
{
  send_write(build_frame(mc602::servo_bus(), MODE_SET, port, {raw_angle, speed}),
             write_opts("servo_bus:" + std::to_string(port)));
}

uint8_t MC602Adapter::deg_to_servo_pwm(double deg, double deg_min, double deg_max)
{
  const double t = (deg - deg_min) / (deg_max - deg_min);
  return static_cast<uint8_t>(std::clamp(
    static_cast<int>(std::round(t * 255.0)), 0, 255));
}

int8_t MC602Adapter::deg_to_servo_bus(double deg, double deg_min, double deg_max)
{
  const double t = (deg - deg_min) / (deg_max - deg_min);
  return static_cast<int8_t>(std::clamp(
    static_cast<int>(std::round(t * 255.0 - 128.0)), -128, 127));
}

// ---- Stepper ----

void MC602Adapter::set_stepper(uint8_t port, int32_t velocity, int32_t position)
{
  send_write(build_frame(mc602::stepper(), MODE_SET, port, {velocity, position}),
             write_opts("stepper:" + std::to_string(port)));
}

// ---- Digital out ----

void MC602Adapter::set_dout(uint8_t port, int8_t value)
{
  send_write(build_frame(mc602::dout(), MODE_SET, port, {value}),
             write_opts("dout:" + std::to_string(port)));
}

// ---- Sensors ----

uint16_t MC602Adapter::read_sensor_raw(uint8_t port, uint8_t sub_mode)
{
  if (port < 1 || port > 16) {
    throw std::runtime_error("read_sensor_raw: port " + std::to_string(port) +
                             " out of range [1,16]");
  }
  auto vals = query(mc602::sensor(sub_mode), sub_mode, port, {},
                    "sensor:" + std::to_string(sub_mode) + ":" + std::to_string(port));
  return (vals.size() >= 4) ? static_cast<uint16_t>(vals[3]) : 0;
}

float MC602Adapter::read_ir(uint8_t port)
{
  // Infrared reports mm; return meters.
  return static_cast<float>(read_sensor_raw(port, SENSOR_INFRARED)) / 1000.0f;
}

float MC602Adapter::read_ultrasonic(uint8_t port)
{
  // Ultrasonic reports mm; return meters.
  return static_cast<float>(read_sensor_raw(port, SENSOR_ULTRASONIC)) / 1000.0f;
}

int16_t MC602Adapter::read_analog(uint8_t port)
{
  return static_cast<int16_t>(read_sensor_raw(port, SENSOR_ANALOG));
}

float MC602Adapter::raw_to_volts(uint16_t raw, uint16_t raw_ref, float volts_max) const
{
  return (raw_ref == 0) ? 0.0f : volts_max * static_cast<float>(raw) / raw_ref;
}

// ---- Power / analog-a ----

float MC602Adapter::read_power_voltage()
{
  // dev 0x0C "bi": mode + int32 — battery voltage in some unit; caller scales.
  auto vals = query(mc602::power(), MODE_GET, std::nullopt, {}, "power");
  return (vals.size() >= 3) ? static_cast<float>(vals[2]) : 0.0f;
}

uint16_t MC602Adapter::read_analog_a(uint8_t port)
{
  // dev 0x08 "bbH": mode + port + H (raw analog value, FREE scale).
  auto payload = transact(
    build_frame(mc602::analog_a(), MODE_GET, port, {}),
    read_opts("analog_a:" + std::to_string(port)));
  auto vals = mc602::analog_a().parse_payload(payload);
  return (vals.size() >= 4) ? static_cast<uint16_t>(vals[3]) : 0;
}

// ---- Misc ----

void MC602Adapter::beep(int freq, float duration_s)
{
  send_write(build_frame(mc602::beep(), MODE_SET, std::nullopt,
                         {freq / 2, static_cast<int>(duration_s * 20.0f)}),
             write_opts("beep"));
}

std::vector<int64_t> MC602Adapter::read_board_key()
{
  return query(mc602::board_key(), MODE_GET, std::nullopt, {}, "board_key");
}

std::vector<int64_t> MC602Adapter::read_bluetooth_pad()
{
  return query(mc602::bluetooth(), MODE_GET, std::nullopt, {}, "bluetooth");
}

void MC602Adapter::set_led_light(uint8_t led_id, int r, int g, int b)
{
  send_write(build_frame(mc602::led_light(), MODE_SET, std::nullopt,
                         {led_id, r, g, b}),
             write_opts("led_light"));
}

void MC602Adapter::set_nixie(int value)
{
  send_write(build_frame(mc602::nixietube(), MODE_SET, std::nullopt, {value}),
             write_opts("nixietube"));
}

void MC602Adapter::set_led_show(const std::string & text)
{
  // 点阵屏:文本转 ASCII,截断/补齐到 100 字符。
  std::vector<int64_t> args;
  args.reserve(100);
  for (size_t i = 0; i < text.size() && args.size() < 100; ++i) {
    args.push_back(static_cast<unsigned char>(text[i]));
  }
  send_write(build_frame(mc602::led_show(), MODE_SET, std::nullopt, std::move(args)),
             write_opts("led_show"));
}

void MC602Adapter::write_actuator(uint8_t port_id, const std::string & actuator_type,
                                  double value)
{
  if (actuator_type == "motor") { set_motor(port_id, static_cast<int8_t>(value)); return; }
  if (actuator_type == "servo_pwm") { set_servo_pwm(port_id, static_cast<int>(value)); return; }
  if (actuator_type == "servo_bus") { set_servo_bus(port_id, static_cast<int>(value)); return; }
  if (actuator_type == "stepper") { set_stepper(port_id, static_cast<int32_t>(value), 0); return; }
  if (actuator_type == "dout") { set_dout(port_id, static_cast<int8_t>(value)); return; }
  throw std::runtime_error("write_actuator: unsupported type '" + actuator_type + "'");
}

// ---- Control-cycle packing ----

void MC602Adapter::begin_burst()
{
  if (burst_active_) {
    throw std::runtime_error("MC602Adapter::begin_burst: burst already active");
  }
  burst_active_ = true;
  burst_frames_.clear();
}

void MC602Adapter::commit_burst()
{
  if (!burst_active_) {
    return;  // idempotent, no-op
  }
  burst_active_ = false;
  if (burst_frames_.empty()) {
    return;
  }
  auto frames = std::move(burst_frames_);
  burst_frames_.clear();

  if (injection_) {
    for (const auto & f : frames) {
      injection_(f);
    }
    return;
  }
  transport_->exchange_burst(frames, ExchangeOpts{});
}

}  // namespace hardware
