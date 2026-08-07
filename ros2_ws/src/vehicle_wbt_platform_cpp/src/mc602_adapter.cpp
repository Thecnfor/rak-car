// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602Adapter — real MC602 binary protocol implementation.
//
// Frame format (docs/hardware-comm.md §MC602):
//   Request:  0x77 0x68 | length(len+4) | dev_id mode port params... | 0x0A
//   Response: 0x77 0x68 | length(total)  | dev_id mode port data...  | 0x0A
//   Payload = res[3:-1] (skip header[0:2], length[2], footer[last])
//
// Device IDs: motor4=0x01, motor=0x02, encoder4=0x03, encoder=0x04,
//             servo_pwm=0x05, servo_bus=0x06, sensor_multi=0x07,
//             dout=0x10, stepper=0x11
//
// Wheel order: M2=FL, M1=FR, M3=RL, M4=RR (from hardware-port-mapping.md)

#include "vehicle_wbt_platform_cpp/mc602_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <arpa/inet.h>  // htonl/ntohl for endian conversion

namespace vehicle_wbt_platform_cpp
{

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
  // MC602-specific port limits (override BaseController generic values).
  if (actuator_type == "motor") return MC602Adapter::MC602_MOTOR_PORTS;
  if (actuator_type == "servo_bus" || actuator_type == "servo_pwm") return MC602Adapter::MC602_SERVO_PORTS;
  if (actuator_type == "stepper") return MC602Adapter::MC602_STEPPER_PORTS;
  if (actuator_type == "dout") return MC602Adapter::MC602_IO_PORTS;
  return 0;
}

} // anonymous namespace

// ---- Constants from the header ----
constexpr uint8_t MC602Adapter::DEV_MOTOR4;
constexpr uint8_t MC602Adapter::DEV_MOTOR;
constexpr uint8_t MC602Adapter::DEV_ENCODER4;
constexpr uint8_t MC602Adapter::DEV_ENCODER;
constexpr uint8_t MC602Adapter::DEV_SERVO_PWM;
constexpr uint8_t MC602Adapter::DEV_SERVO_BUS;
constexpr uint8_t MC602Adapter::DEV_SENSOR_MULTI;
constexpr uint8_t MC602Adapter::DEV_DOUT;
constexpr uint8_t MC602Adapter::DEV_STEPPER;
constexpr uint8_t MC602Adapter::MODE_GET;
constexpr uint8_t MC602Adapter::MODE_SET;
constexpr uint8_t MC602Adapter::MODE_RESET;
constexpr uint8_t MC602Adapter::SENSOR_ANALOG;
constexpr uint8_t MC602Adapter::SENSOR_INFRARED;
constexpr uint8_t MC602Adapter::SENSOR_TOUCH;
constexpr uint8_t MC602Adapter::SENSOR_ULTRASONIC;
constexpr uint8_t MC602Adapter::SENSOR_AMBIENT;
constexpr double MC602Adapter::ENCODER_COUNTS_PER_REV;
constexpr double MC602Adapter::ENCODER_2_RAD;
constexpr double MC602Adapter::RAD_2_VIRTUAL;
constexpr double MC602Adapter::STEPPER_RAD_PER_STEP;
constexpr uint8_t MC602Adapter::FRAME_HEADER_0;
constexpr uint8_t MC602Adapter::FRAME_HEADER_1;
constexpr uint8_t MC602Adapter::FRAME_FOOTER;

MC602Adapter::MC602Adapter(std::string serial_port, uint32_t baud)
: serial_port_(std::move(serial_port))
, baud_(baud)
, fd_(-1)
, serial_(serial_port_, baud)
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
  // When injection is active (test mode), skip real serial port open.
  if (injection_) {
    fd_ = 42;  // sentinel: satisfies is_open() without real hardware
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

void MC602Adapter::set_injection(
  std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)> responder)
{
  injection_ = std::move(responder);
}

// ---- BaseController interface ----

double MC602Adapter::read_sensor(uint8_t port_id, const std::string & sensor_type)
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }

  // Map sensor_type → (dev_id, mode, sub_mode, payload_size, parse_fn)
  uint8_t dev_id = 0, mode = MODE_GET, sub_mode = 0;
  size_t response_payload_size = 0;

  if (sensor_type == "ir") {
    dev_id = DEV_SENSOR_MULTI;
    sub_mode = SENSOR_INFRARED;
    response_payload_size = 4;  // dev_id(1) + mode(1) + port(1) + uint16_data(2)
  } else if (sensor_type == "analog_input") {
    dev_id = DEV_SENSOR_MULTI;
    sub_mode = SENSOR_ANALOG;
    response_payload_size = 4;
  } else if (sensor_type == "ultrasonic") {
    dev_id = DEV_SENSOR_MULTI;
    sub_mode = SENSOR_ULTRASONIC;
    response_payload_size = 5;  // includes float32
  } else if (sensor_type == "touch") {
    dev_id = DEV_SENSOR_MULTI;
    sub_mode = SENSOR_TOUCH;
    response_payload_size = 3;
  } else if (sensor_type == "ambient_light") {
    dev_id = DEV_SENSOR_MULTI;
    sub_mode = SENSOR_AMBIENT;
    response_payload_size = 4;
  } else if (sensor_type == "encoder") {
    dev_id = DEV_ENCODER;
    response_payload_size = 5;  // dev_id + mode + port + int32
  } else {
    throw std::runtime_error("unsupported sensor type '" + sensor_type + "'");
  }

  if (port_id < 1 || port_id > IO_MAX) {
    throw std::runtime_error(
      "port_id " + std::to_string(port_id) +
      " out of range for sensor; must be in [1, " + std::to_string(IO_MAX) + "]");
  }

  // Build frame: 0x77 0x68 | len | dev_id mode port | 0x0A
  std::vector<uint8_t> params = {sub_mode};
  auto frame = build_get(dev_id, port_id, params);

  // Inject test bypass. The injected response is the same payload that
  // exchange() would return after stripping header+len+footer, so we parse
  // it identically to the non-injection path below.
  if (injection_) {
    auto payload = injection_(frame);
    if (sensor_type == "ir" || sensor_type == "analog_input") {
      if (payload.size() < 5) {
        throw std::runtime_error("injection: sensor response too short");
      }
      uint16_t raw;
      std::memcpy(&raw, payload.data() + 3, sizeof(raw));
      raw = le16toh(raw);
      return static_cast<double>(raw);
    }
    if (sensor_type == "ultrasonic") {
      if (payload.size() < 7) {
        throw std::runtime_error("injection: ultrasonic response too short");
      }
      float val;
      std::memcpy(&val, payload.data() + 3, sizeof(val));
      return static_cast<double>(val);
    }
    if (sensor_type == "touch") {
      if (payload.size() < 4) {
        throw std::runtime_error("injection: touch response too short");
      }
      return static_cast<double>(payload[3]);
    }
    if (sensor_type == "encoder") {
      if (payload.size() < 7) {
        throw std::runtime_error("injection: encoder response too short");
      }
      int32_t counts;
      std::memcpy(&counts, payload.data() + 3, sizeof(counts));
      counts = static_cast<int32_t>(le32toh(static_cast<uint32_t>(counts)));
      return counts * ENCODER_2_RAD * 0.03;
    }
    // Default: return raw uint16
    if (payload.size() < 5) {
      throw std::runtime_error("injection: sensor response too short");
    }
    uint16_t raw;
    std::memcpy(&raw, payload.data() + 3, sizeof(raw));
    raw = le16toh(raw);
    return static_cast<double>(raw);
  }

  auto payload = exchange(frame);

  // Parse payload based on sensor type.
  // Payload layout: dev_id(1) mode(1) port(1) data(N)
  if (payload.size() < 3) {
    throw std::runtime_error("MC602 response too short for sensor read");
  }

  if (sensor_type == "ir" || sensor_type == "analog_input") {
    // uint16 LE → raw value
    if (payload.size() < 5) {
      throw std::runtime_error("MC602 IR response too short");
    }
    uint16_t raw;
    std::memcpy(&raw, payload.data() + 3, sizeof(raw));
    raw = le16toh(raw);
    return static_cast<double>(raw);
  }

  if (sensor_type == "ultrasonic") {
    if (payload.size() < 7) {
      throw std::runtime_error("MC602 ultrasonic response too short");
    }
    float val;
    std::memcpy(&val, payload.data() + 3, sizeof(val));
    return static_cast<double>(val);
  }

  if (sensor_type == "touch") {
    if (payload.size() < 4) {
      throw std::runtime_error("MC602 touch response too short");
    }
    return static_cast<double>(payload[3]);
  }

  if (sensor_type == "encoder") {
    if (payload.size() < 7) {
      throw std::runtime_error("MC602 encoder response too short");
    }
    int32_t counts;
    std::memcpy(&counts, payload.data() + 3, sizeof(counts));
    counts = static_cast<int32_t>(le32toh(static_cast<uint32_t>(counts)));
    // Return in meters (convert via wheel_radius=0.03m — caller should adjust)
    return counts * ENCODER_2_RAD * 0.03;
  }

  // Default: return raw uint16
  if (payload.size() < 5) {
    throw std::runtime_error("MC602 sensor response too short for uint16 parse");
  }
  uint16_t raw;
  std::memcpy(&raw, payload.data() + 3, sizeof(raw));
  raw = le16toh(raw);
  return static_cast<double>(raw);
}

void MC602Adapter::write_actuator(uint8_t port_id, const std::string & actuator_type,
                                   double value)
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }

  if (actuator_type_to_method().find(actuator_type) == actuator_type_to_method().end()) {
    throw std::runtime_error("unsupported actuator type '" + actuator_type + "'");
  }

  const uint8_t max_ports = max_ports_for_actuator(actuator_type);
  if (port_id < 1 || port_id > max_ports) {
    throw std::runtime_error(
      "port_id " + std::to_string(port_id) +
      " out of range for actuator type '" + actuator_type +
      "'; must be in [1, " + std::to_string(max_ports) + "]");
  }

  if (!std::isfinite(value)) {
    throw std::runtime_error("value must be a finite number");
  }

  // Dispatch by actuator type.
  if (actuator_type == "motor") {
    // Single motor: dev_id=MOTOR, mode=SET, port, int8 speed
    int8_t v = clamp_virtual(value);
    std::vector<uint8_t> params = {static_cast<uint8_t>(static_cast<int>(v) & 0xFF)};
    auto frame = build_set(DEV_MOTOR, port_id, params);
    if (injection_) {
      injection_(frame);
      return;
    }
    exchange(frame);  // ignore response for SET commands

  } else if (actuator_type == "servo_bus") {
    // Bus servo: dev_id=SERVO_BUS, mode=SET, port, int32 angle(LE), speed(1)
    int32_t angle_le = angle_to_servo_bus(value);
    std::vector<uint8_t> params(5);
    std::memcpy(params.data(), &angle_le, sizeof(angle_le));
    params[4] = 100;  // speed
    auto frame = build_set(DEV_SERVO_BUS, port_id, params);
    if (injection_) {
      injection_(frame);
      return;
    }
    exchange(frame);

  } else if (actuator_type == "servo_pwm") {
    // PWM servo: dev_id=SERVO_PWM, mode=SET, port, speed(1), angle(uint8)
    uint16_t angle = angle_to_servo_pwm(value, port_id == 7);  // S7 is 270° mode
    std::vector<uint8_t> params = {100, static_cast<uint8_t>(angle)};  // speed=100, angle
    auto frame = build_set(DEV_SERVO_PWM, port_id, params);
    if (injection_) {
      injection_(frame);
      return;
    }
    exchange(frame);

  } else if (actuator_type == "stepper") {
    // Stepper: dev_id=STEPPER, mode=SET, port, int32 steps(LE), speed(1)
    int32_t steps = angle_to_stepper_steps(value);
    steps = htole32(steps);
    std::vector<uint8_t> params(5);
    std::memcpy(params.data(), &steps, sizeof(steps));
    params[4] = 50;  // speed
    auto frame = build_set(DEV_STEPPER, port_id, params);
    if (injection_) {
      injection_(frame);
      return;
    }
    exchange(frame);

  } else if (actuator_type == "dout") {
    // Digital output: dev_id=DOUT, mode=SET, port, value(1)
    uint8_t val = (value != 0.0) ? 2 : 1;  // 1=disconnect, 2=connect
    std::vector<uint8_t> params = {val};
    auto frame = build_set(DEV_DOUT, port_id, params);
    if (injection_) {
      injection_(frame);
      return;
    }
    exchange(frame);
  }
}

std::map<std::string, uint32_t> MC602Adapter::enumerate_ports() const
{
  return {{"motor", MC602_MOTOR_PORTS}, {"servo", MC602_SERVO_PORTS},
          {"stepper", MC602_STEPPER_PORTS}, {"io", MC602_IO_PORTS}};
}

// ---- Extended methods ----

std::array<int32_t, 4> MC602Adapter::read_encoder4()
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }

  // dev_id=ENCODER4(0x03), mode=GET, no port → reads all 4
  auto frame = build_get(DEV_ENCODER4, 0, {});

  if (injection_) {
    auto resp = injection_(frame);
    // Payload layout from exchange(): dev_id(1) mode(1) port(1) + 4× int32
    if (resp.size() < 3 + 4 * 4) {
      throw std::runtime_error("injection: encoder4 response too short");
    }
    std::array<int32_t, 4> result;
    for (int i = 0; i < 4; ++i) {
      uint32_t u;
      std::memcpy(&u, resp.data() + 3 + i * 4, sizeof(u));
      result[i] = static_cast<int32_t>(ntohl(u));
    }
    return result;
  }

  auto payload = exchange(frame);
  // Payload: dev_id(1) mode(1) port(1) + 4× int32(LE)
  if (payload.size() < 3 + 4 * 4) {
    throw std::runtime_error("MC602 encoder4 response too short: " + std::to_string(payload.size()));
  }

  std::array<int32_t, 4> result;
  for (int i = 0; i < 4; ++i) {
    uint32_t u;
    std::memcpy(&u, payload.data() + 3 + i * 4, sizeof(u));
    result[i] = static_cast<int32_t>(ntohl(u));
  }
  return result;
}

float MC602Adapter::read_infrared(uint8_t port_id)
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }

  auto frame = build_get(DEV_SENSOR_MULTI, port_id, {SENSOR_INFRARED});

  if (injection_) {
    auto resp = injection_(frame);
    if (resp.size() < 4) {
      throw std::runtime_error("injection: IR response too short");
    }
    uint16_t raw;
    std::memcpy(&raw, resp.data() + 3, sizeof(raw));
    return static_cast<float>(le16toh(raw)) / 1000.0f;  // mm → m
  }

  auto payload = exchange(frame);
  // Payload: dev_id(1) mode(1) port(1) + uint16(mm)
  if (payload.size() < 5) {
    throw std::runtime_error("MC602 IR response too short");
  }

  uint16_t raw;
  std::memcpy(&raw, payload.data() + 3, sizeof(raw));
  raw = le16toh(raw);
  return static_cast<float>(raw) / 1000.0f;  // raw mm → meters
}

void MC602Adapter::write_motor4(int8_t v_fl, int8_t v_fr, int8_t v_rl, int8_t v_rr)
{
  if (!is_open()) {
    throw std::runtime_error("MC602Adapter not open; call open() first");
  }

  // dev_id=MOTOR4(0x01), mode=SET, port=0, 4× int8 speeds
  // Wheel order: M2(FL)=v_fl, M1(FR)=v_fr, M3(RL)=v_rl, M4(RR)=v_rr
  std::vector<uint8_t> params = {
    static_cast<uint8_t>(static_cast<int>(v_fl) & 0xFF),
    static_cast<uint8_t>(static_cast<int>(v_fr) & 0xFF),
    static_cast<uint8_t>(static_cast<int>(v_rl) & 0xFF),
    static_cast<uint8_t>(static_cast<int>(v_rr) & 0xFF),
  };

  auto frame = build_set(DEV_MOTOR4, 0, params);

  if (injection_) {
    injection_(frame);
    return;
  }

  exchange(frame);
}

// ---- Protocol helpers ----

std::vector<uint8_t> MC602Adapter::build_get(uint8_t dev_id, uint8_t port,
                                              std::vector<uint8_t> params)
{
  // Header(2) + length(1) + dev_id(1) + mode(1) + port(1) + params(N) + footer(1)
  std::vector<uint8_t> frame;
  frame.reserve(6 + params.size());

  frame.push_back(FRAME_HEADER_0);
  frame.push_back(FRAME_HEADER_1);

  // Length = 4 (dev_id + mode + port + footer) + params.size()
  uint8_t len = static_cast<uint8_t>(4 + static_cast<int>(params.size()));
  frame.push_back(len);

  frame.push_back(dev_id);
  frame.push_back(MODE_GET);
  frame.push_back(port);

  for (uint8_t b : params) {
    frame.push_back(b);
  }

  frame.push_back(FRAME_FOOTER);

  return frame;
}

std::vector<uint8_t> MC602Adapter::build_set(uint8_t dev_id, uint8_t port,
                                              std::vector<uint8_t> params)
{
  std::vector<uint8_t> frame;
  frame.reserve(6 + params.size());

  frame.push_back(FRAME_HEADER_0);
  frame.push_back(FRAME_HEADER_1);

  uint8_t len = static_cast<uint8_t>(4 + static_cast<int>(params.size()));
  frame.push_back(len);

  frame.push_back(dev_id);
  frame.push_back(MODE_SET);
  frame.push_back(port);

  for (uint8_t b : params) {
    frame.push_back(b);
  }

  frame.push_back(FRAME_FOOTER);

  return frame;
}

std::vector<uint8_t> MC602Adapter::exchange(const std::vector<uint8_t> & frame)
{
  auto response = serial_.exchange(frame);

  // Validate frame structure.
  if (response.size() < 6) {
    throw std::runtime_error(
      "MC602 response too short: " + std::to_string(response.size()) + " bytes");
  }

  if (response[0] != FRAME_HEADER_0 || response[1] != FRAME_HEADER_1) {
    throw std::runtime_error("MC602 response has bad header");
  }

  // Length byte = total frame size - 2 (header) - 1 (length) - 1 (footer)
  // = payload + 2 (dev_id + mode) — verify.
  uint8_t expected_len = response[2];
  if (static_cast<int>(expected_len) + 2 != static_cast<int>(response.size())) {
    throw std::runtime_error(
      "MC602 response length mismatch: field=" + std::to_string(expected_len) +
      " actual=" + std::to_string(response.size()));
  }

  if (response.back() != FRAME_FOOTER) {
    throw std::runtime_error("MC602 response missing footer");
  }

  // Return payload: skip header(2) + length(1), keep everything except footer.
  return std::vector<uint8_t>(response.begin() + 3, response.end() - 1);
}

// ---- Unit conversion ----

int8_t MC602Adapter::clamp_virtual(double v_mps, double wheel_radius)
{
  // Convert m/s → virtual int8 [-100, 100].
  // v_mps → ω = v/r → virtual = ω / (2π×100/2015.13) = ω × 3.207
  const double omega = v_mps / wheel_radius;
  const double virt = omega * RAD_2_VIRTUAL;
  const int16_t clamped = static_cast<int16_t>(std::round(virt));
  return static_cast<int8_t>(std::clamp<int16_t>(clamped, -100, 100));
}

double MC602Adapter::counts_to_meters(int32_t counts, double wheel_radius)
{
  return static_cast<double>(counts) * ENCODER_2_RAD * wheel_radius;
}

double MC602Adapter::meters_to_virtual(double v_mps, double wheel_radius)
{
  // v_mps → ω = v/r → virtual = ω × 3.207
  const double omega = v_mps / wheel_radius;
  return omega * RAD_2_VIRTUAL;
}

int32_t MC602Adapter::angle_to_servo_bus(double angle_deg)
{
  // Bus servo: angle → int32 LE. Range: ±150° per arm ±150 limit decision.
  return static_cast<int32_t>(std::round(angle_deg * 100));  // 0.01° resolution
}

uint16_t MC602Adapter::angle_to_servo_pwm(double angle_deg, bool mode_270)
{
  // PWM servo: 0-180 for 180° mode, 0-270 for 270° mode.
  // Map angle to [0, 180] or [0, 270] uint8/uint16.
  double clamped;
  if (mode_270) {
    clamped = std::clamp(angle_deg + 135.0, 0.0, 270.0);  // center at 135°
  } else {
    clamped = std::clamp(angle_deg + 90.0, 0.0, 180.0);   // center at 90°
  }
  return static_cast<uint16_t>(std::round(clamped));
}

int32_t MC602Adapter::angle_to_stepper_steps(double angle_deg)
{
  // Stepper: 1.8° per step, 16细分 → 0.001963 rad/step
  // angle_deg → radians → steps
  double rad = angle_deg * MC602_PI / 180.0;
  return static_cast<int32_t>(std::round(rad / STEPPER_RAD_PER_STEP));
}

// ---- Low-level POSIX ----

void MC602Adapter::open_serial()
{
  serial_.set_response_handler(injection_);  // no-op if injection_ is empty
  serial_.open();
  fd_ = 1;  // mark as open (actual fd is inside SerialPort)
}

void MC602Adapter::close_serial()
{
  serial_.close();
}

} // namespace vehicle_wbt_platform_cpp
