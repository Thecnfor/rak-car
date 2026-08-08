// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// mc602_protocol — see header. Faithful port of controller_lab's
// StructData + DeviceCommand payload building/parsing.

#include "hardware/mc602_protocol.hpp"

#include <cstddef>
#include <string>
#include <utility>

namespace hardware
{
namespace mc602
{

std::vector<uint8_t> Device::build_payload(
  std::optional<uint8_t> mode, std::optional<uint8_t> opt_port,
  const std::vector<int64_t> & args) const
{
  // values = [dev_id, mode, (port), args...]; total must equal slot_count()
  // (the SDK packs exactly `len("<b"+fmt)` values, dev byte included).
  std::vector<int64_t> values;
  values.reserve(slot_count());
  values.push_back(static_cast<int64_t>(dev_id_));
  values.push_back(static_cast<int64_t>(mode.value_or(0)));
  if (opt_port.has_value()) {
    values.push_back(static_cast<int64_t>(*opt_port));
  }

  // Trim extra args from the front (SDK: pop(0) while too many).
  const size_t expected = slot_count() - values.size();
  size_t start = (args.size() > expected) ? (args.size() - expected) : 0;
  for (size_t j = start; j < args.size() && values.size() < slot_count(); ++j) {
    values.push_back(args[j]);
  }
  while (values.size() < slot_count()) {
    values.push_back(0);  // zero-pad (SDK: append(0))
  }

  // Pack little-endian per slot type.
  std::vector<uint8_t> out;
  out.reserve(1 + slots_.size() * 4);
  auto put = [&out](int64_t v, Slot s) {
    switch (s) {
      case Slot::I8:
        out.push_back(static_cast<uint8_t>(static_cast<int8_t>(v) & 0xFF));
        break;
      case Slot::U8:
        out.push_back(static_cast<uint8_t>(v));
        break;
      case Slot::I16: {
        const int16_t x = static_cast<int16_t>(v);
        out.push_back(static_cast<uint8_t>(x & 0xFF));
        out.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
        break;
      }
      case Slot::U16: {
        const uint16_t x = static_cast<uint16_t>(v);
        out.push_back(static_cast<uint8_t>(x & 0xFF));
        out.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
        break;
      }
      case Slot::I32: {
        const int32_t x = static_cast<int32_t>(v);
        for (int k = 0; k < 4; ++k) {
          out.push_back(static_cast<uint8_t>((x >> (8 * k)) & 0xFF));
        }
        break;
      }
    }
  };

  put(values[0], Slot::I8);  // dev_id
  for (size_t i = 1; i < values.size(); ++i) {
    put(values[i], slots_[i - 1]);
  }
  return out;
}

std::vector<int64_t> Device::parse_payload(const std::vector<uint8_t> & payload) const
{
  std::vector<int64_t> out;
  size_t off = 0;
  if (off < payload.size()) {
    out.push_back(static_cast<int8_t>(payload[off++]));
  }
  for (Slot s : slots_) {
    switch (s) {
      case Slot::I8:
        if (off < payload.size()) {
          out.push_back(static_cast<int8_t>(payload[off++]));
        }
        break;
      case Slot::U8:
        if (off < payload.size()) {
          out.push_back(payload[off++]);
        }
        break;
      case Slot::I16:
        if (off + 2 <= payload.size()) {
          const int16_t x = static_cast<int16_t>(
            static_cast<uint16_t>(payload[off]) |
            (static_cast<uint16_t>(payload[off + 1]) << 8));
          off += 2;
          out.push_back(x);
        }
        break;
      case Slot::U16:
        if (off + 2 <= payload.size()) {
          const uint16_t x = static_cast<uint16_t>(payload[off]) |
                             (static_cast<uint16_t>(payload[off + 1]) << 8);
          off += 2;
          out.push_back(x);
        }
        break;
      case Slot::I32:
        if (off + 4 <= payload.size()) {
          int32_t x = 0;
          for (int k = 0; k < 4; ++k) {
            x |= static_cast<int32_t>(payload[off + k]) << (8 * k);
          }
          off += 4;
          out.push_back(x);
        }
        break;
    }
  }
  return out;
}

namespace
{

// Build a Device from a format string (mirrors DEVICE_DEFS).
Device make_device(uint8_t dev_id, const char * fmt)
{
  std::vector<Slot> slots;
  for (const char * p = fmt; *p; ++p) {
    switch (*p) {
      case 'b': slots.push_back(Slot::I8); break;
      case 'B': slots.push_back(Slot::U8); break;
      case 'h': slots.push_back(Slot::I16); break;
      case 'H': slots.push_back(Slot::U16); break;
      case 'i': slots.push_back(Slot::I32); break;
      default: break;  // ignore unknown (shouldn't happen in our table)
    }
  }
  return Device(dev_id, std::move(slots));
}

}  // namespace

const Device & motor4()
{
  static const Device d = make_device(0x01, "bbbbb");  // mode + 4×int8, no port
  return d;
}

const Device & motor()
{
  static const Device d = make_device(0x02, "bbb");  // mode + port + int8 speed
  return d;
}

const Device & encoder4()
{
  static const Device d = make_device(0x03, "biiii");  // mode + 4×int32, no port
  return d;
}

const Device & encoder()
{
  static const Device d = make_device(0x04, "bbi");  // mode + port + int32
  return d;
}

const Device & servo_pwm()
{
  static const Device d = make_device(0x05, "bbBB");  // mode + port + angle(u8) + speed(u8)
  return d;
}

const Device & servo_bus()
{
  static const Device d = make_device(0x06, "bbbbh");  // mode + port + angle(i8) + speed(i16)
  return d;
}

const Device & sensor(uint8_t sub_mode)
{
  // 0x07 "bbH" — mode(=sub_mode selects analog/ir/touch/ultrasonic/ambient) + port + H
  static const Device d = make_device(0x07, "bbH");
  (void)sub_mode;  // sub_mode maps to the mode byte at call site
  return d;
}

const Device & analog_a()
{
  static const Device d = make_device(0x08, "bbH");
  return d;
}

const Device & power()
{
  static const Device d = make_device(0x0C, "bi");
  return d;
}

const Device & beep()
{
  static const Device d = make_device(0x0A, "BBB");
  return d;
}

const Device & board_key()
{
  static const Device d = make_device(0x0D, "bbb");
  return d;
}

const Device & bluetooth()
{
  static const Device d = make_device(0x09, "BBBBi");
  return d;
}

const Device & led_light()
{
  static const Device d = make_device(0x0E, "bbBBBB");
  return d;
}

const Device & led_show()
{
  // 0x0B "b"×101 — display buffer (make_device copies the slots immediately,
  // so the temporary format string is safe).
  static const Device d = make_device(0x0B, std::string(101, 'b').c_str());
  return d;
}

const Device & nixietube()
{
  static const Device d = make_device(0x0F, "bbi");
  return d;
}

const Device & dout()
{
  static const Device d = make_device(0x10, "bbb");  // mode + port + int8 value
  return d;
}

const Device & stepper()
{
  static const Device d = make_device(0x11, "bbii");  // mode + port + velocity(i32) + position(i32)
  return d;
}

}  // namespace mc602
}  // namespace hardware
