// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// mc602_protocol — faithful port of the MC602 program-mode protocol
// (controller_lab / baidu_smartcar SDK, authoritative for this robot).
//
// Frame:  77 68 <len> <payload> 0A      len = payload_len + 4  (no checksum)
// Payload: dev_id + mode + [port] + args, packed per the device's format
// string (little-endian). Port is present only when the command passes one;
// args are zero-padded / truncated to fill the format.
//
// This is the DRIVER protocol layer: it knows the wire format, nothing about
// business meaning. All values are raw protocol values — the caller decides
// (servo angle byte, stepper velocity/position, dout on/off, etc.).

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace hardware
{

namespace mc602
{

// Payload slots after dev_id: one per format char, each with a C type.
//   b = int8   B = uint8   h = int16   H = uint16   i = int32
enum class Slot : uint8_t { I8, U8, I16, U16, I32 };

// A device's command template (mirrors DEVICE_DEFS + StructData semantics).
class Device
{
public:
  Device(uint8_t dev_id, std::vector<Slot> slots)
  : dev_id_(dev_id), slots_(std::move(slots))
  {
  }

  uint8_t dev_id() const { return dev_id_; }

  // Number of pack slots after dev_id (self.len in the SDK): len(fmt)+1
  // where fmt = slots.size(). This equals the SDK's `len("<b"+fmt)-1`.
  size_t slot_count() const { return slots_.size() + 1; }

  // Build the raw payload for this device:
  //   payload = pack(slots, dev_id, mode, [port], args...)
  // mode is always emitted (explicit, or device default, or 0).
  // port is emitted only when opt_port is set. args are zero-padded to fill.
  std::vector<uint8_t> build_payload(
    std::optional<uint8_t> mode, std::optional<uint8_t> opt_port,
    const std::vector<int64_t> & args) const;

  // Parse a response payload back into the device's values (after dev/mode/port).
  // Returns the decoded values (raw). Caller maps to meaning.
  std::vector<int64_t> parse_payload(const std::vector<uint8_t> & payload) const;

private:
  uint8_t dev_id_;
  std::vector<Slot> slots_;
};

// ---- Convenience device constructors (mirror DEVICE_DEFS) ----
const Device & motor4();        // 0x01 "bbbbb" — mode + 4×int8 speeds, NO port
const Device & motor();         // 0x02 "bbb"   — mode + port + int8 speed
const Device & encoder4();      // 0x03 "biiii" — mode + 4×int32, NO port
const Device & encoder();       // 0x04 "bbi"   — mode + port + int32
const Device & servo_pwm();     // 0x05 "bbBB"  — mode + port + angle(u8) + speed(u8)
const Device & servo_bus();     // 0x06 "bbbbh" — mode + port + angle(i8) + speed(i16)
const Device & sensor(uint8_t sub_mode);  // 0x07 "bbH" — mode + port + H
const Device & analog_a();       // 0x08 "bbH"  — mode + port + H
const Device & power();          // 0x0C "bi"   — mode + int32 (battery/voltage)
const Device & beep();           // 0x0A "BBB"  — mode + freq + duration (no port)
const Device & board_key();      // 0x0D "bbb"  — mode + port + value
const Device & bluetooth();      // 0x09 "BBBBi"— mode + pad values
const Device & led_light();      // 0x0E "bbBBBB"— mode + port + rgb
const Device & led_show();       // 0x0B "b*101" — mode + display buffer
const Device & nixietube();      // 0x0F "bbi"   — mode + int32 value
const Device & dout();           // 0x10 "bbb"   — mode + port + int8 value
const Device & stepper();        // 0x11 "bbii"  — mode + port + velocity(i32) + position(i32)

}  // namespace mc602

}  // namespace hardware
