// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Protocol golden-frame tests for MC602Adapter.
//
// Every frame the driver emits is compared BYTE-EXACTLY against the ground
// truth produced by the authoritative controller_lab tool (the baidu_smartcar
// SDK that actually ran this robot). This is what proves the driver speaks
// the real protocol without needing hardware:
//   len = payload_len + 4; payload = dev + mode + [port] + args per format.

#include "hardware/mc602_adapter.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

using hardware::MC602Adapter;

namespace
{

// Parse a hex string like "77 68 0a 01 02 ..." or "77680a010..." into bytes.
std::vector<uint8_t> hex(const std::string & s)
{
  std::vector<uint8_t> out;
  std::string cleaned;
  for (char c : s) {
    if (c != ' ' && c != '\n') cleaned += c;
  }
  for (size_t i = 0; i + 1 < cleaned.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoi(cleaned.substr(i, 2), nullptr, 16)));
  }
  return out;
}

// Adapter wired with an injection that captures the last outgoing frame.
struct CaptureHarness
{
  MC602Adapter adapter{"/dev/ttyUSB0", 1000000};
  std::vector<uint8_t> captured;

  CaptureHarness()
  {
    adapter.set_injection([this](const std::vector<uint8_t> & frame) {
      captured = frame;
      return std::vector<uint8_t>{};  // writes ignore the response
    });
    adapter.open();
  }
};

#define EXPECT_FRAME(harness, expected_hex) \
  EXPECT_EQ((harness).captured, hex(expected_hex))

// Ground-truth frames from controller_lab (verified on the real robot).
// clang-format off
constexpr const char * kGoldenMotor4    = "77680a01020a141e280a";
constexpr const char * kGoldenMotorP6   = "776808020206320a";
constexpr const char * kGoldenServoBus  = "77680b060203646400000a";
constexpr const char * kGoldenServoPwm  = "77680905020787640a";
constexpr const char * kGoldenStepper   = "77680f11020332000000640000000a";
constexpr const char * kGoldenDoutOn    = "776808100204010a";
constexpr const char * kGoldenDoutOff   = "776808100204000a";
// clang-format on

}  // namespace

TEST(Mc602ProtocolTest, GoldenWriteFramesMatchControllerLab)
{
  CaptureHarness h;

  h.adapter.set_motor4(10, 20, 30, 40);
  EXPECT_FRAME(h, kGoldenMotor4);

  h.adapter.set_motor(6, 50);
  EXPECT_FRAME(h, kGoldenMotorP6);

  h.adapter.set_servo_bus(3, 100, 100);
  EXPECT_FRAME(h, kGoldenServoBus);

  h.adapter.set_servo_pwm(7, 135, 100);
  EXPECT_FRAME(h, kGoldenServoPwm);

  h.adapter.set_stepper(3, 50, 100);
  EXPECT_FRAME(h, kGoldenStepper);

  h.adapter.set_dout(4, 1);  // P4 relay ON = one shot
  EXPECT_FRAME(h, kGoldenDoutOn);

  h.adapter.set_dout(4, 0);
  EXPECT_FRAME(h, kGoldenDoutOff);
}

TEST(Mc602ProtocolTest, Motor4StopFrameIsAllZero)
{
  CaptureHarness h;
  h.adapter.set_motor4(0, 0, 0, 0);
  EXPECT_FRAME(h, "77680a0102000000000a");
}

TEST(Mc602ProtocolTest, Encoder4ReadParsesFourCounts)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  // Controller-lab encoder4 response payload: dev(03) mode(01) + 4×int32.
  const std::vector<uint8_t> payload = hex(
    "03 01 01000000 02000000 03000000 04000000");
  a.set_injection([&](const std::vector<uint8_t> & /*frame*/) { return payload; });
  a.open();

  auto counts = a.read_encoder4();
  EXPECT_EQ(counts[0], 1);
  EXPECT_EQ(counts[1], 2);
  EXPECT_EQ(counts[2], 3);
  EXPECT_EQ(counts[3], 4);
}

TEST(Mc602ProtocolTest, IrReadParsesDistance)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  // IR response payload: dev(07) mode(01) port(08) H=0x07d0 (2000 mm).
  const std::vector<uint8_t> payload = hex("07 01 08 d0 07");
  a.set_injection([&](const std::vector<uint8_t> & /*frame*/) { return payload; });
  a.open();

  EXPECT_FLOAT_EQ(a.read_ir(8), 2.0f);  // 2000 mm → 2.0 m
  EXPECT_EQ(a.read_sensor_raw(8, MC602Adapter::SENSOR_INFRARED), 2000u);
}

TEST(Mc602ProtocolTest, FrameLenIsPayloadPlusFour)
{
  // The core rule that was previously broken: len = payload_len + 4.
  CaptureHarness h;
  h.adapter.set_dout(4, 1);  // payload = 10 02 04 01 (4 bytes) → len 8
  ASSERT_EQ(h.captured.size(), 8u);
  EXPECT_EQ(h.captured[2], 8u);  // len byte
  EXPECT_EQ(h.captured[0], 0x77);
  EXPECT_EQ(h.captured[1], 0x68);
  EXPECT_EQ(h.captured.back(), 0x0A);
}
