// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Protocol-layer tests for MC602Adapter — frame structure, parameter encoding,
// and edge cases for each actuator type. Uses the injection seam.
//
// Run: colcon test --packages-select hardware

#include "hardware/mc602_adapter.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace vw = hardware;

using CaptureFrame = std::vector<uint8_t>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Capture the outgoing frame from write_actuator via injection.
static CaptureFrame capture_write_frame(vw::MC602Adapter & adapter,
                                         std::function<void()> call)
{
  CaptureFrame captured;
  adapter.set_injection([&captured](const std::vector<uint8_t> & frame) {
    captured = frame;
    return std::vector<uint8_t>{0x0A};
  });
  call();
  return captured;
}

// ---------------------------------------------------------------------------
// Actuator frame structure tests
// ---------------------------------------------------------------------------

class ActuatorFrameTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    adapter_ = std::make_unique<vw::MC602Adapter>("/dev/ttyUSB0", 1000000);
  }

  std::unique_ptr<vw::MC602Adapter> adapter_;
};

// motor: single wheel speed — int8 virtual [-100,100]
TEST_F(ActuatorFrameTest, MotorFrame_Structure)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(1, "motor", 0.2);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[0], vw::MC602Adapter::FRAME_HEADER_0);
  EXPECT_EQ(frame[1], vw::MC602Adapter::FRAME_HEADER_1);
  // Frame: header(2) + len(1) + dev_id(1) + mode(1) + port(1) + param(1) + footer(1) = 8
  EXPECT_EQ(frame.size(), 8u);
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_MOTOR);
  EXPECT_EQ(frame[4], vw::MC602Adapter::MODE_SET);
  EXPECT_EQ(frame[5], 1u);  // port
  // 0.2 m/s / 0.03m * RAD_2_VIRTUAL ≈ 21
  EXPECT_EQ(frame[6], 21u); // speed param
  EXPECT_EQ(frame[7], vw::MC602Adapter::FRAME_FOOTER);
}

// motor: negative speed wraps correctly via uint8
TEST_F(ActuatorFrameTest, MotorFrame_NegativeSpeed)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(2, "motor", -0.2);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_MOTOR);
  EXPECT_EQ(frame[5], 2u);  // port
  // -0.2 m/s / 0.03 * 3.207 ≈ -21 → 0xEB (235)
  EXPECT_EQ(frame[6], static_cast<uint8_t>(235)); // -21 as uint8
}

// servo_bus: int32 LE angle + speed byte
TEST_F(ActuatorFrameTest, ServoBusFrame_Structure)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(3, "servo_bus", 90.0);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[0], vw::MC602Adapter::FRAME_HEADER_0);
  EXPECT_EQ(frame[1], vw::MC602Adapter::FRAME_HEADER_1);
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_SERVO_BUS);
  EXPECT_EQ(frame[4], vw::MC602Adapter::MODE_SET);
  EXPECT_EQ(frame[5], 3u);  // port

  // params[6..9] = int32 LE angle, params[10] = speed
  int32_t angle_le;
  std::memcpy(&angle_le, frame.data() + 6, sizeof(angle_le));
  EXPECT_EQ(angle_le, static_cast<int32_t>(9000)); // 90° = 9000 centi-degrees
  EXPECT_EQ(frame[10], 100u); // speed
  EXPECT_EQ(frame[11], vw::MC602Adapter::FRAME_FOOTER);
}

// servo_bus: 0 degrees → 0 int32
TEST_F(ActuatorFrameTest, ServoBusFrame_ZeroDegrees)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(1, "servo_bus", 0.0);
  });

  ASSERT_FALSE(frame.empty());
  int32_t angle_le;
  std::memcpy(&angle_le, frame.data() + 6, sizeof(angle_le));
  EXPECT_EQ(angle_le, 0);
}

// servo_pwm (180° mode): speed + uint8 angle
TEST_F(ActuatorFrameTest, ServoPwmFrame_180Mode)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(5, "servo_pwm", 0.0); // port != 7 → 180° mode
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_SERVO_PWM);
  EXPECT_EQ(frame[4], vw::MC602Adapter::MODE_SET);
  EXPECT_EQ(frame[5], 5u);  // port
  // speed param
  EXPECT_EQ(frame[6], 100u);
  // angle: 0° → 90 (center of 180° mode)
  EXPECT_EQ(frame[7], 90u);
}

// servo_pwm (270° mode, port 7): speed + uint8 angle
TEST_F(ActuatorFrameTest, ServoPwmFrame_270Mode)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(7, "servo_pwm", 0.0); // port == 7 → 270° mode
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_SERVO_PWM);
  EXPECT_EQ(frame[5], 7u);  // port
  // angle: 0° → 135 (center of 270° mode)
  EXPECT_EQ(frame[7], 135u);
}

// servo_pwm: negative angle clamps to 0
TEST_F(ActuatorFrameTest, ServoPwmFrame_ClampsNegative)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(5, "servo_pwm", -200.0);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[7], 0u); // clamped
}

// servo_pwm: positive angle clamps to max (180 or 270)
TEST_F(ActuatorFrameTest, ServoPwmFrame_ClampsPositive)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(5, "servo_pwm", 200.0);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[7], 180u); // clamped to 180° mode max
}

// stepper: int32 LE steps + speed byte
TEST_F(ActuatorFrameTest, StepperFrame_Structure)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(2, "stepper", 90.0);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_STEPPER);
  EXPECT_EQ(frame[4], vw::MC602Adapter::MODE_SET);
  EXPECT_EQ(frame[5], 2u);  // port

  // params[6..9] = int32 LE step count, params[10] = speed
  int32_t steps_le;
  std::memcpy(&steps_le, frame.data() + 6, sizeof(steps_le));
  EXPECT_EQ(steps_le, static_cast<int32_t>(adapter_->angle_to_stepper_steps(90.0)));
  EXPECT_EQ(frame[10], 50u); // speed
}

// dout: value=0 → disconnect (1), value!=0 → connect (2)
TEST_F(ActuatorFrameTest, DoutFrame_Connect)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(1, "dout", 1.0);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[3], vw::MC602Adapter::DEV_DOUT);
  EXPECT_EQ(frame[4], vw::MC602Adapter::MODE_SET);
  EXPECT_EQ(frame[5], 1u);  // port
  EXPECT_EQ(frame[6], 2u);  // connect
}

TEST_F(ActuatorFrameTest, DoutFrame_Disconnect)
{
  auto frame = capture_write_frame(*adapter_, [this]() {
    adapter_->open();
    adapter_->write_actuator(1, "dout", 0.0);
  });

  ASSERT_FALSE(frame.empty());
  EXPECT_EQ(frame[6], 1u);  // disconnect
}

// write_motor4: frame contains DEV_MOTOR4 and 4 wheel speeds
TEST_F(ActuatorFrameTest, WriteMotor4_Frame)
{
  std::vector<uint8_t> captured;
  adapter_->set_injection([&captured](const std::vector<uint8_t> & frame) {
    captured = frame;
    return std::vector<uint8_t>{0x0A};
  });
  adapter_->open();
  adapter_->write_motor4(50, -30, 80, -60);

  ASSERT_FALSE(captured.empty());
  // Frame: header(2) + len(1) + dev_id(1) + mode(1) + port(1) + 4×speed(4) + footer(1) = 11
  EXPECT_EQ(captured[3], vw::MC602Adapter::DEV_MOTOR4);
  EXPECT_EQ(captured[4], vw::MC602Adapter::MODE_SET);
  EXPECT_EQ(captured[5], 0u);  // port (write_motor4 uses port=0)
  // Wheel order: M2(FL)=50, M1(FR)=-30, M3(RL)=80, M4(RR)=-60
  EXPECT_EQ(captured[6], 50u);
  EXPECT_EQ(captured[7], static_cast<uint8_t>(226)); // -30
  EXPECT_EQ(captured[8], 80u);
  EXPECT_EQ(captured[9], static_cast<uint8_t>(196)); // -60
}

// write_motor4: all zeros — valid stop command
TEST_F(ActuatorFrameTest, WriteMotor4_AllZeros)
{
  std::vector<uint8_t> captured;
  adapter_->set_injection([&captured](const std::vector<uint8_t> & frame) {
    captured = frame;
    return std::vector<uint8_t>{0x0A};
  });
  adapter_->open();
  adapter_->write_motor4(0, 0, 0, 0);

  ASSERT_FALSE(captured.empty());
  EXPECT_EQ(captured[6], 0u);
  EXPECT_EQ(captured[7], 0u);
  EXPECT_EQ(captured[8], 0u);
  EXPECT_EQ(captured[9], 0u);
}

// ---------------------------------------------------------------------------
// Unit conversion edge cases
// ---------------------------------------------------------------------------

class ConversionEdgeCaseTest : public ::testing::Test
{};

// counts_to_meters: zero counts → zero distance
TEST_F(ConversionEdgeCaseTest, CountsToMeters_Zero)
{
  EXPECT_NEAR(vw::MC602Adapter::counts_to_meters(0, 0.03), 0.0, 1e-10);
}

// counts_to_meters: 2015.13 counts (1 rev) = 2π * 0.03
TEST_F(ConversionEdgeCaseTest, CountsToMeters_OneRevolution)
{
  double result = vw::MC602Adapter::counts_to_meters(
    static_cast<int32_t>(vw::MC602Adapter::ENCODER_COUNTS_PER_REV), 0.03);
  EXPECT_NEAR(result, 2.0 * vw::MC602_PI * 0.03, 1e-4);
}

// counts_to_meters: large count (10 rev) is accurate
TEST_F(ConversionEdgeCaseTest, CountsToMeters_TenRevolutions)
{
  double result = vw::MC602Adapter::counts_to_meters(
    static_cast<int32_t>(vw::MC602Adapter::ENCODER_COUNTS_PER_REV * 10), 0.03);
  EXPECT_NEAR(result, 2.0 * vw::MC602_PI * 0.03 * 10, 1e-3);
}

// meters_to_virtual: round-trip with clamp_virtual
TEST_F(ConversionEdgeCaseTest, MetersToVirtual_RoundTrip)
{
  double v_mps = 0.5;
  double virt = vw::MC602Adapter::meters_to_virtual(v_mps, 0.03);
  int8_t clamped = vw::MC602Adapter::clamp_virtual(v_mps);
  // virt ≈ 0.5/0.03 * 3.207 ≈ 53
  EXPECT_NEAR(virt, clamped, 1.0);
}

// clamp_virtual: 0.94 m/s → virtual ≈ 100 (just at max)
TEST_F(ConversionEdgeCaseTest, ClampVirtual_BelowMax)
{
  EXPECT_EQ(vw::MC602Adapter::clamp_virtual(0.94), 100);
}

// clamp_virtual: 1.5 m/s (virtual ≈ 160 > 100 → clamped)
TEST_F(ConversionEdgeCaseTest, ClampVirtual_AboveMax)
{
  EXPECT_EQ(vw::MC602Adapter::clamp_virtual(1.5), 100);
}

// clamp_virtual: symmetry — positive and negative of same magnitude
TEST_F(ConversionEdgeCaseTest, ClampVirtual_Symmetric)
{
  for (double v : {0.1, 0.2, 0.3, 0.4}) {
    EXPECT_EQ(vw::MC602Adapter::clamp_virtual(v),
              -vw::MC602Adapter::clamp_virtual(-v));
  }
}

// angle_to_servo_bus: negative angle
TEST_F(ConversionEdgeCaseTest, AngleToServoBus_Negative)
{
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_bus(-90.0), -9000);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_bus(-180.0), -18000);
}

// angle_to_stepper_steps: 360° = 2π rad / STEPPER_RAD_PER_STEP ≈ 3200 steps
TEST_F(ConversionEdgeCaseTest, AngleToStepperSteps_FullRotation)
{
  // 360° → 2π rad → 2π / 0.001963 ≈ 3200 steps
  double steps = vw::MC602Adapter::angle_to_stepper_steps(360.0);
  EXPECT_NEAR(steps, 2.0 * vw::MC602_PI / vw::MC602Adapter::STEPPER_RAD_PER_STEP, 1.0);
}

// ---------------------------------------------------------------------------
// Port validation tests
// ---------------------------------------------------------------------------

class PortValidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    adapter_ = std::make_unique<vw::MC602Adapter>("/dev/ttyUSB0", 1000000);
    adapter_->set_injection([](const std::vector<uint8_t> &) {
      return std::vector<uint8_t>{0x0A};
    });
    adapter_->open();
  }

  std::unique_ptr<vw::MC602Adapter> adapter_;
};

// Port 0 rejected for all actuator types
TEST_F(PortValidationTest, WriteActuator_RejectsPortZero)
{
  EXPECT_THROW(adapter_->write_actuator(0, "motor", 1.0), std::runtime_error);
  EXPECT_THROW(adapter_->write_actuator(0, "servo_bus", 1.0), std::runtime_error);
  EXPECT_THROW(adapter_->write_actuator(0, "stepper", 1.0), std::runtime_error);
  EXPECT_THROW(adapter_->write_actuator(0, "dout", 1.0), std::runtime_error);
}

// Motor port 5 rejected (max is 4)
TEST_F(PortValidationTest, WriteActuator_MotorPortAboveMax)
{
  EXPECT_THROW(adapter_->write_actuator(5, "motor", 1.0), std::runtime_error);
}

// Servo port 8 rejected (max is 7)
TEST_F(PortValidationTest, WriteActuator_ServoPortAboveMax)
{
  EXPECT_THROW(adapter_->write_actuator(8, "servo_bus", 1.0), std::runtime_error);
  EXPECT_THROW(adapter_->write_actuator(8, "servo_pwm", 1.0), std::runtime_error);
}

// Stepper port 5 rejected (max is 4)
TEST_F(PortValidationTest, WriteActuator_StepperPortAboveMax)
{
  EXPECT_THROW(adapter_->write_actuator(5, "stepper", 1.0), std::runtime_error);
}

// Motor port 4 is the maximum valid
TEST_F(PortValidationTest, WriteActuator_MotorPort4_Accepted)
{
  EXPECT_NO_THROW(adapter_->write_actuator(4, "motor", 1.0));
}

// Servo port 7 is the maximum valid
TEST_F(PortValidationTest, WriteActuator_ServoPort7_Accepted)
{
  EXPECT_NO_THROW(adapter_->write_actuator(7, "servo_bus", 1.0));
  EXPECT_NO_THROW(adapter_->write_actuator(7, "servo_pwm", 1.0));
}

// NaN and Inf rejected by write_actuator
TEST_F(PortValidationTest, WriteActuator_RejectsNaN)
{
  EXPECT_THROW(adapter_->write_actuator(1, "motor", std::nan("")), std::runtime_error);
}

TEST_F(PortValidationTest, WriteActuator_RejectsInf)
{
  EXPECT_THROW(adapter_->write_actuator(1, "motor", std::numeric_limits<double>::infinity()),
               std::runtime_error);
  EXPECT_THROW(adapter_->write_actuator(1, "motor", -std::numeric_limits<double>::infinity()),
               std::runtime_error);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
