// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Unit tests for SerialPort and MC602Adapter protocol layer.
//
// Uses the injection/test seam: set_response_handler() on SerialPort
// bypasses real fd I/O; set_injection() on MC602Adapter bypasses both
// the adapter logic AND the serial I/O. Both are tested here.

#include <gmock/gmock.h>

#include <arpa/inet.h>  // htonl/ntohl
#include <cmath>
#include <cstring>

#include "vehicle_wbt_platform_cpp/serial_port.hpp"
#include "vehicle_wbt_platform_cpp/mc602_adapter.hpp"

namespace vw = vehicle_wbt_platform_cpp;

// ============================================================================
// SerialPort tests — frame structure, injection seam, timeout behavior
// ============================================================================

class SerialPortTest : public ::testing::Test
{
protected:
  // We never open a real fd; the injection handler replaces exchange().
  void SetUp() override {}
};

// build_get produces the correct binary frame per hardware-comm.md §MC602
TEST_F(SerialPortTest, BuildGetFrame_Structure)
{
  // Not directly testing SerialPort::build_get — that's a static helper on
  // MC602Adapter. Here we verify the frame via MC602Adapter::build_get.
  auto frame = vw::MC602Adapter::build_get(0x07, 1, {1});  // IR sensor

  // Frame: header(2) + len(1) + dev_id(1) + mode(1) + port(1) + params(1) + footer(1) = 8
  ASSERT_EQ(frame.size(), 8u);
  EXPECT_EQ(frame[0], 0x77);
  EXPECT_EQ(frame[1], 0x68);

  // Length byte = 4 (dev_id + mode + port + footer) + 1 param = 5
  EXPECT_EQ(frame[2], 5u);

  // dev_id, mode, port
  EXPECT_EQ(frame[3], 0x07u);  // DEV_SENSOR_MULTI
  EXPECT_EQ(frame[4], 1u);     // MODE_GET
  EXPECT_EQ(frame[5], 1u);     // port 1

  // param: sub_mode = SENSOR_INFRARED
  EXPECT_EQ(frame[6], 1u);

  // Footer
  EXPECT_EQ(frame[7], 0x0Au);
}

// build_set produces the correct binary frame
TEST_F(SerialPortTest, BuildSetFrame_Structure)
{
  // Single motor speed command: dev_id=MOTOR(0x02), port=1, speed=50
  std::vector<uint8_t> params = {50};
  auto frame = vw::MC602Adapter::build_set(0x02, 1, params);

  ASSERT_EQ(frame.size(), 8u);
  EXPECT_EQ(frame[0], 0x77);
  EXPECT_EQ(frame[1], 0x68);
  EXPECT_EQ(frame[2], 5u);     // length
  EXPECT_EQ(frame[3], 0x02u);  // dev_id = MOTOR
  EXPECT_EQ(frame[4], 2u);     // MODE_SET
  EXPECT_EQ(frame[5], 1u);     // port
  EXPECT_EQ(frame[6], 50u);    // speed param
  EXPECT_EQ(frame[7], 0x0Au);  // footer
}

// build_get with no params (e.g., encoder4 read)
TEST_F(SerialPortTest, BuildGetFrame_NoParams)
{
  auto frame = vw::MC602Adapter::build_get(0x03, 0, {});

  // Frame: header(2) + len(1) + dev_id(1) + mode(1) + port(1) + footer(1) = 7
  ASSERT_EQ(frame.size(), 7u);
  EXPECT_EQ(frame[0], 0x77);
  EXPECT_EQ(frame[1], 0x68);
  EXPECT_EQ(frame[2], 4u);     // length = 4 (no params)
  EXPECT_EQ(frame[3], 0x03u);  // ENCODER4
  EXPECT_EQ(frame[4], 1u);     // MODE_GET
  EXPECT_EQ(frame[5], 0u);     // port 0
  EXPECT_EQ(frame[6], 0x0Au);  // footer
}

// set_response_handler replaces real I/O with the handler's output
TEST_F(SerialPortTest, SetResponseHandler_BypassesRealIO)
{
  vw::SerialPort port("/dev/ttyUSB0", 1000000);

  // Inject a canned response frame that looks like a valid MC602 response.
  std::vector<uint8_t> canned = {
    0x77, 0x68,    // header
    0x08,          // length = 8
    0x07,          // dev_id = SENSOR_MULTI
    0x01,          // mode = GET (response echoes request mode)
    0x02,          // port 2
    0x34, 0x12,    // uint16 LE = 0x1234
    0x0A           // footer
  };

  port.set_response_handler([canned](const std::vector<uint8_t> &) {
    return canned;
  });

  // SerialPort::exchange() returns the FULL frame (no stripping).
  // Frame: header(2) + length(1) + payload(4) + footer(1) = 8 bytes.
  auto result = port.exchange({0x77, 0x68, 0x05, 0x07, 0x01, 0x02, 0x01, 0x0A});

  ASSERT_EQ(result.size(), 9u);
  EXPECT_EQ(result[0], 0x77u);   // header
  EXPECT_EQ(result[1], 0x68u);   // header
  EXPECT_EQ(result[2], 0x08u);   // length
  EXPECT_EQ(result[3], 0x07u);   // dev_id
  EXPECT_EQ(result[4], 0x01u);   // mode
  EXPECT_EQ(result[5], 0x02u);   // port
  EXPECT_EQ(result[6], 0x34u);   // uint16 LE low
  EXPECT_EQ(result[7], 0x12u);   // uint16 LE high
  // footer is 0x0A = 10, verified by result.back() below
  EXPECT_EQ(result.back(), 0x0Au); // footer
}

// exchange without a handler throws if port is not open (no fd)
TEST_F(SerialPortTest, Exchange_ThrowsWhenNotOpen)
{
  vw::SerialPort port("/dev/nonexistent", 1000000);
  EXPECT_THROW(port.exchange({0x77}), std::runtime_error);
}

// ============================================================================
// MC602Adapter tests — injection seam, protocol constants, frame routing
// ============================================================================

class MC602AdapterInjectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    adapter_ = std::make_unique<vw::MC602Adapter>("/dev/ttyUSB0", 1000000);
    // Enable injection so open() skips real serial port.
    adapter_->set_injection([](const std::vector<uint8_t> &) -> std::vector<uint8_t> {
      return std::vector<uint8_t>{0x0A};
    });
    adapter_->open();
  }

  std::unique_ptr<vw::MC602Adapter> adapter_;
};

// Protocol constants match hardware spec
TEST_F(MC602AdapterInjectionTest, ProtocolConstants)
{
  EXPECT_EQ(vw::MC602Adapter::DEV_MOTOR4, 0x01u);
  EXPECT_EQ(vw::MC602Adapter::DEV_MOTOR, 0x02u);
  EXPECT_EQ(vw::MC602Adapter::DEV_ENCODER4, 0x03u);
  EXPECT_EQ(vw::MC602Adapter::DEV_ENCODER, 0x04u);
  EXPECT_EQ(vw::MC602Adapter::DEV_SERVO_PWM, 0x05u);
  EXPECT_EQ(vw::MC602Adapter::DEV_SERVO_BUS, 0x06u);
  EXPECT_EQ(vw::MC602Adapter::DEV_SENSOR_MULTI, 0x07u);
  EXPECT_EQ(vw::MC602Adapter::DEV_DOUT, 0x10u);
  EXPECT_EQ(vw::MC602Adapter::DEV_STEPPER, 0x11u);
  EXPECT_EQ(vw::MC602Adapter::FRAME_HEADER_0, 0x77u);
  EXPECT_EQ(vw::MC602Adapter::FRAME_HEADER_1, 0x68u);
  EXPECT_EQ(vw::MC602Adapter::FRAME_FOOTER, 0x0Au);
}

// Encoder calibration: counts_per_rev * encoder2rad == 2π
TEST_F(MC602AdapterInjectionTest, EncoderCalibrationConsistent)
{
  const double two_pi = 2.0 * vw::MC602_PI;
  double computed = vw::MC602Adapter::ENCODER_COUNTS_PER_REV *
                    vw::MC602Adapter::ENCODER_2_RAD;
  EXPECT_NEAR(computed, two_pi, 1e-6);
}

// Virtual speed: RAD_2_VIRTUAL * 2π * 100 ≈ ENCODER_COUNTS_PER_REV
TEST_F(MC602AdapterInjectionTest, VirtualSpeedConversionConsistent)
{
  double computed = vw::MC602Adapter::RAD_2_VIRTUAL * 2.0 * vw::MC602_PI * 100.0;
  EXPECT_NEAR(computed, vw::MC602Adapter::ENCODER_COUNTS_PER_REV, 1e-3);
}

// Stepper: STEPPER_RAD_PER_STEP should equal π/180 * 1.8/16
TEST_F(MC602AdapterInjectionTest, StepperConversionConstant)
{
  double expected = vw::MC602_PI / 180.0 * 1.8 / 16.0;
  EXPECT_NEAR(vw::MC602Adapter::STEPPER_RAD_PER_STEP, expected, 1e-10);
}

// clamp_virtual: m/s=0 → virtual=0, m/s=1.0 → ~107 (clamped to 100)
TEST_F(MC602AdapterInjectionTest, ClampVirtual_Range)
{
  EXPECT_EQ(vw::MC602Adapter::clamp_virtual(0.0), 0);
  EXPECT_EQ(vw::MC602Adapter::clamp_virtual(1.0), 100);  // clamped
  EXPECT_EQ(vw::MC602Adapter::clamp_virtual(-1.0), -100);
}

// angle_to_servo_bus: degrees → int32 LE (centi-degrees)
TEST_F(MC602AdapterInjectionTest, AngleToServoBus)
{
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_bus(0.0), 0);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_bus(90.0), 9000);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_bus(-45.0), -4500);
}

// angle_to_servo_pwm: 180° mode
TEST_F(MC602AdapterInjectionTest, AngleToServoPwm_180Mode)
{
  // center at 90°: angle=0 → 90, angle=-90 → 0, angle=90 → 180
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(0.0, false), 90);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(-90.0, false), 0);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(90.0, false), 180);
  // Clamping
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(200.0, false), 180);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(-200.0, false), 0);
}

// angle_to_servo_pwm: 270° mode
TEST_F(MC602AdapterInjectionTest, AngleToServoPwm_270Mode)
{
  // center at 135°: angle=0 → 135, angle=-135 → 0, angle=135 → 270
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(0.0, true), 135);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(-135.0, true), 0);
  EXPECT_EQ(vw::MC602Adapter::angle_to_servo_pwm(135.0, true), 270);
}

// angle_to_stepper_steps: 180° → 180 * 16 / 1.8 = 1600 micro-steps
TEST_F(MC602AdapterInjectionTest, AngleToStepperSteps)
{
  EXPECT_EQ(vw::MC602Adapter::angle_to_stepper_steps(0.0), 0);
  EXPECT_EQ(vw::MC602Adapter::angle_to_stepper_steps(180.0), 1600);
}

// counts_to_meters: forward conversion
TEST_F(MC602AdapterInjectionTest, CountsToMeters)
{
  // 2015.13 counts = 1 revolution = 2π * 0.03 m ≈ 0.1885 m
  double result = vw::MC602Adapter::counts_to_meters(
    static_cast<int32_t>(vw::MC602Adapter::ENCODER_COUNTS_PER_REV), 0.03);
  EXPECT_NEAR(result, 2.0 * vw::MC602_PI * 0.03, 1e-4);
}

// read_sensor throws when not open
TEST_F(MC602AdapterInjectionTest, ReadSensor_ThrowsWhenClosed)
{
  adapter_->close();
  EXPECT_THROW(
    adapter_->read_sensor(1, "ir"),
    std::runtime_error);
}

// write_actuator throws for unknown actuator type
TEST_F(MC602AdapterInjectionTest, WriteActuator_ThrowsOnUnknownType)
{
  adapter_->set_injection([](const std::vector<uint8_t> &) -> std::vector<uint8_t> {
    return {};
  });
  EXPECT_THROW(
    adapter_->write_actuator(1, "nonexistent", 1.0),
    std::runtime_error);
}

// write_actuator throws when port out of range
TEST_F(MC602AdapterInjectionTest, WriteActuator_ThrowsOnBadPort)
{
  adapter_->set_injection([](const std::vector<uint8_t> &) -> std::vector<uint8_t> {
    return {};
  });
  EXPECT_THROW(
    adapter_->write_actuator(99, "motor", 1.0),
    std::runtime_error);
}

// read_encoder4 returns [FL(M2), FR(M1), RL(M3), RR(M4)] via injection
TEST_F(MC602AdapterInjectionTest, ReadEncoder4_Injection)
{
  // Simulate encoder4 response payload: dev_id + mode + port + 4× int32(BE)
  std::array<int32_t, 4> counts = {100, -200, 300, -400};
  std::vector<uint8_t> payload(3 + 4 * 4);
  payload[0] = 0x03;  // dev_id
  payload[1] = 0x01;  // mode
  payload[2] = 0x00;  // port
  for (int i = 0; i < 4; ++i) {
    uint32_t be = htonl(static_cast<uint32_t>(counts[i]));
    std::memcpy(payload.data() + 3 + i * 4, &be, 4);
  }

  adapter_->set_injection([payload](const std::vector<uint8_t> &) -> std::vector<uint8_t> {
    return payload;
  });

  auto result = adapter_->read_encoder4();
  ASSERT_EQ(result.size(), 4u);
  EXPECT_EQ(result[0], 100);
  EXPECT_EQ(result[1], -200);
  EXPECT_EQ(result[2], 300);
  EXPECT_EQ(result[3], -400);
}

// read_infrared converts mm→meters via injection
TEST_F(MC602AdapterInjectionTest, ReadInfrared_Conversion)
{
  // 1500 mm = 1.5 m
  uint16_t mm = 1500;
  std::vector<uint8_t> payload(5);
  payload[0] = 0x07;
  payload[1] = 0x01;
  payload[2] = 0x01;
  uint16_t le = htole16(mm);
  std::memcpy(payload.data() + 3, &le, 2);

  adapter_->set_injection([payload](const std::vector<uint8_t> &) -> std::vector<uint8_t> {
    return payload;
  });

  float dist = adapter_->read_infrared(1);
  EXPECT_NEAR(dist, 1.5f, 0.001f);
}

// write_motor4 sends correct frame via injection
TEST_F(MC602AdapterInjectionTest, WriteMotor4_InjectionFrame)
{
  std::vector<uint8_t> captured_frame;
  adapter_->set_injection(
    [&captured_frame](const std::vector<uint8_t> & frame) -> std::vector<uint8_t> {
      captured_frame = frame;
      return std::vector<uint8_t>{0x0A};  // minimal valid response
    });

  adapter_->write_motor4(50, -30, 80, -60);

  ASSERT_FALSE(captured_frame.empty());
  EXPECT_EQ(captured_frame[0], 0x77u);
  EXPECT_EQ(captured_frame[1], 0x68u);
  EXPECT_EQ(captured_frame[3], 0x01u);  // DEV_MOTOR4
  EXPECT_EQ(captured_frame[4], 2u);     // MODE_SET
  // params at [6-9]: M2(FL)=50, M1(FR)=-30, M3(RL)=80, M4(RR)=-60
  EXPECT_EQ(captured_frame[6], 50u);
  EXPECT_EQ(captured_frame[7], static_cast<uint8_t>(226));  // -30 as uint8
  EXPECT_EQ(captured_frame[8], 80u);
  EXPECT_EQ(captured_frame[9], static_cast<uint8_t>(196));  // -60 as uint8
}

// write_actuator "motor" sends correct frame
TEST_F(MC602AdapterInjectionTest, WriteActuator_Motor_InjectionFrame)
{
  std::vector<uint8_t> captured_frame;
  adapter_->set_injection(
    [&captured_frame](const std::vector<uint8_t> & frame) -> std::vector<uint8_t> {
      captured_frame = frame;
      return std::vector<uint8_t>{0x0A};
    });

  // v=0.2 m/s, wheel_r=0.03 → virtual = 0.2/0.03 * 3.207 ≈ 21
  adapter_->write_actuator(1, "motor", 0.2);

  ASSERT_FALSE(captured_frame.empty());
  EXPECT_EQ(captured_frame[3], 0x02u);  // DEV_MOTOR
  EXPECT_EQ(captured_frame[4], 2u);     // MODE_SET
  EXPECT_EQ(captured_frame[5], 1u);     // port
}

// read_sensor "ir" via injection returns raw uint16
TEST_F(MC602AdapterInjectionTest, ReadSensor_IR_Injection)
{
  // Simulate IR response: dev_id + mode + port + uint16_le(mm)
  uint16_t mm = 500;
  std::vector<uint8_t> payload(5);
  payload[0] = 0x07;
  payload[1] = 0x01;
  payload[2] = 0x01;
  uint16_t le = htole16(mm);
  std::memcpy(payload.data() + 3, &le, 2);

  adapter_->set_injection([payload](const std::vector<uint8_t> &) -> std::vector<uint8_t> {
    return payload;
  });

  double raw = adapter_->read_sensor(1, "ir");
  // read_sensor("ir") returns raw uint16; node layer converts mm→m
  EXPECT_EQ(raw, 500.0);
}

// enumerate_ports returns expected port counts
TEST_F(MC602AdapterInjectionTest, EnumeratePorts)
{
  auto ports = adapter_->enumerate_ports();
  ASSERT_EQ(ports.size(), 4u);
  EXPECT_EQ(ports["motor"], 4u);
  EXPECT_EQ(ports["servo"], 7u);
  EXPECT_EQ(ports["stepper"], 4u);
  EXPECT_EQ(ports["io"], 16u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
