// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for MC602Adapter — covers BaseController contract, port
// bounds, value validation. Hardware protocol tests land in Plan B.

#include "vehicle_wbt_platform_cpp/mc602_adapter.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

using vehicle_wbt_platform_cpp::BaseController;
using vehicle_wbt_platform_cpp::MC602Adapter;

namespace
{
// A minimal FakeMC602 that records every call and returns canned values.
class FakeController : public BaseController
{
public:
  bool opened = false;
  int read_calls = 0;
  int write_calls = 0;
  double last_value = 0.0;
  uint8_t last_port = 0;
  std::string last_sensor_type;
  std::string last_actuator_type;

  void open() override { opened = true; }
  void close() override { opened = false; }
  bool is_open() const override { return opened; }
  std::string serial_port() const override { return "/fake/ttyUSB0"; }
  uint32_t baud() const override { return 1000000; }
  double read_sensor(uint8_t port_id, const std::string & sensor_type) override
  {
    read_calls++;
    last_port = port_id;
    last_sensor_type = sensor_type;
    return 0.42;
  }
  void write_actuator(uint8_t port_id, const std::string & actuator_type, double value) override
  {
    write_calls++;
    last_port = port_id;
    last_actuator_type = actuator_type;
    last_value = value;
  }
  std::map<std::string, uint32_t> enumerate_ports() const override
  {
    return {{"motor", 6}, {"servo", 7}, {"stepper", 3}, {"io", 8}};
  }
};
}  // namespace

TEST(MC602AdapterTest, ConstructorRejectsUnsupportedBaud)
{
  EXPECT_THROW(MC602Adapter("/dev/ttyUSB0", 9600), std::runtime_error);
  EXPECT_NO_THROW(MC602Adapter("/dev/ttyUSB0", 1000000));
  EXPECT_NO_THROW(MC602Adapter("/dev/ttyUSB0", 380400));
  EXPECT_NO_THROW(MC602Adapter("/dev/ttyUSB0", 115200));
}

TEST(MC602AdapterTest, OpenCloseIdempotent)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  EXPECT_FALSE(a.is_open());
  a.open();
  EXPECT_TRUE(a.is_open());
  a.close();
  EXPECT_FALSE(a.is_open());
  a.close();  // second close must not throw
  EXPECT_FALSE(a.is_open());
}

TEST(MC602AdapterTest, PortMetadataRoundTrips)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  EXPECT_EQ(a.serial_port(), "/dev/ttyUSB0");
  EXPECT_EQ(a.baud(), 1000000u);
}

TEST(MC602AdapterTest, EnumeratePortsMatchesHardwareSpec)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  auto ports = a.enumerate_ports();
  EXPECT_EQ(ports.at("motor"), 6u);
  EXPECT_EQ(ports.at("servo"), 7u);
  EXPECT_EQ(ports.at("stepper"), 3u);
  EXPECT_EQ(ports.at("io"), 8u);
}

TEST(MC602AdapterTest, ReadRequiresOpen)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  EXPECT_THROW(a.read_sensor(7, "ir"), std::runtime_error);
}

TEST(MC602AdapterTest, WriteRequiresOpen)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  EXPECT_THROW(a.write_actuator(5, "motor", 0.5), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsUnknownSensorType)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.read_sensor(7, "tachyon_beam"), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsUnknownActuatorType)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.write_actuator(5, "warp_drive", 0.0), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsPortIdZeroForSensor)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.read_sensor(0, "ir"), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsPortIdAboveIOMaxForSensor)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.read_sensor(9, "ir"), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsPortIdAboveMotorMaxForMotor)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.write_actuator(7, "motor", 0.5), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsNaNValue)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.write_actuator(5, "motor", std::nan("")), std::runtime_error);
}

TEST(MC602AdapterTest, RejectsInfValue)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.open();
  EXPECT_THROW(a.write_actuator(5, "motor", std::numeric_limits<double>::infinity()),
               std::runtime_error);
}
