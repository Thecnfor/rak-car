// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SystemIo unit tests — injected fake device, verifies semantic dispatch,
// type validation, and error surfacing. No serial, no ROS2.

#include "hardware/system_io.hpp"

#include <gtest/gtest.h>

using namespace hardware;

namespace
{
// 注入式假设备:记录调用,返回固定值。
class FakeDevice : public MC602AdapterIface
{
public:
  std::string last_sensor_type;
  uint8_t last_port = 0;
  int last_value = 0;
  bool beep_called = false;
  bool led_show_called = false;
  std::vector<int64_t> key_values{1, 2, 3};

  double read_sensor(uint8_t port, const std::string & type) override
  {
    last_port = port;
    last_sensor_type = type;
    return 2.5;
  }
  void beep(int freq, float /*duration_s*/) override
  {
    beep_called = true;
    last_value = freq;
  }
  void set_led_light(uint8_t led_id, int r, int /*g*/, int /*b*/) override
  {
    last_port = led_id;
    last_value = r;
  }
  void set_led_show(const std::string & /*text*/) override
  {
    led_show_called = true;
  }
  void set_nixie(int value) override { last_value = value; }
  std::vector<int64_t> read_board_key() override { return key_values; }
  std::vector<int64_t> read_bluetooth_pad() override { return {9, 8}; }
};
}  // namespace

TEST(SystemIo, ReadSensorDispatchesToDriver)
{
  FakeDevice f;
  SystemIo io(&f);
  bool ok; std::string err; double v = -1.0;
  io.read_sensor(8, "ir", ok, err, v);
  EXPECT_TRUE(ok);
  EXPECT_EQ(v, 2.5);
  EXPECT_EQ(f.last_port, 8);
  EXPECT_EQ(f.last_sensor_type, "ir");
}

TEST(SystemIo, ReadSensorRejectsUnknownTypeWithoutTouchingBus)
{
  FakeDevice f;
  SystemIo io(&f);
  bool ok = true; std::string err; double v = -1.0;
  io.read_sensor(1, "bogus", ok, err, v);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(err.empty());
  EXPECT_TRUE(f.last_sensor_type.empty());  // 驱动未被调用
}

TEST(SystemIo, BeepLedNixieRouteToDriver)
{
  FakeDevice f;
  SystemIo io(&f);
  bool ok; std::string err;

  io.beep(1000, 0.5f, ok, err);
  EXPECT_TRUE(ok);
  EXPECT_TRUE(f.beep_called);
  EXPECT_EQ(f.last_value, 1000);

  io.set_led_light(2, 255, 0, 0, ok, err);
  EXPECT_TRUE(ok);
  EXPECT_EQ(f.last_port, 2);
  EXPECT_EQ(f.last_value, 255);

  io.set_nixie(42, ok, err);
  EXPECT_TRUE(ok);
  EXPECT_EQ(f.last_value, 42);
}

TEST(SystemIo, LedShowRoutesToDriver)
{
  FakeDevice f;
  SystemIo io(&f);
  bool ok; std::string err;
  io.set_led_show("hello", ok, err);
  EXPECT_TRUE(ok);
  EXPECT_TRUE(f.led_show_called);
}

TEST(SystemIo, ReadKeyAndPadReturnArrays)
{
  FakeDevice f;
  SystemIo io(&f);
  bool ok; std::string err; std::vector<int64_t> v;

  io.read_key(ok, err, v);
  EXPECT_TRUE(ok);
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], 1);

  io.read_pad(ok, err, v);
  EXPECT_TRUE(ok);
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], 9);
}
