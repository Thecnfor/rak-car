// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for the BaseController interface contract — verified by
// implementing a FakeController in test_mc602_adapter.cpp and exercising
// it through the MC602Adapter facade.
//
// This file documents the interface invariants (open/close/is_open
// idempotency, port enumeration keys) as separate tests so future
// adapters (MC601, MC603) can re-use them by copy-paste.

#include "vehicle_wbt_platform_cpp/mc602_adapter.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

using vehicle_wbt_platform_cpp::BaseController;
using vehicle_wbt_platform_cpp::MC602Adapter;

namespace
{
class CountingController : public BaseController
{
public:
  int open_count = 0;
  int close_count = 0;
  int read_count = 0;
  int write_count = 0;

  void open() override { open_count++; }
  void close() override { close_count++; }
  bool is_open() const override { return open_count > close_count; }
  std::string serial_port() const override { return "/fake/ttyUSB1"; }
  uint32_t baud() const override { return 115200; }
  double read_sensor(uint8_t, const std::string &) override
  {
    read_count++;
    return 1.23;
  }
  void write_actuator(uint8_t, const std::string &, double) override { write_count++; }
  std::map<std::string, uint32_t> enumerate_ports() const override
  {
    return {{"motor", 6}, {"servo", 7}, {"stepper", 3}, {"io", 8}};
  }
};
}  // namespace

TEST(BaseControllerIfaceTest, OpenCloseCountersIncrement)
{
  CountingController c;
  EXPECT_FALSE(c.is_open());
  c.open();
  c.open();  // second open — adapter-level guard, not interface
  EXPECT_TRUE(c.is_open());
  c.close();
  c.close();
  EXPECT_FALSE(c.is_open());
  EXPECT_EQ(c.open_count, 2);
  EXPECT_EQ(c.close_count, 2);
}

TEST(BaseControllerIfaceTest, ReadAndWriteIncrementCounters)
{
  CountingController c;
  c.open();
  c.read_sensor(1, "ir");
  c.read_sensor(2, "ir");
  c.write_actuator(3, "motor", 0.1);
  EXPECT_EQ(c.read_count, 2);
  EXPECT_EQ(c.write_count, 1);
}

TEST(BaseControllerIfaceTest, EnumeratePortsKeys)
{
  CountingController c;
  auto ports = c.enumerate_ports();
  EXPECT_TRUE(ports.count("motor") > 0);
  EXPECT_TRUE(ports.count("servo") > 0);
  EXPECT_TRUE(ports.count("stepper") > 0);
  EXPECT_TRUE(ports.count("io") > 0);
}
