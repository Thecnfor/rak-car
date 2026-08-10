// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for MC602Adapter (driver) — constructor contract, open/close
// idempotence, port metadata, and end-to-end typed operations via the
// injection seam (no hardware). Wire-level correctness is covered by
// test_mc602_protocol's golden frames.

#include "hardware/mc602_adapter.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

using hardware::MC602Adapter;

namespace
{

void enable_injection(MC602Adapter & a,
                      std::vector<uint8_t> (*responder)(const std::vector<uint8_t> &))
{
  a.set_injection(responder);
}

std::vector<uint8_t> no_response(const std::vector<uint8_t> &) { return {}; }

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
  enable_injection(a, no_response);
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

TEST(MC602AdapterTest, UnsupportedActuatorTypeThrows)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  enable_injection(a, no_response);
  a.open();
  EXPECT_THROW(a.write_actuator(1, "no_such_actuator", 0.0), std::runtime_error);
}

TEST(MC602AdapterTest, IrReadScalesMillimetersToMeters)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  a.set_injection([](const std::vector<uint8_t> &) {
    return std::vector<uint8_t>{0x07, 0x01, 0x08, 0x88, 0x13};  // H = 0x1388 = 5000 mm
  });
  a.open();
  EXPECT_FLOAT_EQ(a.read_ir(8), 5.0f);
}

TEST(MC602AdapterTest, BurstPacksWritesThenCommits)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  std::vector<std::vector<uint8_t>> frames;
  a.set_injection([&](const std::vector<uint8_t> & frame) {
    frames.push_back(frame);
    return std::vector<uint8_t>{};
  });
  a.open();

  a.begin_burst();
  a.set_dout(4, 1);   // buffered
  a.set_motor(6, 50); // buffered
  a.commit_burst();   // sent as one burst

  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0], std::vector<uint8_t>({0x77, 0x68, 0x08, 0x10, 0x02, 0x04, 0x01, 0x0A}));
  EXPECT_EQ(frames[1], std::vector<uint8_t>({0x77, 0x68, 0x08, 0x02, 0x02, 0x06, 0x32, 0x0A}));
}

TEST(MC602AdapterTest, MpsToVirtualClamps)
{
  MC602Adapter a("/dev/ttyUSB0", 1000000);
  // 0.5 m/s on a 0.03 m radius wheel → high virtual value, clamps to 100.
  EXPECT_EQ(a.mps_to_virtual(5.0, 0.03), 100);
  EXPECT_EQ(a.mps_to_virtual(0.0, 0.03), 0);
  EXPECT_EQ(a.mps_to_virtual(-5.0, 0.03), -100);
}
