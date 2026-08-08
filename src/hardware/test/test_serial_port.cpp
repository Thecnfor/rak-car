// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Unit tests for SerialPort (the raw byte I/O layer).
//
// Protocol/golden-frame tests live in test_mc602_protocol; driver behavior
// in test_mc602_adapter. This file covers SerialPort alone: the injection
// seam, frame pass-through, and not-open error handling.

#include <gmock/gmock.h>

#include <cstdint>
#include <vector>

#include "hardware/serial_port.hpp"

namespace vw = hardware;

class SerialPortTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

// set_response_handler replaces real I/O with the handler's output
TEST_F(SerialPortTest, SetResponseHandler_BypassesRealIO)
{
  vw::SerialPort port("/dev/ttyUSB0", 1000000);

  // Canned response frame (full frame, header + len + payload + footer).
  std::vector<uint8_t> canned = {
    0x77, 0x68,    // header
    0x08,          // length = 8
    0x07,          // dev_id = SENSOR_MULTI
    0x01,          // mode = GET
    0x02,          // port 2
    0x34, 0x12,    // uint16 LE = 0x1234
    0x0A           // footer
  };

  port.set_response_handler([canned](const std::vector<uint8_t> &) {
    return canned;
  });

  // SerialPort::exchange() returns the FULL frame (no stripping).
  auto result = port.exchange({0x77, 0x68, 0x05, 0x07, 0x01, 0x02, 0x01, 0x0A});

  ASSERT_EQ(result.size(), 9u);
  EXPECT_EQ(result[0], 0x77u);
  EXPECT_EQ(result[1], 0x68u);
  EXPECT_EQ(result[2], 0x08u);
  EXPECT_EQ(result[3], 0x07u);
  EXPECT_EQ(result[4], 0x01u);
  EXPECT_EQ(result[5], 0x02u);
  EXPECT_EQ(result[6], 0x34u);
  EXPECT_EQ(result[7], 0x12u);
  EXPECT_EQ(result.back(), 0x0Au);
}

// exchange without a handler throws if port is not open (no fd)
TEST_F(SerialPortTest, Exchange_ThrowsWhenNotOpen)
{
  vw::SerialPort port("/dev/nonexistent", 1000000);
  EXPECT_THROW(port.exchange({0x77}), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
