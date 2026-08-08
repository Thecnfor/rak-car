// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// DirectSerialTransport — local POSIX fd transport (thin SerialPort wrapper).

#include "hardware/serial_transport.hpp"

#include <stdexcept>
#include <utility>

namespace hardware
{

DirectSerialTransport::DirectSerialTransport(std::string device, uint32_t baud)
: serial_(device, baud), device_(std::move(device)), baud_(baud)
{
}

DirectSerialTransport::~DirectSerialTransport()
{
  close();
}

void DirectSerialTransport::open()
{
  serial_.open();
}

void DirectSerialTransport::close()
{
  serial_.close();
}

bool DirectSerialTransport::is_open() const
{
  return serial_.is_open();
}

std::string DirectSerialTransport::serial_port() const
{
  return device_;
}

uint32_t DirectSerialTransport::baud() const
{
  return baud_;
}

std::vector<uint8_t> DirectSerialTransport::exchange(
  const std::vector<uint8_t> & frame, const ExchangeOpts & /*opts*/)
{
  // SerialPort::exchange returns the full response frame (reads until footer).
  // No framing knowledge here — the frame passes through untouched.
  return serial_.exchange(frame);
}

}  // namespace hardware
