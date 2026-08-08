// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SerialPort — POSIX termios serial port wrapper for MC602 communication.
//
// This class is independent of ROS2 (no rclcpp). It wraps open()/close()/
// write()/read() with proper termios configuration and thread-safe exchange().
//
// The MC602 protocol uses:
//   - 8N1 (8 data bits, no parity, 1 stop bit)
//   - Header: 0x77 0x68, Footer: 0x0A
//   - No software flow control, no hardware flow control
//   - VMIN=0, VTIME=10 (100ms read timeout)

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace hardware
{

class SerialPort
{
public:
  using ResponseHandler =
    std::function<std::vector<uint8_t>(const std::vector<uint8_t> &)>;

  // baud: 380400 (MC601), 1000000 (MC602 USB), 115200 (MC602 wireless)
  explicit SerialPort(std::string device, uint32_t baud);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Open the serial port and configure termios. Returns true on success.
  // Throws std::runtime_error on failure.
  bool open();

  // Close the serial port. Safe to call multiple times.
  void close();

  // True if the port is open and ready for I/O.
  bool is_open() const { return fd_ >= 0; }

  // Thread-safe request-response exchange:
  //   1. Write tx_data to the serial port
  //   2. Block-read until footer byte (0x0A) or timeout
  //   3. Return raw response bytes
  // Thread-safe: multiple threads can call exchange() concurrently;
  // calls are serialized by the internal mutex.
  std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & tx_data,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

  // For testing: inject a canned response instead of reading from the real fd.
  // When set, exchange() ignores the real fd and returns the handler's output.
  void set_response_handler(ResponseHandler handler)
  {
    response_handler_ = std::move(handler);
  }

private:
  bool configure_port_();
  ssize_t timed_read(uint8_t * buf, size_t len, std::chrono::milliseconds timeout);

  std::string device_;
  uint32_t baud_;
  int fd_;  // -1 when closed
  mutable std::mutex mutex_;
  ResponseHandler response_handler_;
};

}  // namespace hardware
