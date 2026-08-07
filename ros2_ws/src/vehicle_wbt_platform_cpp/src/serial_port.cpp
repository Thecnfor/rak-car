// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SerialPort — POSIX termios serial port wrapper.

#include "vehicle_wbt_platform_cpp/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#if defined(__linux__)
#include <sys/select.h>
#endif

namespace vehicle_wbt_platform_cpp
{

namespace
{

// Map baud rate integer to POSIX speed constant.
speed_t baud_to_speed(uint32_t baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 380400: return B38400;  // closest standard; actual 380400 set via ioctl on some platforms
    case 1000000: return B1000000;
    default:
      throw std::runtime_error("SerialPort: unsupported baud rate " + std::to_string(baud));
  }
}

}  // anonymous namespace

SerialPort::SerialPort(std::string device, uint32_t baud)
: device_(std::move(device)), baud_(baud), fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_open()) {
    return true;
  }

  // O_RDWR | O_NOCTTY | O_NONBLOCK
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("SerialPort::open '" + device_ + "': " + std::strerror(errno));
  }

  if (!configure_port_()) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("SerialPort::configure_port '" + device_ + "': " + std::strerror(errno));
  }

  return true;
}

void SerialPort::close()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::vector<uint8_t> SerialPort::exchange(
  const std::vector<uint8_t> & tx_data,
  std::chrono::milliseconds timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (response_handler_) {
    // Test mode: bypass real I/O.
    return response_handler_(tx_data);
  }

  if (!is_open()) {
    throw std::runtime_error("SerialPort::exchange: port not open");
  }

  // 1. Write all bytes.
  size_t written = 0;
  while (written < tx_data.size()) {
    ssize_t n = ::write(fd_, tx_data.data() + written, tx_data.size() - written);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      throw std::runtime_error("SerialPort::write: " + std::string(std::strerror(errno)));
    }
    written += static_cast<size_t>(n);
  }

  // 2. Read response until footer byte 0x0A.
  std::vector<uint8_t> response;
  const uint8_t FOOTER = 0x0A;
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    uint8_t byte;
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      break;
    }

    ssize_t n = timed_read(&byte, 1, std::min(remaining, std::chrono::milliseconds(50)));
    if (n > 0) {
      response.push_back(byte);
      if (byte == FOOTER) {
        break;  // frame complete
      }
    }
    // n == 0 means timeout on this read iteration; loop to check deadline
  }

  if (response.empty() || response.back() != FOOTER) {
    throw std::runtime_error(
      "SerialPort::exchange: timeout after " +
      std::to_string(timeout.count()) + "ms, got " +
      std::to_string(response.size()) + " bytes");
  }

  return response;
}

ssize_t SerialPort::timed_read(uint8_t * buf, size_t len,
                                std::chrono::milliseconds timeout)
{
  // Use select() for timeout on the fd.
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_, &read_fds);

  struct timeval tv;
  tv.tv_sec = timeout.count() / 1000;
  tv.tv_usec = (timeout.count() % 1000) * 1000;

  int rc = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (rc <= 0) {
    return 0;  // timeout or error
  }

  ssize_t n = ::read(fd_, buf, len);
  if (n < 0 && errno != EAGAIN && errno != EINTR) {
    throw std::runtime_error("SerialPort::read: " + std::string(std::strerror(errno)));
  }
  return n;
}

bool SerialPort::configure_port_()
{
  struct termios tio;
  if (tcgetattr(fd_, &tio) != 0) {
    return false;
  }

  // Raw mode: 8N1, no processing, no flow control.
  cfmakeraw(&tio);

  // Baud rate.
  speed_t speed = baud_to_speed(baud_);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  // VMIN=0, VTIME=10 (100ms deciseconds) — non-blocking reads with timeout.
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 10;

  // No flow control.
  tio.c_cflag &= ~CRTSCTS;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Enable receiver, ignore modem control lines.
  tio.c_cflag |= CREAD | CLOCAL;

  // 8 data bits.
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  // No parity.
  tio.c_cflag &= ~PARENB;

  // 1 stop bit.
  tio.c_cflag &= ~CSTOPB;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    return false;
  }

  // Flush any pending data.
  tcdrain(fd_);
  tcflush(fd_, TCIOFLUSH);

  return true;
}

}  // namespace vehicle_wbt_platform_cpp
