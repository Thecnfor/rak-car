// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// mc602_bootloader — see header.

#include "hardware/mc602_bootloader.hpp"

#include <array>
#include <chrono>
#include <thread>
#include <vector>

namespace hardware
{

namespace
{

using namespace std::chrono_literals;

// Bootloader protocol (55 AA ... checksum). Checksum = ~sum(first n-1) & 0xFF.
constexpr std::array<uint8_t, 8> kBootPing = {0x55, 0xAA, 0x00, 0x01, 0x08, 0x00, 0x00, 0xF7};
// Program-mode ping (77 68 ...).
constexpr std::array<uint8_t, 8> kProgramPing = {0x77, 0x68, 0x07, 0x02, 0x01, 0x10, 0x0A};

uint8_t boot_checksum(const std::vector<uint8_t> & frame)
{
  uint32_t sum = 0;
  for (size_t i = 0; i + 1 < frame.size(); ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>((~sum) & 0xFF);
}

std::vector<uint8_t> runcode_frame(uint32_t addr)
{
  std::vector<uint8_t> f = {
    0x55, 0xAA, 0x00, 0x40, 0x0B, 0x00,
    static_cast<uint8_t>(addr & 0xFF),
    static_cast<uint8_t>((addr >> 8) & 0xFF),
    static_cast<uint8_t>((addr >> 16) & 0xFF),
    static_cast<uint8_t>((addr >> 24) & 0xFF),
    0,
  };
  f[10] = boot_checksum(f);
  return f;
}

constexpr std::chrono::milliseconds kByteDelay{1};     // slow-write gap
constexpr std::chrono::milliseconds kAckTimeout{180};  // bootloader ack window

}  // namespace

bool probe_program(SerialPort & port, std::chrono::milliseconds timeout)
{
  try {
    auto resp = port.exchange(std::vector<uint8_t>(kProgramPing.begin(), kProgramPing.end()),
                              timeout);
    return resp.size() >= 6 && resp[0] == 0x77 && resp[1] == 0x68 && resp.back() == 0x0A;
  } catch (...) {
    return false;
  }
}

BootloaderResult recover_to_program(
  SerialPort & port, std::vector<uint32_t> slots)
{
  // 1) Already in program mode?
  if (probe_program(port)) {
    return {BootloaderResult::State::PROGRAM, 0};
  }

  // 2) Wake the bootloader with a slow byte-by-byte ping.
  bool boot_online = false;
  for (int attempt = 0; attempt < 6; ++attempt) {
    port.flush();
    port.write_raw(std::vector<uint8_t>(kBootPing.begin(), kBootPing.end()), kByteDelay);
    std::this_thread::sleep_for(10ms);
    auto ack = port.read_raw(10, kAckTimeout);
    if (ack.size() >= 2 && ack[0] == 0x66 && ack[1] == 0xBB) {
      boot_online = true;
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  if (!boot_online) {
    return {BootloaderResult::State::NO_CONTROLLER, 0};
  }

  // 3) RUNCODE each candidate slot until the app answers the program ping.
  for (uint32_t slot : slots) {
    for (int attempt = 0; attempt < 4; ++attempt) {
      port.flush();
      auto frame = runcode_frame(slot);
      port.write_raw(frame, kByteDelay);
      std::this_thread::sleep_for(50ms);
      port.read_raw(11, kAckTimeout);  // ack (optional); ignore content
      // Give the app a beat, then ask it directly.
      std::this_thread::sleep_for(600ms);
      if (probe_program(port)) {
        return {BootloaderResult::State::RECOVERED, slot};
      }
      std::this_thread::sleep_for(150ms);
    }
  }
  return {BootloaderResult::State::BOOTLOADER, 0};
}

}  // namespace hardware
