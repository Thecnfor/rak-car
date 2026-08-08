// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// mc602_bootloader — MC602 bootloader ↔ program mode handshake.
//
// Ported from main-branch runtime/hardware/controller_recover.py (which kept
// the robot alive across reboots). The MC602 boots into a bootloader that
// speaks its own 55 AA protocol; the robot's app firmware lives at a flash
// slot (RunA = 0x08060000, the official flash target) and must be launched
// with a RUNCODE command. Until then, 77 68 program frames are ignored.
//
// Sequence (all at 1 Mbaud, bootloader frames written byte-by-byte ~1ms apart):
//   1. probe program mode (77 68 ping) — if it answers, nothing to do.
//   2. boot_ping (55 AA ...) — bootloader answers with 66 BB.
//   3. RUNCODE @ flash slot (55 AA ... 0x40 ...) — jump to the app.
//   4. probe program mode again — done when it answers.
//
// Pure C++ / rclcpp-free; works over a SerialPort-like byte interface so it
// is testable and reusable by the bridge node at startup.

#pragma once

#include "hardware/serial_port.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace hardware
{

// Default app slot — RunA, the official Scratch_Download_MC602P target.
inline constexpr uint32_t MC602_RUN_SLOT_RUNA = 0x08060000;
// Main-branch recover also knew this older slot.
inline constexpr uint32_t MC602_RUN_SLOT_LEGACY = 0x0800D000;

struct BootloaderResult
{
  enum class State
  {
    PROGRAM,     // already in program mode
    RECOVERED,   // was bootloader, RUNCODE launched the app
    BOOTLOADER,  // bootloader alive but RUNCODE never confirmed the app
    NO_CONTROLLER,  // neither program nor bootloader responded
  };
  State state;
  uint32_t launched_slot = 0;
};

// Bring the controller to program mode, trying the given flash slots in order
// (first that yields a program ping wins). Returns the outcome.
BootloaderResult recover_to_program(
  SerialPort & port,
  std::vector<uint32_t> slots = {MC602_RUN_SLOT_RUNA, MC602_RUN_SLOT_LEGACY});

// True if the controller answers the program-mode ping (77 68).
bool probe_program(SerialPort & port,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds(200));

}  // namespace hardware
