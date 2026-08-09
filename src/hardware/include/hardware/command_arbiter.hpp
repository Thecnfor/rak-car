// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// CommandArbiter — resource mutual exclusion for long robot actions.
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §8
//
// The MC602 single bus is shared; the SerialScheduler already serializes
// frame traffic. This arbiter serializes SEMANTIC control: a chassis navigate
// and an arm trajectory are long actions that must not run concurrently, or
// the robot fights itself.
//
// Rules:
//   * CHASSIS and ARM are LONG resources — mutually exclusive.
//   * PUMP follows ARM and PERIPHERAL is cosmetic (beep/led/nixie): both are
//     SHORT and never block on a long action.
//   * URGENT always wins: it preempts the current long holder.
//
// Pure C++ (no rclcpp) — unit-testable in isolation.

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace hardware
{

enum class ControlResource : uint8_t
{
  CHASSIS = 0,  // long: navigate / drive
  ARM = 1,      // long: trajectory / cartesian move
  PUMP = 2,     // short: follows the arm action
  PERIPHERAL = 3,  // short: beep / led / nixie / dout (cosmetic)
};

enum class ControlPriority : uint8_t
{
  NORMAL = 0,
  URGENT = 1,  // e-stop / safety: always wins
};

struct ArbiterResult
{
  enum class Status : uint8_t
  {
    GRANTED = 0,
    BUSY = 1,     // another long action holds the resource
    REJECTED = 2, // invalid resource / misuse
  };
  Status status = Status::REJECTED;
  std::string reason;
};

class CommandArbiter
{
public:
  // Request the resource. Long actions (CHASSIS / ARM) are exclusive; URGENT
  // preempts the current holder. Short resources are always granted.
  ArbiterResult acquire(ControlResource res, ControlPriority prio = ControlPriority::NORMAL);

  // Release the resource (idempotent; only the holder's release takes effect).
  void release(ControlResource res);

  // Which long resource currently holds the bus, if any.
  std::optional<ControlResource> holder() const { return holder_; }

private:
  bool is_long(ControlResource res) const
  {
    return res == ControlResource::CHASSIS || res == ControlResource::ARM;
  }

  std::optional<ControlResource> holder_;
};

}  // namespace hardware
