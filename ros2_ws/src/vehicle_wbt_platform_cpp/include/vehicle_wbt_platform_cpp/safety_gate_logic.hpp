// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// safety_gate_logic — pure 4-layer safety gate logic, decoupled from rclcpp.
//
// The rclcpp::Node wrapper (src/safety_gate_node.cpp) calls apply_safety_gate()
// to decide whether a /cmd/vel_raw passes through. The function is pure
// (no rclcpp types in the signature) so it can be unit-tested with gtest
// without spinning up a ROS2 runtime.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §安全设计
//
// 4 layers (evaluated in order):
//   1. Physical estop: if pressed, output zero twist + always publish stop
//   2. Mode: if AUTO, drop the message (don't publish)
//   3. Rate limit: clamp linear/angular velocity to max_linear/max_angular
//   4. Heartbeat: caller checks last_heartbeat; this fn doesn't track time

#pragma once

#include <cstdint>
#include <optional>

namespace vehicle_wbt_platform_cpp
{

// Mode state for the gate. Mirrors the rclcpp node's Mode enum.
enum class GateMode : int { AUTO = 0, MANUAL = 1, DEV_TEST = 2, E_STOP = 3 };

// Input parameters for the gate (passed by value for testability).
struct GateInput
{
  GateMode mode{GateMode::AUTO};
  bool estop_pressed{false};
  double max_linear{0.3};    // m/s
  double max_angular{0.5};   // rad/s
};

// Decision returned by apply_safety_gate.
//   publish = false  : drop the message (e.g. AUTO mode)
//   publish = true   : publish the cmd_out (which may be a zero twist on estop)
struct GateDecision
{
  bool publish{false};
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  // Reason for the decision (for logging). Bitfield so multiple apply.
  enum class Reason : uint8_t {
    NONE = 0,
    ESTOP = 1 << 0,         // physical estop active
    MODE_DROPPED = 1 << 1,  // AUTO mode dropped the message
    RATE_LIMITED = 1 << 2,   // velocity was clamped
    PASS_THROUGH = 1 << 3,  // message passed unchanged
  } reason{Reason::NONE};
};

// Bitwise operators for Reason bitfield
inline GateDecision::Reason operator&(GateDecision::Reason a, GateDecision::Reason b)
{
  return static_cast<GateDecision::Reason>(
    static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
inline GateDecision::Reason & operator&=(GateDecision::Reason & a, GateDecision::Reason b)
{
  a = a & b;
  return a;
}

// Apply the 4-layer safety gate. Pure function, no side effects.
//
// in_cmd:  linear.x, linear.y, angular.z are the requested velocities
// Returns: GateDecision with publish=true if the caller should
//   publish (possibly zeroed) on /cmd/vel_safe, publish=false if drop.
inline GateDecision apply_safety_gate(const GateInput & in, double in_cmd_x,
                                     double in_cmd_y, double in_cmd_z)
{
  GateDecision d;

  // Layer 4 (highest priority): physical estop
  if (in.estop_pressed) {
    d.publish = true;  // always publish a stop
    d.reason = GateDecision::Reason::ESTOP;
    return d;  // cmd already zero-initialized
  }

  // Layer 1: mode gate — AUTO drops everything
  if (in.mode == GateMode::AUTO) {
    d.publish = false;
    d.reason = GateDecision::Reason::MODE_DROPPED;
    return d;
  }

  // Layer 2: rate limit (clamp, don't drop)
  double vx = in_cmd_x;
  double vy = in_cmd_y;
  double wz = in_cmd_z;
  bool limited = false;
  if (vx > in.max_linear) { vx = in.max_linear; limited = true; }
  else if (vx < -in.max_linear) { vx = -in.max_linear; limited = true; }
  if (vy > in.max_linear) { vy = in.max_linear; limited = true; }
  else if (vy < -in.max_linear) { vy = -in.max_linear; limited = true; }
  if (wz > in.max_angular) { wz = in.max_angular; limited = true; }
  else if (wz < -in.max_angular) { wz = -in.max_angular; limited = true; }

  d.linear_x = vx;
  d.linear_y = vy;
  d.angular_z = wz;
  d.publish = true;
  d.reason = limited ? GateDecision::Reason::RATE_LIMITED
                     : GateDecision::Reason::PASS_THROUGH;
  return d;
}

}  // namespace vehicle_wbt_platform_cpp
