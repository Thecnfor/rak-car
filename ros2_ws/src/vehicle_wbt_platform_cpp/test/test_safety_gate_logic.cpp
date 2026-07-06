// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for safety_gate_logic — the pure 4-layer safety gate.
//
// Tests cover each layer in isolation AND the priority order:
//   1. Physical estop: always wins (zero twist, publish stop)
//   2. Mode: AUTO drops everything
//   3. Rate limit: clamps to max_linear / max_angular (doesn't drop)
//   4. Heartbeat: tracked by the rclcpp::Node wrapper, not this fn
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §安全设计

#include "vehicle_wbt_platform_cpp/safety_gate_logic.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace vwpc = vehicle_wbt_platform_cpp;
using vwpc::GateDecision;
using vwpc::GateInput;
using vwpc::GateMode;
using vwpc::apply_safety_gate;

namespace
{
constexpr double kMaxLinear = 0.3;   // m/s
constexpr double kMaxAngular = 0.5;  // rad/s
}

// =================================================================
// Layer 1: Physical estop — always wins
// =================================================================

TEST(SafetyGateLogicTest, EstopZeroesTwistAndPublishes)
{
  GateInput in{GateMode::MANUAL, true, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.5, 0.5, 0.5);
  EXPECT_TRUE(d.publish);  // must publish the stop
  EXPECT_EQ(d.linear_x, 0.0);
  EXPECT_EQ(d.linear_y, 0.0);
  EXPECT_EQ(d.angular_z, 0.0);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::ESTOP));
}

TEST(SafetyGateLogicTest, EstopWinsOverMode)
{
  // estop should publish even in AUTO mode (safety critical)
  GateInput in{GateMode::AUTO, true, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.5, 0.5, 0.5);
  EXPECT_TRUE(d.publish);
  EXPECT_EQ(d.linear_x, 0.0);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::ESTOP));
}

TEST(SafetyGateLogicTest, EstopWinsOverRateLimit)
{
  // estop zeroes twist regardless of input velocity
  GateInput in{GateMode::MANUAL, true, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 1.0, 1.0, 1.0);  // would normally be clamped
  EXPECT_EQ(d.linear_x, 0.0);
  EXPECT_EQ(d.angular_z, 0.0);
}

// =================================================================
// Layer 2: Mode gate — AUTO drops everything (except estop)
// =================================================================

TEST(SafetyGateLogicTest, AutoModeDropsCommand)
{
  GateInput in{GateMode::AUTO, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.1, 0.0, 0.0);
  EXPECT_FALSE(d.publish);
  EXPECT_EQ(d.linear_x, 0.0);  // not echoed back
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::MODE_DROPPED));
}

TEST(SafetyGateLogicTest, ManualModePassesThrough)
{
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.1, 0.0, 0.0);
  EXPECT_TRUE(d.publish);
  EXPECT_DOUBLE_EQ(d.linear_x, 0.1);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::PASS_THROUGH));
}

TEST(SafetyGateLogicTest, DevTestModePassesThrough)
{
  GateInput in{GateMode::DEV_TEST, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.2, 0.1, 0.05);
  EXPECT_TRUE(d.publish);
  EXPECT_DOUBLE_EQ(d.linear_x, 0.2);
  EXPECT_DOUBLE_EQ(d.linear_y, 0.1);
  EXPECT_DOUBLE_EQ(d.angular_z, 0.05);
}

TEST(SafetyGateLogicTest, EStopModePassesThroughIfNotPressed)
{
  // E_STOP mode by itself doesn't block; only the physical button does
  GateInput in{GateMode::E_STOP, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.1, 0.0, 0.0);
  EXPECT_TRUE(d.publish);
  EXPECT_DOUBLE_EQ(d.linear_x, 0.1);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::PASS_THROUGH));
}

// =================================================================
// Layer 3: Rate limit — clamps but doesn't drop
// =================================================================

TEST(SafetyGateLogicTest, RateLimitClampsLinearX)
{
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 5.0, 0.0, 0.0);  // 5.0 m/s >> 0.3
  EXPECT_TRUE(d.publish);
  EXPECT_DOUBLE_EQ(d.linear_x, kMaxLinear);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}

TEST(SafetyGateLogicTest, RateLimitClampsLinearXNegative)
{
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, -5.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(d.linear_x, -kMaxLinear);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}

TEST(SafetyGateLogicTest, RateLimitClampsAngularZ)
{
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.0, 0.0, 2.0);  // 2.0 rad/s >> 0.5
  EXPECT_DOUBLE_EQ(d.angular_z, kMaxAngular);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}

TEST(SafetyGateLogicTest, RateLimitClampsBoth)
{
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 5.0, 5.0, 2.0);
  EXPECT_DOUBLE_EQ(d.linear_x, kMaxLinear);
  EXPECT_DOUBLE_EQ(d.linear_y, kMaxLinear);
  EXPECT_DOUBLE_EQ(d.angular_z, kMaxAngular);
}

TEST(SafetyGateLogicTest, RateLimitExactBoundary)
{
  // Exactly at the limit should NOT be flagged as limited
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, kMaxLinear, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(d.linear_x, kMaxLinear);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::PASS_THROUGH));
  EXPECT_FALSE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}

TEST(SafetyGateLogicTest, RateLimitCustomThreshold)
{
  // Test that a custom max_linear is respected
  GateInput in{GateMode::DEV_TEST, false, 1.0, 2.0};
  auto d = apply_safety_gate(in, 1.5, 0.0, 3.0);
  EXPECT_DOUBLE_EQ(d.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(d.angular_z, 2.0);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}

TEST(SafetyGateLogicTest, RateLimitZeroVelocityPassesThrough)
{
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.0, 0.0, 0.0);
  EXPECT_TRUE(d.publish);
  EXPECT_DOUBLE_EQ(d.linear_x, 0.0);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::PASS_THROUGH));
}

// =================================================================
// Combined scenarios — verify layer priority
// =================================================================

TEST(SafetyGateLogicTest, EstopDoesNotHaveModeDropped)
{
  // estop and mode_dropped are mutually exclusive in the bitfield
  GateInput in{GateMode::AUTO, true, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 0.5, 0.5, 0.5);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::ESTOP));
  EXPECT_FALSE(static_cast<bool>(d.reason & GateDecision::Reason::MODE_DROPPED));
  EXPECT_FALSE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}

TEST(SafetyGateLogicTest, ManualHighSpeedIsLimitedNotDropped)
{
  // Manual mode + over-speed: clamp to max, don't drop
  GateInput in{GateMode::MANUAL, false, kMaxLinear, kMaxAngular};
  auto d = apply_safety_gate(in, 10.0, 10.0, 10.0);
  EXPECT_TRUE(d.publish);
  EXPECT_DOUBLE_EQ(d.linear_x, kMaxLinear);
  EXPECT_DOUBLE_EQ(d.angular_z, kMaxAngular);
  EXPECT_TRUE(static_cast<bool>(d.reason & GateDecision::Reason::RATE_LIMITED));
}
