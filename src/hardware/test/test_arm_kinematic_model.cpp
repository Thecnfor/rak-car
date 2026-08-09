// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Unit tests for ArmKinematicModel — parameterized serial-chain FK + analytic
// task-space IK (x / z / yaw), with fixed-plane and joint-limit semantics.

#include "hardware/arm_kinematic_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using hardware::ArmKinematicModel;
using hardware::ArmJointSpec;
using hardware::ArmLinkSpec;
using hardware::ArmToolSpec;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

TEST(ArmKinematicModelTest, FkZeroYawMapsXAndZDirectly)
{
  auto m = ArmKinematicModel::make_default();
  auto p = m.forward({0.1, 0.2, 0.0, 0.0});
  EXPECT_NEAR(p.x, 0.1, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.z, 0.2, 1e-9);
  EXPECT_NEAR(p.yaw, 0.0, 1e-9);
}

TEST(ArmKinematicModelTest, FkAccumulatesYawAboutZ)
{
  auto m = ArmKinematicModel::make_default();
  auto p = m.forward({0.1, 0.2, kPi / 2.0, 0.0});
  EXPECT_NEAR(p.x, 0.1, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.z, 0.2, 1e-9);
  EXPECT_NEAR(p.yaw, kPi / 2.0, 1e-9);
}

TEST(ArmKinematicModelTest, FkRotatesToolOffsetByYaw)
{
  // Tool extends +x by 5cm; yaw 90° swings it to +y.
  std::vector<ArmLinkSpec> links;
  links.push_back(ArmLinkSpec{
    {"arm_x", ArmJointSpec::Type::PRISMATIC, 'x', 0.0, 0.3, 0.0}, {0, 0, 0}});
  links.push_back(ArmLinkSpec{
    {"arm_z", ArmJointSpec::Type::PRISMATIC, 'z', 0.0, 0.3, 0.0}, {0, 0, 0}});
  links.push_back(ArmLinkSpec{
    {"arm_yaw", ArmJointSpec::Type::REVOLUTE, 'z', -kPi, kPi, 0.0}, {0, 0, 0}});
  ArmKinematicModel m(std::move(links), ArmToolSpec{{0.05, 0.0, -0.1}, 0.0, 0.0});

  auto p = m.forward({0.1, 0.2, kPi / 2.0});
  EXPECT_NEAR(p.x, 0.1, 1e-9);        // cos(90°)*0.05 = 0
  EXPECT_NEAR(p.y, 0.05, 1e-9);       // sin(90°)*0.05 = 0.05
  EXPECT_NEAR(p.z, 0.2 - 0.1, 1e-9);  // tool z offset is axis-aligned
  EXPECT_NEAR(p.yaw, kPi / 2.0, 1e-9);
}

TEST(ArmKinematicModelTest, FkThrowsOnSizeMismatch)
{
  auto m = ArmKinematicModel::make_default();
  EXPECT_THROW(m.forward({0.1, 0.2}), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Inverse kinematics
// ---------------------------------------------------------------------------

TEST(ArmKinematicModelTest, IkRoundTripsThroughFk)
{
  auto m = ArmKinematicModel::make_default();
  const std::vector<double> q{0.1, 0.2, 0.5, 0.0};
  const auto pose = m.forward(q);
  auto res = m.inverse_task(pose.x, pose.z, pose.yaw, q, pose.y);
  ASSERT_EQ(res.status, ArmKinematicModel::IkResult::Status::SOLVED);
  EXPECT_NEAR(res.q[0], q[0], 1e-9);
  EXPECT_NEAR(res.q[1], q[1], 1e-9);
  EXPECT_NEAR(res.q[2], q[2], 1e-9);
}

TEST(ArmKinematicModelTest, IkWithToolOffsetRoundTrips)
{
  std::vector<ArmLinkSpec> links;
  links.push_back(ArmLinkSpec{
    {"arm_x", ArmJointSpec::Type::PRISMATIC, 'x', 0.0, 0.3, 0.0}, {0, 0, 0}});
  links.push_back(ArmLinkSpec{
    {"arm_z", ArmJointSpec::Type::PRISMATIC, 'z', 0.0, 0.3, 0.0}, {0, 0, 0}});
  links.push_back(ArmLinkSpec{
    {"arm_yaw", ArmJointSpec::Type::REVOLUTE, 'z', -kPi, kPi, 0.0}, {0, 0, 0}});
  // Tool extends straight DOWN (z) from the yaw joint, so yaw only rotates
  // orientation and the tip stays on the fixed work plane (y=0).
  ArmKinematicModel m(std::move(links), ArmToolSpec{{0.0, 0.0, -0.1}, 0.0, 0.0});

  const std::vector<double> q{0.1, 0.2, -0.7};
  const auto pose = m.forward(q);
  auto res = m.inverse_task(pose.x, pose.z, pose.yaw, q);
  ASSERT_EQ(res.status, ArmKinematicModel::IkResult::Status::SOLVED);
  EXPECT_NEAR(res.q[0], q[0], 1e-9);
  EXPECT_NEAR(res.q[1], q[1], 1e-9);
  EXPECT_NEAR(res.q[2], q[2], 1e-9);
}

TEST(ArmKinematicModelTest, IkOutOfLimitsIsUnreachable)
{
  auto m = ArmKinematicModel::make_default();
  // x target beyond the 0..300mm range.
  auto res = m.inverse_task(10.0, 0.1, 0.0, {0.0, 0.0, 0.0, 0.0}, 0.0);
  EXPECT_EQ(res.status, ArmKinematicModel::IkResult::Status::UNREACHABLE);
}

TEST(ArmKinematicModelTest, IkYawOutOfLimitsIsUnreachable)
{
  auto m = ArmKinematicModel::make_default();
  // yaw ±150° → 2.618 rad; 3.0 rad is out.
  auto res = m.inverse_task(0.1, 0.1, 3.0, {0.0, 0.0, 0.0, 0.0}, 0.0);
  EXPECT_EQ(res.status, ArmKinematicModel::IkResult::Status::UNREACHABLE);
}

TEST(ArmKinematicModelTest, IkYOutsideFixedPlaneUnsupported)
{
  auto m = ArmKinematicModel::make_default();
  auto res = m.inverse_task(0.1, 0.1, 0.0, {0.0, 0.0, 0.0, 0.0}, 0.05);
  EXPECT_EQ(res.status, ArmKinematicModel::IkResult::Status::UNSUPPORTED_DIMENSION);
}

TEST(ArmKinematicModelTest, IkYWithinToleranceAccepted)
{
  std::vector<ArmLinkSpec> links;
  links.push_back(ArmLinkSpec{
    {"arm_x", ArmJointSpec::Type::PRISMATIC, 'x', 0.0, 0.3, 0.0}, {0, 0, 0}});
  links.push_back(ArmLinkSpec{
    {"arm_z", ArmJointSpec::Type::PRISMATIC, 'z', 0.0, 0.3, 0.0}, {0, 0, 0}});
  links.push_back(ArmLinkSpec{
    {"arm_yaw", ArmJointSpec::Type::REVOLUTE, 'z', -kPi, kPi, 0.0}, {0, 0, 0}});
  ArmKinematicModel m(std::move(links), ArmToolSpec{{0.0, 0.0, 0.0}, 0.0, 0.01});

  // Model y = 0 (no y-offset); request y=0.005 within 0.01 tolerance.
  auto res = m.inverse_task(0.1, 0.2, 0.3, {0.05, 0.1, 0.2}, 0.005);
  ASSERT_EQ(res.status, ArmKinematicModel::IkResult::Status::SOLVED);
  EXPECT_NEAR(m.forward(res.q).y, 0.0, 1e-9);
}

TEST(ArmKinematicModelTest, IkInvalidInputOnBadQSize)
{
  auto m = ArmKinematicModel::make_default();
  auto res = m.inverse_task(0.1, 0.1, 0.0, {0.0, 0.0}, 0.0);
  EXPECT_EQ(res.status, ArmKinematicModel::IkResult::Status::INVALID_INPUT);
}

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------

TEST(ArmKinematicModelTest, WithinLimitsBoundary)
{
  auto m = ArmKinematicModel::make_default();
  EXPECT_TRUE(m.within_limits({0.0, 0.0, 0.0, 0.0}));
  EXPECT_TRUE(m.within_limits({0.300, 0.300, 0.0, 0.0}));
  EXPECT_FALSE(m.within_limits({0.301, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(m.within_limits({0.0, -0.001, 0.0, 0.0}));
  EXPECT_FALSE(m.within_limits({0.0, 0.0, 0.0}));  // size mismatch
}
