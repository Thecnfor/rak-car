// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Unit tests for ArmCartesianPlanner — goal → joint-space plan + rejection.

#include "hardware/arm_cartesian_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using hardware::ArmCartesianPlanner;
using hardware::ArmKinematicModel;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(ArmCartesianPlannerTest, PlansReachableGoalInMmAndDeg)
{
  auto p = ArmCartesianPlanner::make_default();
  // x=100mm z=50mm yaw=0 → q_x=0.1 q_z=0.05 q_yaw=0.
  auto plan = p.plan("arm_base", 100.0, 50.0, 0.0, false, 0.0, 0, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::OK);
  ASSERT_EQ(plan.q_target.size(), 4u);
  EXPECT_NEAR(plan.q_target[0], 0.1, 1e-9);
  EXPECT_NEAR(plan.q_target[1], 0.05, 1e-9);
  EXPECT_NEAR(plan.q_target[2], 0.0, 1e-9);
  EXPECT_GT(plan.duration_sec, 0.0);
}

TEST(ArmCartesianPlannerTest, GripActionSetsGripJoint)
{
  auto p = ArmCartesianPlanner::make_default();
  auto grip = p.plan("arm_base", 100.0, 50.0, 0.0, false, 0.0, 1, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(grip.status, ArmCartesianPlanner::Plan::Status::OK);
  EXPECT_NEAR(grip.q_target[3], 90.0 * kPi / 180.0, 1e-9);

  auto release = p.plan("arm_base", 100.0, 50.0, 0.0, false, 0.0, 2, 1.0, 0.0,
                        {0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(release.status, ArmCartesianPlanner::Plan::Status::OK);
  EXPECT_NEAR(release.q_target[3], -90.0 * kPi / 180.0, 1e-9);
}

TEST(ArmCartesianPlannerTest, RejectsNonArmBaseFrame)
{
  auto p = ArmCartesianPlanner::make_default();
  auto plan = p.plan("base_link", 100.0, 50.0, 0.0, false, 0.0, 0, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::UNSUPPORTED_DIMENSION);
}

TEST(ArmCartesianPlannerTest, RejectsUnreachableGoal)
{
  auto p = ArmCartesianPlanner::make_default();
  // x=10m >> 300mm limit.
  auto plan = p.plan("arm_base", 10000.0, 50.0, 0.0, false, 0.0, 0, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::UNREACHABLE);
}

TEST(ArmCartesianPlannerTest, RejectsYawOutOfRange)
{
  auto p = ArmCartesianPlanner::make_default();
  auto plan = p.plan("arm_base", 100.0, 50.0, 300.0, false, 0.0, 0, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::UNREACHABLE);
}

TEST(ArmCartesianPlannerTest, RejectsYOutsideFixedPlane)
{
  auto p = ArmCartesianPlanner::make_default();
  // y_enabled + y=50mm → fixed plane (y_nominal=0) mismatch.
  auto plan = p.plan("arm_base", 100.0, 50.0, 0.0, true, 50.0, 0, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::UNSUPPORTED_DIMENSION);
}

TEST(ArmCartesianPlannerTest, RejectsPumpUntilWired)
{
  auto p = ArmCartesianPlanner::make_default();
  auto plan = p.plan("arm_base", 100.0, 50.0, 0.0, false, 0.0, 3, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::INVALID_INPUT);
}

TEST(ArmCartesianPlannerTest, VelocityScaleSlowsPlan)
{
  auto p = ArmCartesianPlanner::make_default();
  auto fast = p.plan("arm_base", 300.0, 300.0, 90.0, false, 0.0, 0, 1.0, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  auto slow = p.plan("arm_base", 300.0, 300.0, 90.0, false, 0.0, 0, 0.2, 0.0,
                     {0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(fast.status, ArmCartesianPlanner::Plan::Status::OK);
  ASSERT_EQ(slow.status, ArmCartesianPlanner::Plan::Status::OK);
  EXPECT_GT(slow.duration_sec, fast.duration_sec);
}

TEST(ArmCartesianPlannerTest, PlannerRoundTripsModel)
{
  auto p = ArmCartesianPlanner::make_default();
  auto plan = p.plan("arm_base", 150.0, 120.0, -60.0, false, 0.0, 0, 1.0, 0.0,
                     {0.05, 0.1, 0.0, 0.0});
  ASSERT_EQ(plan.status, ArmCartesianPlanner::Plan::Status::OK);
  auto m = ArmKinematicModel::make_default();
  const auto pose = m.forward(plan.q_target);
  EXPECT_NEAR(pose.x, 0.150, 1e-9);
  EXPECT_NEAR(pose.z, 0.120, 1e-9);
  EXPECT_NEAR(pose.yaw, -60.0 * kPi / 180.0, 1e-9);
}
