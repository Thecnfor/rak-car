// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for MecanumChassis kinematics.
//
// Phase 1.5: pure-math tests (no hardware). Real closed-loop tests with
// ros2_control controller_manager land in Plan B.

#include "vehicle_wbt_platform_cpp/mecanum_chassis.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using vehicle_wbt_platform_cpp::MecanumChassis;
using vehicle_wbt_platform_cpp::Pose2D;

namespace
{
// Helper: compare doubles with tolerance.
constexpr double kEps = 1e-9;
}

TEST(MecanumChassisTest, ConstructorRejectsNonPositiveGeometry)
{
  EXPECT_THROW(MecanumChassis("c1", 0.0, 0.1, 0.03), std::invalid_argument);
  EXPECT_THROW(MecanumChassis("c2", 0.1, -0.1, 0.03), std::invalid_argument);
  EXPECT_THROW(MecanumChassis("c3", 0.1, 0.1, 0.0), std::invalid_argument);
  EXPECT_NO_THROW(MecanumChassis("c4", 0.1, 0.1, 0.03));
}

TEST(MecanumChassisTest, NumWheelsIs4)
{
  MecanumChassis c("mec1", 0.1, 0.1, 0.03);
  EXPECT_EQ(c.num_wheels(), 4u);
}

TEST(MecanumChassisTest, ChassisIdRoundTrips)
{
  MecanumChassis c("hero", 0.15, 0.10, 0.03);
  EXPECT_EQ(c.chassis_id(), "hero");
  EXPECT_DOUBLE_EQ(c.Lx(), 0.15);
  EXPECT_DOUBLE_EQ(c.Ly(), 0.10);
  EXPECT_DOUBLE_EQ(c.wheel_radius(), 0.03);
}

TEST(MecanumChassisTest, InverseKinematicsPureForward)
{
  // vx=1, vy=0, omega=0 -> all wheels = 1
  MecanumChassis c("mec1", 0.1, 0.1, 0.03);
  double w[4];
  c.inverse_kinematics(1.0, 0.0, 0.0, w);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(w[i], 1.0, kEps);
  }
}

TEST(MecanumChassisTest, InverseKinematicsPureStrafe)
{
  // vx=0, vy=1, omega=0 -> fl=-1, fr=+1, rl=+1, rr=-1
  MecanumChassis c("mec1", 0.1, 0.1, 0.03);
  double w[4];
  c.inverse_kinematics(0.0, 1.0, 0.0, w);
  EXPECT_NEAR(w[0], -1.0, kEps);  // fl
  EXPECT_NEAR(w[1], +1.0, kEps);  // fr
  EXPECT_NEAR(w[2], +1.0, kEps);  // rl
  EXPECT_NEAR(w[3], -1.0, kEps);  // rr
}

TEST(MecanumChassisTest, InverseKinematicsPureRotation)
{
  // vx=0, vy=0, omega=1, Lx=Ly=0.1 -> k=0.2
  //   fl = 0 - 0 - 0.2*1 = -0.2
  //   fr = 0 + 0 + 0.2*1 = +0.2
  //   rl = 0 + 0 - 0.2*1 = -0.2
  //   rr = 0 - 0 + 0.2*1 = +0.2
  MecanumChassis c("mec1", 0.1, 0.1, 0.03);
  double w[4];
  c.inverse_kinematics(0.0, 0.0, 1.0, w);
  EXPECT_NEAR(w[0], -0.2, kEps);
  EXPECT_NEAR(w[1], +0.2, kEps);
  EXPECT_NEAR(w[2], -0.2, kEps);
  EXPECT_NEAR(w[3], +0.2, kEps);
}

TEST(MecanumChassisTest, ForwardInverseRoundTrip)
{
  // inverse then forward should recover the original (vx, vy, omega).
  MecanumChassis c("mec1", 0.15, 0.10, 0.03);
  const double vx = 0.7, vy = -0.3, omega = 0.5;
  double w[4];
  c.inverse_kinematics(vx, vy, omega, w);
  double vx2, vy2, omega2;
  c.forward_kinematics(w, vx2, vy2, omega2);
  EXPECT_NEAR(vx, vx2, 1e-9);
  EXPECT_NEAR(vy, vy2, 1e-9);
  EXPECT_NEAR(omega, omega2, 1e-9);
}

TEST(MecanumChassisTest, SetVelocityAdvancesPose)
{
  // dt=1.0 hard-coded in the stub; pose must move by exactly (vx, vy, omega).
  MecanumChassis c("mec1", 0.1, 0.1, 0.03);
  c.reset_odometry();
  EXPECT_DOUBLE_EQ(c.get_pose().x, 0.0);
  EXPECT_DOUBLE_EQ(c.get_pose().y, 0.0);
  EXPECT_DOUBLE_EQ(c.get_pose().theta, 0.0);

  c.set_velocity(1.0, 2.0, 0.0);
  Pose2D p = c.get_pose();
  EXPECT_DOUBLE_EQ(p.x, 1.0);
  EXPECT_DOUBLE_EQ(p.y, 2.0);
  EXPECT_DOUBLE_EQ(p.theta, 0.0);
}

TEST(MecanumChassisTest, TypedArrayConvenienceReturns4Elements)
{
  MecanumChassis c("mec1", 0.1, 0.1, 0.03);
  auto ws = c.inverse(1.0, 0.0, 0.0);
  ASSERT_EQ(ws.values.size(), 4u);
  for (double v : ws.values) {
    EXPECT_NEAR(v, 1.0, kEps);
  }
}
