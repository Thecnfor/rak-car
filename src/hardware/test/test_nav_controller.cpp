// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// NavController unit tests — pure control law: direction, clamping, arrival,
// timeout, angle wrap. No ROS2, no serial.

#include "hardware/nav_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace hardware;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(NavController, DrivesTowardTargetAndReaches)
{
  NavController ctrl;
  NavGoal g;
  g.target = Pose2D{1.0, 0.0, 0.0};
  g.max_linear_speed = 0.3f;
  g.max_angular_speed = 1.0f;
  g.tolerance_lin = 0.02f;
  g.tolerance_ang = 0.05f;
  g.timeout_sec = 5.0f;

  NavTwist tw;
  // 起点在原点,面向 +x → 应直行(vx>0, vy≈0)。
  auto st = ctrl.update(Pose2D{0.0, 0.0, 0.0}, g, 0.1, tw);
  EXPECT_EQ(st, NavStatus::RUNNING);
  EXPECT_GT(tw.vx, 0.0f);
  EXPECT_LT(std::abs(tw.vy), 0.01f);

  // 已接近目标 → REACHED。
  st = ctrl.update(Pose2D{0.99, 0.0, 0.0}, g, 0.2, tw);
  EXPECT_EQ(st, NavStatus::REACHED);
}

TEST(NavController, ReachesOnlyWhenOrientationAligned)
{
  NavController ctrl;
  NavGoal g;
  g.target = Pose2D{1.0, 0.0, 0.0};     // 到位但朝向不对(差 π)
  g.max_linear_speed = 0.3f;
  g.max_angular_speed = 1.0f;
  g.tolerance_lin = 0.02f;
  g.tolerance_ang = 0.05f;
  g.timeout_sec = 5.0f;

  NavTwist tw;
  auto st = ctrl.update(Pose2D{0.995, 0.0, kPi}, g, 0.1, tw);
  EXPECT_EQ(st, NavStatus::RUNNING);    // 位置到位但航向差 π,仍在修正
  // 航向差 π → wrap(-π)= -π → omega 饱和到 -max_angular(-1.0)。
  EXPECT_NEAR(std::abs(tw.omega), 1.0f, 1e-6);
}

TEST(NavController, AbortsOnTimeout)
{
  NavController ctrl;
  NavGoal g;
  g.target = Pose2D{5.0, 0.0, 0.0};
  g.max_linear_speed = 0.1f;
  g.max_angular_speed = 1.0f;
  g.tolerance_lin = 0.02f;
  g.tolerance_ang = 0.05f;
  g.timeout_sec = 2.0f;

  NavTwist tw;
  NavStatus st = NavStatus::RUNNING;
  double t = 0.0;
  while (t < 3.0 && st == NavStatus::RUNNING) {
    st = ctrl.update(Pose2D{0.0, 0.0, 0.0}, g, t, tw);   // 卡住不动
    t += 0.1;
  }
  EXPECT_EQ(st, NavStatus::ABORTED);
}

TEST(NavController, ClampsSpeeds)
{
  NavController ctrl;
  NavGoal g;
  g.target = Pose2D{100.0, 0.0, 0.0};
  g.max_linear_speed = 0.5f;
  g.max_angular_speed = 2.0f;
  g.tolerance_lin = 0.02f;
  g.tolerance_ang = 0.05f;
  g.timeout_sec = 10.0f;

  NavTwist tw;
  ctrl.update(Pose2D{0.0, 0.0, 0.0}, g, 0.1, tw);
  EXPECT_LE(std::abs(tw.vx), 0.5f + 1e-6);
  EXPECT_LE(std::abs(tw.vy), 0.5f + 1e-6);
  EXPECT_LE(std::abs(tw.omega), 2.0f + 1e-6);
}

TEST(NavController, AngleWrapIsShortestPath)
{
  // 目标航向 -2.9,当前 2.9:直接差 -5.8(绕远),wrap 后 = +0.483(短路径)。
  // kP_ang=3 → omega = 0.483*3 = 1.449。max_angular 调大避免饱和,直接钉住数学。
  NavController ctrl;
  NavGoal g;
  g.target = Pose2D{0.0, 0.0, -2.9};
  g.max_linear_speed = 0.1f;
  g.max_angular_speed = 5.0f;
  g.tolerance_lin = 0.02f;
  g.tolerance_ang = 0.05f;
  g.timeout_sec = 5.0f;

  NavTwist tw;
  ctrl.update(Pose2D{0.0, 0.0, 2.9}, g, 0.1, tw);
  // wrap(-5.8) = -5.8 + 2π ≈ +0.483;omega = 0.483*3 ≈ 1.449(短路径正方向)。
  EXPECT_GT(tw.omega, 0.0f);
  EXPECT_NEAR(tw.omega, 1.449f, 0.01f);
}
