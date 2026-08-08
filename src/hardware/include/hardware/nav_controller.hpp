// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// NavController — 底盘点到位 P 控制器(纯类,无 ROS 依赖,可单测)。
//
// 每个控制 tick 喂当前位姿 + 目标 + 已耗时,输出期望的 body-frame Twist
// 与状态(RUNNING/REACHED/ABORTED)。chassis 节点把它接在 action server 上:
// goal → NavGoal,里程计 pose → current,输出 Twist 直接命令电机。
//
// 控制律(简单 P):
//   vx/vy  = 朝目标方向的分量 * dist * kP_lin(夹到 max_linear_speed)
//   omega  = 目标航向差 * kP_ang(夹到 max_angular_speed)
// 到位  = dist < tol_lin && |theta_err| < tol_ang
// 超时  = elapsed > timeout_sec → ABORTED

#pragma once

#include "hardware/base_chassis.hpp"

#include <cmath>
#include <cstdint>

namespace hardware
{

struct NavGoal
{
  Pose2D target;
  float max_linear_speed = 0.3f;    // m/s
  float max_angular_speed = 1.0f;   // rad/s
  float tolerance_lin = 0.02f;      // m
  float tolerance_ang = 0.05f;      // rad
  float timeout_sec = 5.0f;
};

struct NavTwist
{
  float vx = 0.0f;      // m/s
  float vy = 0.0f;      // m/s
  float omega = 0.0f;   // rad/s
};

enum class NavStatus
{
  RUNNING,
  REACHED,
  ABORTED,
};

class NavController
{
public:
  // 每 tick 调用一次;elapsed = 距 goal 开始的秒数。输出期望 body Twist。
  NavStatus update(const Pose2D & current, const NavGoal & goal,
                   double elapsed, NavTwist & out);

private:
  static double wrap_pi(double a);
};

}  // namespace hardware
