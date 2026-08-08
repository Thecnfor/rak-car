// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// NavController — see header.

#include "hardware/nav_controller.hpp"

#include <algorithm>

namespace hardware
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double NavController::wrap_pi(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

NavStatus NavController::update(const Pose2D & cur, const NavGoal & g,
                                double elapsed, NavTwist & out)
{
  if (elapsed > g.timeout_sec) {
    return NavStatus::ABORTED;
  }

  const double dx = g.target.x - cur.x;
  const double dy = g.target.y - cur.y;
  const double dist = std::hypot(dx, dy);
  const double heading_err = wrap_pi(std::atan2(dy, dx) - cur.theta);
  const double theta_err = wrap_pi(g.target.theta - cur.theta);

  const double max_lin = static_cast<double>(g.max_linear_speed);
  const double max_ang = static_cast<double>(g.max_angular_speed);

  // 朝目标方向前进:body-frame 里分解,夹到 max_linear_speed。
  out.vx = static_cast<float>(std::clamp(std::cos(heading_err) * dist * 2.0,
                                         -max_lin, max_lin));
  out.vy = static_cast<float>(std::clamp(std::sin(heading_err) * dist * 2.0,
                                         -max_lin, max_lin));
  // 同时修正航向(与移动解耦,可原地转)。
  out.omega = static_cast<float>(std::clamp(theta_err * 3.0, -max_ang, max_ang));

  if (dist < static_cast<double>(g.tolerance_lin) &&
      std::abs(theta_err) < static_cast<double>(g.tolerance_ang)) {
    return NavStatus::REACHED;
  }
  return NavStatus::RUNNING;
}

}  // namespace hardware
