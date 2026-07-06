// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MecanumChassis implementation — standard 4-wheel O-layout kinematics.

#include "vehicle_wbt_platform_cpp/mecanum_chassis.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace vehicle_wbt_platform_cpp
{

MecanumChassis::MecanumChassis(std::string chassis_id, double Lx, double Ly, double wheel_radius)
: BaseChassis(std::move(chassis_id)), Lx_(Lx), Ly_(Ly), wheel_radius_(wheel_radius)
{
  if (Lx <= 0.0 || Ly <= 0.0 || wheel_radius <= 0.0) {
    throw std::invalid_argument("MecanumChassis: Lx, Ly, wheel_radius must all be > 0");
  }
}

void MecanumChassis::set_velocity(double vx, double vy, double omega)
{
  // In a real implementation this would call MC602Adapter::write_actuator
  // for each wheel. Here we just store the last commanded velocity for
  // forward-kinematics testability. Hardware dispatch lands in Plan B.
  // The 4 wheel speeds are derived from inverse kinematics.
  std::array<double, 4> ws = inverse(vx, vy, omega).values;
  // Integrate pose assuming dt = 1 for simplicity. Real impl will use the
  // ros2_control write() callback tick.
  pose_.x += vx * 1.0;
  pose_.y += vy * 1.0;
  pose_.theta += omega * 1.0;
  // Normalize theta to [-pi, pi].
  while (pose_.theta > M_PI) pose_.theta -= 2.0 * M_PI;
  while (pose_.theta < -M_PI) pose_.theta += 2.0 * M_PI;
  (void)ws;  // wheel command dispatch lives in ros2_control layer
}

void MecanumChassis::forward_kinematics(const double * wheel_speeds, double & vx, double & vy, double & omega) const
{
  // wheel_speeds: (fl, fr, rl, rr) in rad/s
  const double v_fl = wheel_speeds[0];
  const double v_fr = wheel_speeds[1];
  const double v_rl = wheel_speeds[2];
  const double v_rr = wheel_speeds[3];
  const double k = (Lx_ + Ly_);
  vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
  vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
  omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * k);
}

void MecanumChassis::inverse_kinematics(double vx, double vy, double omega, double * out) const
{
  const double k = (Lx_ + Ly_);
  out[0] = vx - vy - k * omega;  // fl
  out[1] = vx + vy + k * omega;  // fr
  out[2] = vx + vy - k * omega;  // rl
  out[3] = vx - vy + k * omega;  // rr
}

MecanumChassis::FourWheelSpeeds MecanumChassis::inverse(double vx, double vy, double omega) const
{
  double w[4];
  inverse_kinematics(vx, vy, omega, w);
  return FourWheelSpeeds{std::array<double, 4>{w[0], w[1], w[2], w[3]}};
}

}  // namespace vehicle_wbt_platform_cpp
