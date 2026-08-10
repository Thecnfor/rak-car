// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MecanumChassis — 4-wheel mecanum (O-layout) chassis kinematics.
//
// Wheel layout (top-down view, car facing +x):
//     M2(front-left)   M1(front-right)
//     M3(rear-left)    M4(rear-right)
//
// Standard mecanum equations (4-wheel O layout, identical wheels):
//   v_fl = vx - vy - (Lx + Ly) * omega
//   v_fr = vx + vy + (Lx + Ly) * omega
//   v_rl = vx + vy - (Lx + Ly) * omega
//   v_rr = vx - vy + (Lx + Ly) * omega
//
// where Lx = half wheelbase, Ly = half track. Inverse swaps signs.

#pragma once

#include "hardware/base_chassis.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace hardware
{

class MecanumChassis
{
public:
  // Lx = half wheelbase (m), Ly = half track (m), wheel_radius (m).
  // All 4 wheels are mecanum (O layout) with the same radius.
  MecanumChassis(std::string chassis_id, double Lx, double Ly, double wheel_radius);

  std::size_t num_wheels() const { return 4; }
  const std::string & chassis_id() const { return chassis_id_; }

  void set_velocity(double vx, double vy, double omega);
  Pose2D get_pose() const { return pose_; }
  void reset_odometry() { pose_ = Pose2D{0.0, 0.0, 0.0}; }

  void forward_kinematics(const double * wheel_speeds, double & vx, double & vy, double & omega) const;
  void inverse_kinematics(double vx, double vy, double omega, double * out_wheel_speeds) const;

  // Convenience: typed array for the 4 mecanum wheels.
  std::array<double, 4> inverse(double vx, double vy, double omega) const;

  // Geometry accessors (used by URDF/xacro to validate).
  double Lx() const { return Lx_; }
  double Ly() const { return Ly_; }
  double wheel_radius() const { return wheel_radius_; }

private:
  std::string chassis_id_;
  Pose2D pose_{0.0, 0.0, 0.0};
  double Lx_;          // half wheelbase (m)
  double Ly_;          // half track (m)
  double wheel_radius_;
};

}  // namespace hardware
