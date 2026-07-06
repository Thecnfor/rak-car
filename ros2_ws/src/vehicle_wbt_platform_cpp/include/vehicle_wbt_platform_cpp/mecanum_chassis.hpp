// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MecanumChassis — 4-wheel mecanum (O-layout) chassis kinematics.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象
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

#include "vehicle_wbt_platform_cpp/base_chassis.hpp"

#include <array>
#include <string>

namespace vehicle_wbt_platform_cpp
{

class MecanumChassis : public BaseChassis
{
public:
  // Lx = half wheelbase (m), Ly = half track (m), wheel_radius (m).
  // All 4 wheels are mecanum (O layout) with the same radius.
  MecanumChassis(std::string chassis_id, double Lx, double Ly, double wheel_radius);

  std::size_t num_wheels() const override { return 4; }

  void set_velocity(double vx, double vy, double omega) override;
  Pose2D get_pose() const override { return pose_; }
  void reset_odometry() override { pose_ = Pose2D{0.0, 0.0, 0.0}; }

  void forward_kinematics(const double * wheel_speeds, double & vx, double & vy, double & omega) const override;
  void inverse_kinematics(double vx, double vy, double omega, double * out_wheel_speeds) const override;

  // Convenience: typed array for the 4 mecanum wheels.
  using FourWheelSpeeds = WheelSpeeds<4>;
  FourWheelSpeeds inverse(double vx, double vy, double omega) const;

  // Geometry accessors (used by URDF/xacro to validate).
  double Lx() const { return Lx_; }
  double Ly() const { return Ly_; }
  double wheel_radius() const { return wheel_radius_; }

private:
  double Lx_;          // half wheelbase (m)
  double Ly_;          // half track (m)
  double wheel_radius_;
};

}  // namespace vehicle_wbt_platform_cpp
