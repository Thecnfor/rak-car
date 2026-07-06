// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BaseChassis — chassis-agnostic abstract class (C++ counterpart of
// vehicle_wbt_platform.chassis_base.BaseChassis).
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象
//
// 5 concrete subclasses planned: MecanumChassis (current), Diff2Chassis,
// Diff4Chassis, TricycleChassis, QuadricycleChassis. Only MecanumChassis
// ships in Phase 1.5; the rest land in Plan B.

#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace vehicle_wbt_platform_cpp
{

// 2D pose in the odom frame: x/y in meters, theta in radians.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// Per-wheel angular velocity, in chassis-specific order.
// - Mecanum 4-wheel: (front_left, front_right, rear_left, rear_right)
// - Diff2: (left, right)
// - Diff4: (fl, fr, rl, rr)
// - Tricycle: (left, right, steer)
// - Quadricycle: (fl, fr, rl, rr)
template <std::size_t N>
struct WheelSpeeds
{
  std::array<double, N> values;

  WheelSpeeds() = default;
  explicit WheelSpeeds(std::array<double, N> v) : values(v) {}
};

class BaseChassis
{
public:
  explicit BaseChassis(std::string chassis_id) : chassis_id_(std::move(chassis_id)) {}

  virtual ~BaseChassis() = default;

  // --- Metadata ---
  virtual std::size_t num_wheels() const = 0;
  const std::string & chassis_id() const { return chassis_id_; }

  // --- Command & odometry ---
  // Command body-frame velocity. vx/vy in m/s, omega in rad/s.
  virtual void set_velocity(double vx, double vy, double omega) = 0;

  // Return current pose (read from wheel encoders + IMU if present).
  virtual Pose2D get_pose() const = 0;

  // Reset pose to origin. Use after physical relocation.
  virtual void reset_odometry() = 0;

  // --- Kinematics ---
  // Convert per-wheel speeds to body-frame velocity.
  virtual void forward_kinematics(const double * wheel_speeds, double & vx, double & vy, double & omega) const = 0;

  // Convert body-frame velocity to per-wheel speeds. Out array must hold
  // num_wheels() elements.
  virtual void inverse_kinematics(double vx, double vy, double omega, double * out_wheel_speeds) const = 0;

protected:
  std::string chassis_id_;
  Pose2D pose_{0.0, 0.0, 0.0};
};

}  // namespace vehicle_wbt_platform_cpp
