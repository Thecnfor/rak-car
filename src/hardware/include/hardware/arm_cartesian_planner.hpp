// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// ArmCartesianPlanner — pure planning logic for the ArmCartesianMove action.
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §6.3
//
// Converts an ArmCartesianMove goal (mm / deg) into a concrete joint-space
// target + a crude duration estimate, using ArmKinematicModel. rclcpp-free —
// the action server node is a thin shell around this.
//
//   goal:  x/z (mm), yaw (deg), optional y (mm, fixed work plane),
//          gripper_action (0=none 1=grip 2=release 3=pump_on 4=pump_off)
//   out:   q_target (m / rad) + duration_sec, or a precise rejection reason.
//
// pump_on/pump_off are NOT dispatched here (no pump/valve command interface
// yet) — they yield INVALID_INPUT until the DOUT path is wired.

#pragma once

#include "hardware/arm_kinematic_model.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace hardware
{

class ArmCartesianPlanner
{
public:
  struct Plan
  {
    enum class Status : uint8_t
    {
      OK = 0,
      UNREACHABLE,
      UNSUPPORTED_DIMENSION,
      INVALID_INPUT,  // bad frame / gripper action / param
    };
    Status status = Status::INVALID_INPUT;
    std::vector<double> q_target;  // m / rad, valid when OK
    double duration_sec = 0.0;     // crude time estimate (ramp + settle)
    std::string error;
  };

  // model: arm kinematics; grip_angle_deg / release_angle_deg: arm_grip joint
  // targets for gripper_action grip / release (deg).
  ArmCartesianPlanner(ArmKinematicModel model,
                      double grip_angle_deg, double release_angle_deg);

  // frame_id must be "arm_base" (only supported frame in the first version).
  Plan plan(const std::string & frame_id,
            double x_mm, double z_mm, double yaw_deg,
            bool y_enabled, double y_mm, uint8_t gripper_action,
            double velocity_scale, double position_tolerance_mm,
            const std::vector<double> & current_q) const;

  // Default planner for the rak arm (make_default model + ±90° grip targets).
  static ArmCartesianPlanner make_default();

private:
  ArmKinematicModel model_;
  double grip_angle_deg_;
  double release_angle_deg_;
};

}  // namespace hardware
