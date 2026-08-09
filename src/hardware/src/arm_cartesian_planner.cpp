// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// ArmCartesianPlanner — see header.

#include "hardware/arm_cartesian_planner.hpp"

#include <cmath>
#include <limits>

namespace hardware
{

namespace
{

inline double deg2rad(double d)
{
  return d * 3.14159265358979323846 / 180.0;
}

inline double mm2m(double mm)
{
  return mm / 1000.0;
}

}  // namespace

ArmCartesianPlanner::ArmCartesianPlanner(ArmKinematicModel model,
                                         double grip_angle_deg,
                                         double release_angle_deg)
: model_(std::move(model)), grip_angle_deg_(grip_angle_deg), release_angle_deg_(release_angle_deg)
{
}

ArmCartesianPlanner::Plan ArmCartesianPlanner::plan(
  const std::string & frame_id,
  double x_mm, double z_mm, double yaw_deg,
  bool y_enabled, double y_mm, uint8_t gripper_action,
  double velocity_scale, double position_tolerance_mm,
  const std::vector<double> & current_q) const
{
  Plan out;

  if (frame_id != "arm_base") {
    out.status = Plan::Status::UNSUPPORTED_DIMENSION;
    out.error = "only frame_id='arm_base' supported, got '" + frame_id + "'";
    return out;
  }
  if (current_q.size() != model_.size()) {
    out.status = Plan::Status::INVALID_INPUT;
    out.error = "current_q size mismatch";
    return out;
  }
  if (gripper_action > 2) {  // 3=pump_on / 4=pump_off not wired yet
    out.status = Plan::Status::INVALID_INPUT;
    out.error = "gripper_action pump/valve not wired to ros2_control yet";
    return out;
  }
  if (!std::isfinite(x_mm) || !std::isfinite(z_mm) || !std::isfinite(yaw_deg)) {
    out.status = Plan::Status::INVALID_INPUT;
    out.error = "non-finite x/z/yaw";
    return out;
  }

  // Fixed work plane semantics: y is only accepted near the plane origin.
  const double y_m = y_enabled ? mm2m(y_mm) : 0.0;

  auto ik = model_.inverse_task(mm2m(x_mm), mm2m(z_mm), deg2rad(yaw_deg),
                                current_q, y_enabled ? y_m : std::numeric_limits<double>::quiet_NaN());
  switch (ik.status) {
    case ArmKinematicModel::IkResult::Status::SOLVED:
      break;
    case ArmKinematicModel::IkResult::Status::UNREACHABLE:
      out.status = Plan::Status::UNREACHABLE;
      out.error = "unreachable: " + ik.reason;
      return out;
    case ArmKinematicModel::IkResult::Status::UNSUPPORTED_DIMENSION:
      out.status = Plan::Status::UNSUPPORTED_DIMENSION;
      out.error = ik.reason;
      return out;
    case ArmKinematicModel::IkResult::Status::INVALID_INPUT:
    default:
      out.status = Plan::Status::INVALID_INPUT;
      out.error = ik.reason;
      return out;
  }

  // Gripper joint target for grip / release.
  switch (gripper_action) {
    case 1: ik.q[3] = deg2rad(grip_angle_deg_); break;    // grip → closed
    case 2: ik.q[3] = deg2rad(release_angle_deg_); break; // release → open
    default: break;                                        // 0 = hold
  }

  out.status = Plan::Status::OK;
  out.q_target = std::move(ik.q);

  // Crude duration: dominant joint motion, slowed by velocity_scale.
  double dist = 0.0;
  for (std::size_t i = 0; i < model_.size(); ++i) {
    dist = std::max(dist, std::fabs(out.q_target[i] - current_q[i]));
  }
  const double scale = (velocity_scale > 0.0 && velocity_scale <= 1.0)
    ? velocity_scale : 1.0;
  out.duration_sec = std::max(0.5, dist / 0.1 / scale + 0.4);  // 0.1 unit/s + settle
  (void)position_tolerance_mm;  // consumed by the executor, not the planner
  return out;
}

ArmCartesianPlanner ArmCartesianPlanner::make_default()
{
  return ArmCartesianPlanner(ArmKinematicModel::make_default(), 90.0, -90.0);
}

}  // namespace hardware
