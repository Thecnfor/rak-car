// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// ArmKinematicModel — see header.

#include "hardware/arm_kinematic_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hardware
{

namespace
{

inline constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d)
{
  return d * kPi / 180.0;
}

}  // namespace

ArmKinematicModel::ArmKinematicModel(std::vector<ArmLinkSpec> links, ArmToolSpec tool)
: links_(std::move(links)), tool_(tool)
{
}

bool ArmKinematicModel::within_limits(const std::vector<double> & q) const
{
  if (q.size() != links_.size()) {
    return false;
  }
  for (std::size_t i = 0; i < links_.size(); ++i) {
    const double v = q[i];
    if (v < links_[i].joint.lower - 1e-9 || v > links_[i].joint.upper + 1e-9) {
      return false;
    }
  }
  return true;
}

ArmTaskPose ArmKinematicModel::forward(const std::vector<double> & q) const
{
  if (q.size() != links_.size()) {
    throw std::invalid_argument(
      "ArmKinematicModel::forward: expected " + std::to_string(links_.size()) +
      " joints, got " + std::to_string(q.size()));
  }

  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
  for (std::size_t i = 0; i < links_.size(); ++i) {
    const auto & link = links_[i];
    x += link.parent_offset[0];
    y += link.parent_offset[1];
    z += link.parent_offset[2];
    if (link.joint.type == ArmJointSpec::Type::PRISMATIC) {
      switch (link.joint.axis) {
        case 'x': x += q[i]; break;
        case 'y': y += q[i]; break;
        case 'z': z += q[i]; break;
        default: break;
      }
    } else if (link.joint.axis == 'z') {
      // Rotation about z. (Revolute x/y axes, e.g. the grip, do not move the
      // task-space origin under the axis-aligned model.)
      yaw += q[i];
    }
  }

  // Rotate the fixed tool offset by the accumulated yaw (rotation about z).
  const double cx = std::cos(yaw);
  const double sx = std::sin(yaw);
  x += cx * tool_.offset[0] - sx * tool_.offset[1];
  y += sx * tool_.offset[0] + cx * tool_.offset[1];
  z += tool_.offset[2];

  return ArmTaskPose{x, y, z, yaw};
}

std::size_t ArmKinematicModel::yaw_joint_index() const
{
  for (std::size_t i = 0; i < links_.size(); ++i) {
    if (links_[i].joint.type == ArmJointSpec::Type::REVOLUTE &&
        links_[i].joint.axis == 'z') {
      return i;
    }
  }
  return links_.size();  // none
}

ArmKinematicModel::IkResult ArmKinematicModel::inverse_task(
  double x, double z, double yaw_rad,
  const std::vector<double> & current_q, double y) const
{
  IkResult out;
  if (current_q.size() != links_.size()) {
    out.status = IkResult::Status::INVALID_INPUT;
    out.reason = "current_q size mismatch";
    return out;
  }

  // The arm has a fixed work plane: only y_nominal is reachable.
  if (std::isfinite(y) && std::fabs(y - tool_.y_nominal) > tool_.y_tolerance) {
    out.status = IkResult::Status::UNSUPPORTED_DIMENSION;
    out.reason = "y outside fixed work plane (y_nominal=" +
                 std::to_string(tool_.y_nominal) + ", tolerance=" +
                 std::to_string(tool_.y_tolerance) + ")";
    return out;
  }

  // Locate the axes the closed-form solver needs: one x-prismatic, one
  // z-prismatic (both BEFORE the yaw revolute), and the yaw joint.
  std::size_t idx_x = links_.size(), idx_z = links_.size(), idx_yaw = yaw_joint_index();
  double base_x = 0.0, base_y = 0.0, base_z = 0.0;
  for (std::size_t i = 0; i < links_.size(); ++i) {
    const auto & link = links_[i];
    base_x += link.parent_offset[0];
    base_y += link.parent_offset[1];
    base_z += link.parent_offset[2];
    if (link.joint.type == ArmJointSpec::Type::PRISMATIC && link.joint.axis == 'x' &&
        idx_x == links_.size()) {
      idx_x = i;
    } else if (link.joint.type == ArmJointSpec::Type::PRISMATIC && link.joint.axis == 'z' &&
               idx_z == links_.size()) {
      idx_z = i;
    }
  }
  if (idx_x == links_.size() || idx_z == links_.size() || idx_yaw == links_.size()) {
    out.status = IkResult::Status::UNREACHABLE;
    out.reason = "chain lacks an x-prismatic / z-prismatic / z-revolute axis for the closed form";
    return out;
  }

  // Tool-offset contribution rotated by the target yaw.
  const double cx = std::cos(yaw_rad);
  const double sx = std::sin(yaw_rad);
  const double x_rot = cx * tool_.offset[0] - sx * tool_.offset[1];
  const double y_rot = sx * tool_.offset[0] + cx * tool_.offset[1];

  // Fixed-plane consistency: the model's y must land on y_nominal.
  const double y_model = base_y + y_rot;
  if (std::fabs(y_model - tool_.y_nominal) > tool_.y_tolerance) {
    out.status = IkResult::Status::UNREACHABLE;
    out.reason = "fixed plane mismatch: model y=" + std::to_string(y_model) +
                 " vs y_nominal=" + std::to_string(tool_.y_nominal);
    return out;
  }

  // Closed-form prismatic solves (prismatics precede the yaw joint).
  std::vector<double> q = current_q;
  q[idx_x] = x - base_x - x_rot;
  q[idx_z] = z - base_z - tool_.offset[2];
  q[idx_yaw] = yaw_rad;

  if (!within_limits(q)) {
    out.status = IkResult::Status::UNREACHABLE;
    out.reason = "solution outside joint limits";
    return out;
  }

  out.status = IkResult::Status::SOLVED;
  out.q = std::move(q);
  return out;
}

ArmKinematicModel ArmKinematicModel::make_default()
{
  std::vector<ArmLinkSpec> links;
  links.push_back(ArmLinkSpec{
    ArmJointSpec{"arm_x", ArmJointSpec::Type::PRISMATIC, 'x', 0.0, 0.300, 0.0},
    {0.0, 0.0, 0.0}});
  links.push_back(ArmLinkSpec{
    ArmJointSpec{"arm_z", ArmJointSpec::Type::PRISMATIC, 'z', 0.0, 0.300, 0.0},
    {0.0, 0.0, 0.0}});
  links.push_back(ArmLinkSpec{
    ArmJointSpec{"arm_yaw", ArmJointSpec::Type::REVOLUTE, 'z',
                 deg2rad(-150.0), deg2rad(150.0), 0.0},
    {0.0, 0.0, 0.0}});
  links.push_back(ArmLinkSpec{
    ArmJointSpec{"arm_grip", ArmJointSpec::Type::REVOLUTE, 'y',
                 deg2rad(-90.0), deg2rad(90.0), 0.0},
    {0.0, 0.0, 0.0}});
  return ArmKinematicModel(std::move(links), ArmToolSpec{{0.0, 0.0, 0.0}, 0.0, 0.0});
}

}  // namespace hardware
