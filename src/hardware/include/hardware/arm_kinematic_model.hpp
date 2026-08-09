// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// ArmKinematicModel — parameterized serial-chain kinematics for the rak 4-axis
// arm (M6 horizontal lead-screw + stepper3 vertical + S3 rotation + S7 grip).
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §6.2
//
// DESIGN
//   The kinematics description (axis-aligned DH: per-joint axis + fixed parent
//   offset) lives in DATA (a std::vector<ArmLinkSpec>), not in code. Adding an
//   axis or changing lengths never touches the solver.
//
//   Task space is deliberately bounded (the arm has no independent y axis):
//     * x / z  : prismatic axes (M6 / stepper3), meters
//     * yaw    : rotation about z (S3), radians
//     * y      : a FIXED work plane (y_nominal). Any request whose y deviates
//                beyond y_tolerance returns UNSUPPORTED_DIMENSION, never a
//                silently-wrong solution.
//     * grip   : end-effector only; not part of the task-space IK.
//
//   IK is analytic + nearest-branch:
//     * solve the prismatic axes in closed form for a given yaw,
//     * if multiple branch choices exist, pick the one closest to the current
//       joint state (none today — 3 axes / 3 task DOF — but the structure
//       supports redundancy),
//     * every candidate must satisfy joint limits; otherwise UNREACHABLE.
//
//   Pure C++ (no rclcpp) — unit-testable in isolation.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hardware
{

// Task-space pose (axis-aligned model: full orientation is just yaw about z).
struct ArmTaskPose
{
  double x{0.0};    // m
  double y{0.0};    // m (fixed work plane)
  double z{0.0};    // m
  double yaw{0.0};  // rad (about z)
};

// One joint of the arm chain.
struct ArmJointSpec
{
  enum class Type : uint8_t { PRISMATIC, REVOLUTE };

  std::string name;
  Type type = Type::PRISMATIC;
  char axis = 'x';       // 'x' | 'y' | 'z' (translation / rotation axis)
  double lower = 0.0;    // m (prismatic) or rad (revolute)
  double upper = 0.0;
  double home = 0.0;     // default position
};

// One link: the fixed offset from the parent frame to this joint's origin,
// followed by the joint itself.
struct ArmLinkSpec
{
  ArmJointSpec joint;
  std::array<double, 3> parent_offset{0.0, 0.0, 0.0};  // x, y, z (m)
};

// Tool0 offset from the last joint frame, plus the fixed work plane.
struct ArmToolSpec
{
  std::array<double, 3> offset{0.0, 0.0, 0.0};  // x, y, z (m)
  double y_nominal{0.0};   // the only y the arm can reach (m)
  double y_tolerance{0.0}; // accepted |y - y_nominal| (m)
};

class ArmKinematicModel
{
public:
  ArmKinematicModel(std::vector<ArmLinkSpec> links, ArmToolSpec tool);

  // --- introspection ---
  std::size_t size() const { return links_.size(); }
  const ArmLinkSpec & link(std::size_t i) const { return links_.at(i); }
  const ArmToolSpec & tool() const { return tool_; }
  bool within_limits(const std::vector<double> & q) const;

  // --- forward kinematics ---
  // q: meters for prismatic joints, radians for revolute. Throws
  // std::invalid_argument on size mismatch. Returns the tool0 pose.
  ArmTaskPose forward(const std::vector<double> & q) const;

  // --- inverse kinematics (task space x / z / yaw) ---
  struct IkResult
  {
    enum class Status : uint8_t
    {
      SOLVED = 0,
      UNREACHABLE,          // no branch satisfies joint limits / geometry
      UNSUPPORTED_DIMENSION, // y outside the fixed work plane
      INVALID_INPUT,        // wrong q size
    };
    Status status = Status::INVALID_INPUT;
    std::vector<double> q;  // valid when SOLVED
    std::string reason;
  };

  // current_q: joint state for nearest-branch selection and limit checks.
  // y: pass std::nan("") for "any" (fixed plane); a finite y is validated
  //    against y_nominal ± y_tolerance.
  IkResult inverse_task(double x, double z, double yaw_rad,
                        const std::vector<double> & current_q,
                        double y = std::numeric_limits<double>::quiet_NaN()) const;

  // Default rak arm (M6 + stepper3 + S3 + S7), x/z 0..300 mm, yaw ±150°,
  // grip ±90° (from user, 2026-08-09). Parent/tool offsets default to zero —
  // calibrate before trusting absolute reach.
  static ArmKinematicModel make_default();

private:
  // Accumulated fixed offsets of all prismatic joints along a given axis.
  double fixed_prismatic_sum(char axis) const;
  // Index of the single revolute-z joint used by the task IK, or size() if none.
  std::size_t yaw_joint_index() const;

  std::vector<ArmLinkSpec> links_;
  ArmToolSpec tool_;
};

}  // namespace hardware
