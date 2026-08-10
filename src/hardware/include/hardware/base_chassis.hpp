// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Shared 2D pose type for chassis kinematics. (The BaseChassis abstract
// layer was cut — MecanumChassis is the only concrete chassis and holds no
// base type.)

#pragma once

namespace hardware
{

// 2D pose in the odom frame: x/y in meters, theta in radians.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

}  // namespace hardware
