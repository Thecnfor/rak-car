// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary

#pragma once

#include <string>

namespace hardware
{

struct CameraCaptureConfig
{
  std::string pixel_format;
  int width;
  int height;
  double rate_hz;
  int fourcc;
};

CameraCaptureConfig make_camera_capture_config(
  const std::string & pixel_format,
  int width,
  int height,
  double rate_hz);

}  // namespace hardware
