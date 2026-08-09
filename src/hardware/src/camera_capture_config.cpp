// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary

#include "hardware/camera_capture_config.hpp"

#include <opencv2/videoio.hpp>

#include <stdexcept>

namespace hardware
{

CameraCaptureConfig make_camera_capture_config(
  const std::string & pixel_format,
  int width,
  int height,
  double rate_hz)
{
  if (pixel_format.size() != 4) {
    throw std::invalid_argument("camera pixel format must be exactly four characters");
  }
  if (width <= 0 || height <= 0 || rate_hz <= 0.0) {
    throw std::invalid_argument("camera width, height, and rate must be positive");
  }

  return CameraCaptureConfig{
    pixel_format,
    width,
    height,
    rate_hz,
    cv::VideoWriter::fourcc(
      pixel_format[0], pixel_format[1], pixel_format[2], pixel_format[3])};
}

}  // namespace hardware
