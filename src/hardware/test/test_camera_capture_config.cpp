// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary

#include "hardware/camera_capture_config.hpp"

#include <opencv2/videoio.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

TEST(CameraCaptureConfigTest, EncodesYuyv680x480At30Hz)
{
  const auto config = hardware::make_camera_capture_config("YUYV", 680, 480, 30.0);

  EXPECT_EQ(config.pixel_format, "YUYV");
  EXPECT_EQ(config.width, 680);
  EXPECT_EQ(config.height, 480);
  EXPECT_DOUBLE_EQ(config.rate_hz, 30.0);
  EXPECT_EQ(config.fourcc, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
}

TEST(CameraCaptureConfigTest, RejectsMalformedPixelFormat)
{
  EXPECT_THROW(
    hardware::make_camera_capture_config("YUY", 680, 480, 30.0),
    std::invalid_argument);
}

TEST(CameraCaptureConfigTest, RejectsNonPositiveCaptureValues)
{
  EXPECT_THROW(
    hardware::make_camera_capture_config("YUYV", 0, 480, 30.0),
    std::invalid_argument);
  EXPECT_THROW(
    hardware::make_camera_capture_config("YUYV", 680, 480, 0.0),
    std::invalid_argument);
}
