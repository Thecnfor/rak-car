// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Unit tests for MC602HardwareInterface — interface export + read/write
// dispatch over a fake responder.
//
// Requires ros2_control (hardware_interface) — gated in CMake, runs on the
// Humble target (dev Lyrical has no hardware_interface).

#include "hardware/mc602_hardware_interface.hpp"

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace
{

using hardware::MC602HardwareInterface;

// Minimal URDF describing the full MC602 joint set (bridge off → direct for
// the test; the adapter is injected anyway so no serial fd is touched).
const std::string kUrdf = R"xml(<?xml version="1.0"?>
<robot name="rak">
  <ros2_control name="MC602" type="system">
    <hardware>
      <plugin>hardware/MC602HardwareInterface</plugin>
      <param name="mc602_transport">direct</param>
      <param name="serial_port">/dev/ttyUSB0</param>
      <param name="baud">1000000</param>
    </hardware>
    <joint name="wheel_m1_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wheel_m2_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wheel_m3_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="wheel_m4_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="arm_horiz_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="arm_vert_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="arm_hand_rotate_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="arm_hand_grip_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>)xml";

std::vector<uint8_t> hex(const std::string & s)
{
  std::vector<uint8_t> out;
  std::string cleaned;
  for (char c : s) {
    if (c != ' ') cleaned += c;
  }
  for (size_t i = 0; i + 1 < cleaned.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoi(cleaned.substr(i, 2), nullptr, 16)));
  }
  return out;
}

struct Harness
{
  MC602HardwareInterface hw;
  hardware_interface::HardwareInfo info;
  std::vector<std::vector<uint8_t>> sent;

  static hardware_interface::HardwareInfo make_info()
  {
    // Humble API: returns a vector (one entry per <ros2_control> block).
    auto infos = hardware_interface::parse_control_resources_from_urdf(kUrdf);
    if (infos.empty()) {
      throw std::runtime_error("no <ros2_control> block parsed from URDF");
    }
    return infos[0];
  }

  Harness()
  : info(make_info())
  {
    auto adapter = std::make_unique<hardware::MC602Adapter>("/dev/ttyUSB0", 1000000);
    adapter->set_injection([this](const std::vector<uint8_t> & frame) {
      sent.push_back(frame);
      // encoder4 response: FL=1 FR=2 RL=3 RR=4 counts.
      return hex("03 01 01000000 02000000 03000000 04000000");
    });
    hw.set_adapter(std::move(adapter));
    EXPECT_EQ(hw.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  }
};

}  // namespace

TEST(Mc602HardwareInterfaceTest, ExportsWheelVelocityAndArmPositionCommands)
{
  Harness h;
  auto cmds = h.hw.export_command_interfaces();
  EXPECT_EQ(cmds.size(), 8u);  // 4 wheel velocity + 4 arm position

  bool saw_wheel_vel = false, saw_arm_pos = false;
  for (const auto & ci : cmds) {
    if (ci.get_prefix_name() == "wheel_m1_joint" &&
        ci.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      saw_wheel_vel = true;
    }
    if (ci.get_prefix_name() == "arm_horiz_joint" &&
        ci.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      saw_arm_pos = true;
    }
  }
  EXPECT_TRUE(saw_wheel_vel);
  EXPECT_TRUE(saw_arm_pos);
}

TEST(Mc602HardwareInterfaceTest, WriteDispatchesMotor4Frame)
{
  Harness h;
  auto cmds = h.hw.export_command_interfaces();
  for (auto & ci : cmds) {
    if (ci.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      ci.set_value(1.0);  // 1 rad/s per wheel
    }
  }
  EXPECT_EQ(h.hw.write(rclcpp::Time(0), rclcpp::Duration(0, 1e8)),
            hardware_interface::return_type::OK);
  ASSERT_FALSE(h.sent.empty());
  // First outgoing frame should be a motor4 SET (dev 0x01).
  EXPECT_EQ(h.sent[0][0], 0x77);
  EXPECT_EQ(h.sent[0][1], 0x68);
  EXPECT_EQ(h.sent[0][2], 0x0A);  // len = payload(4) + 4
  EXPECT_EQ(h.sent[0][3], 0x01);  // dev_id motor4
}

TEST(Mc602HardwareInterfaceTest, ReadPopulatesWheelStatesFromEncoder4)
{
  Harness h;
  EXPECT_EQ(h.hw.read(rclcpp::Time(0), rclcpp::Duration(0, 1e8)),
            hardware_interface::return_type::OK);
  auto states = h.hw.export_state_interfaces();
  double fl = -1.0, rr = -1.0;
  for (const auto & si : states) {
    if (si.get_prefix_name() == "wheel_m1_joint" &&
        si.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      // M1 = front-right = encoder index 1 = 2 counts.
      fl = si.get_value();
    }
    if (si.get_prefix_name() == "wheel_m4_joint" &&
        si.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      // M4 = rear-right = encoder index 3 = 4 counts.
      rr = si.get_value();
    }
  }
  // pos = counts / counts_per_rev * 2π
  EXPECT_NEAR(fl, 2.0 / 2015.13 * 2.0 * 3.14159265358979323846, 1e-9);
  EXPECT_NEAR(rr, 4.0 / 2015.13 * 2.0 * 3.14159265358979323846, 1e-9);
}

TEST(Mc602HardwareInterfaceTest, ArmPositionCommandDispatchesMotorFrame)
{
  Harness h;
  auto cmds = h.hw.export_command_interfaces();
  for (auto & ci : cmds) {
    if (ci.get_prefix_name() == "arm_horiz_joint" &&
        ci.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      ci.set_value(0.1);  // 0.1 m target
    }
  }
  EXPECT_EQ(h.hw.write(rclcpp::Time(0), rclcpp::Duration(0, 1e8)),
            hardware_interface::return_type::OK);
  // A motor frame (dev 0x02) should have been dispatched for M6.
  bool saw_motor = false;
  for (const auto & f : h.sent) {
    if (f.size() >= 5 && f[3] == 0x02) {
      saw_motor = true;
    }
  }
  EXPECT_TRUE(saw_motor);
}
