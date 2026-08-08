// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SeedingTask — example 8.10 task implemented as a BaseTask subclass.
//
// Demonstrates the "task framework" pattern: a single .cpp file
// with class SeedingTask : public BaseTask + one REGISTER_TASK line.
// To add a new task (pest_scout, shoot_pest, etc.), copy this file,
// change the class + execute() body, update the registered name.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型

#include "hardware/base_task.hpp"
#include "hardware/task_registry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace vwh = hardware;

class SeedingTask : public vwh::BaseTask
{
public:
  // Constructor takes a rclcpp::Node* (the runner passes itself).
  // The task should NOT store this past on_cleanup(); treat it as a
  // scratch reference for creating publishers/subscribers/params.
  explicit SeedingTask(rclcpp::Node * node)
  : node_(node)
  {
  }

  std::string name() const override { return "seeding"; }

  vwh::TaskStatus on_init(const vwh::TaskContext & ctx) override
  {
    // Read parameters (real impl would use cfg_mission.yml)
    stations_ = node_->declare_parameter<std::vector<double>>(
      "seeding_stations",
      std::vector<double>{0.45, 0.55, 0.785, 0.60, 0.70, 0.785, 0.75, 0.85, 0.785}
    );
    // Create publisher for /cmd/vel_safe (where this task sends movement)
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "/rak/cmd/vel_safe", 10);
    current_station_ = 0;
    RCLCPP_INFO(node_->get_logger(),
      "SeedingTask initialized: %zu stations, timeout=%.1fs",
      stations_.size() / 3, ctx.timeout_sec);
    return vwh::TaskStatus::RUNNING;
  }

  vwh::TaskStatus execute() override
  {
    // Simplified: pretend to drive to each station + beep.
    // Real impl: use my_car.move_to_position + task.ring.rings()
    if (current_station_ >= stations_.size() / 3) {
      return vwh::TaskStatus::SUCCESS;
    }
    // Stub: publish a forward command, increment, and finish after
    // a small "drive time" simulated by ticks.
    if (drive_ticks_++ < 3) {
      auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
      cmd->linear.x = 0.3;
      vel_pub_->publish(std::move(cmd));
      return vwh::TaskStatus::RUNNING;
    }
    drive_ticks_ = 0;
    ++current_station_;
    RCLCPP_INFO(node_->get_logger(),
      "SeedingTask: station %zu seeded", current_station_);
    if (current_station_ >= stations_.size() / 3) {
      return vwh::TaskStatus::SUCCESS;
    }
    return vwh::TaskStatus::RUNNING;
  }

  void on_cleanup() override
  {
    vel_pub_.reset();
    RCLCPP_INFO(node_->get_logger(), "SeedingTask cleaned up");
  }

  std::string info() const override
  {
    return "seeding: 3-station planting pattern, ~10s per station";
  }

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::vector<double> stations_;
  std::size_t current_station_{0};
  // Per-instance tick counter for the stub drive simulation (was a
  // function-local static — shared across instances, a latent bug).
  int drive_ticks_{0};
};

// Self-registration: at program startup, register "seeding" -> SeedingTask factory
REGISTER_TASK("seeding", SeedingTask)
