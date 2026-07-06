// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BaseTask — abstract task lifecycle contract for the 8.10 mission framework.
//
// Tasks (seeding, pest_scout, shoot_pest, harvest, read_order, delivery)
// implement this interface. The framework (MissionRunnerNode + TaskRegistry)
// drives their lifecycle. Tasks declare what they need via TaskContext;
// ROS2 details (topics, lifecycle transitions, param servers) are owned
// by the framework, not the task.
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型
//
// Lifecycle:
//   on_init     — called once. Read params, validate config, create pubs/subs.
//   on_activate  — called when the task should start producing work.
//   execute     — called every tick (e.g. 50Hz). Return TaskStatus:
//                   RUNNING  — keep calling next tick
//                   SUCCESS  — task done, move to next
//                   FAILED   — abort mission
//                   TIMEOUT  — exceeded on_init timeout_sec
//   on_cleanup   — called once at end. Release resources.

#pragma once

#include <cstdint>
#include <memory>
#include <string>

// Forward declarations to keep this header rclcpp-free (unit-testable).
namespace rclcpp
{
class Node;
}

namespace vehicle_wbt_platform_cpp
{

enum class TaskStatus : int
{
  RUNNING = 0,  // task in progress; runner will call execute() again
  SUCCESS = 1,  // task done successfully; runner moves to next
  FAILED  = 2,  // task failed irrecoverably; runner aborts mission
  TIMEOUT = 3,  // task exceeded its time budget
};

inline const char * to_string(TaskStatus s)
{
  switch (s) {
    case TaskStatus::RUNNING: return "RUNNING";
    case TaskStatus::SUCCESS: return "SUCCESS";
    case TaskStatus::FAILED:  return "FAILED";
    case TaskStatus::TIMEOUT: return "TIMEOUT";
  }
  return "UNKNOWN";
}

// Task context — passed to on_init(). Tasks should not store the
// node pointer past their lifecycle; use it only during on_init.
struct TaskContext
{
  rclcpp::Node * node{nullptr};
  double timeout_sec{30.0};
  // Task index in the mission (0-based) — useful for logging
  int task_index{0};
};

class BaseTask
{
public:
  virtual ~BaseTask() = default;

  // Identity — used by the registry and for logging
  virtual std::string name() const = 0;

  // Lifecycle hooks (see class comment for contract)
  virtual TaskStatus on_init(const TaskContext & ctx) = 0;
  virtual TaskStatus execute() = 0;
  virtual void on_cleanup() = 0;

  // Optional: announce what topics this task publishes / subscribes.
  // Used for documentation and debug introspection; not enforced.
  virtual std::string info() const { return name(); }
};

}  // namespace vehicle_wbt_platform_cpp
