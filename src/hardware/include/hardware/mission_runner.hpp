// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MissionRunner — pure mission state machine (no rclcpp).
//
// Drives an ordered list of BaseTask instances through their lifecycle:
// for each task, on_init() once, execute() every tick, on_cleanup() once.
// Owns the abort semantics: FAILED / TIMEOUT abort the mission, SUCCESS
// advances to the next task, RUNNING keeps ticking.
//
// This class is deliberately rclcpp-free (it only stores an opaque
// rclcpp::Node* to forward into TaskContext). MissionRunnerNode is the
// thin ROS2 shell that wires the real TaskRegistry, a progress publisher,
// and a timer. Unit tests drive this class with fake tasks + synthetic
// clocks — see test/test_mission_runner.cpp.
//
// Progress messages are plain strings (the shell publishes them on
// /rak/state/mission_progress):
//   "started: N tasks" / "completed: N tasks"
//   "failed: unknown task 'X'" / "failed: X on_init returned Y"
//   "failed: factory for 'X' returned null"
//   "timeout: X (elapsed Ns)"
//   "<STATUS>: X" for each finished task

#pragma once

#include "hardware/base_task.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

// Opaque; never dereferenced by this class.
namespace rclcpp
{
class Node;
}

namespace hardware
{

class MissionRunner
{
public:
  // Creates a task by name. The shell wires this to TaskRegistry; tests
  // substitute a fake factory returning DummyTask-like instances.
  using TaskFactory = std::function<std::unique_ptr<BaseTask>(const std::string & name)>;

  // One progress event string (see class comment for the format).
  using ProgressCallback = std::function<void(const std::string & message)>;

  MissionRunner(TaskFactory factory, ProgressCallback progress);

  // Inject the owning node so TaskContext.node reaches tasks' on_init().
  // The runner stores it opaquely and never calls into ROS2 itself.
  void set_node(rclcpp::Node * node) { node_ = node; }

  // (Re)start a mission. Returns true if the mission is now running,
  // false if it failed to start (empty task list or unknown task name —
  // the reason is reported via the progress callback). available_names is
  // the set of registered task names used for validation.
  bool start(
    const std::vector<std::string> & task_names,
    double per_task_timeout_sec,
    const std::vector<std::string> & available_names);

  // Advance one tick. now_sec is wall-clock seconds (elapsed is measured
  // per task for the timeout). Returns true while the mission is still
  // running, false when it has completed or aborted.
  bool tick(double now_sec);

  bool running() const { return running_; }
  std::size_t current_index() const { return current_index_; }
  const std::vector<std::string> & task_names() const { return task_names_; }

private:
  void report(const std::string & msg) const;
  // Calls current_task_->on_cleanup() guarded against exceptions, then
  // resets the task. Never throws.
  void safe_cleanup();

  TaskFactory factory_;
  ProgressCallback progress_;
  rclcpp::Node * node_{nullptr};

  std::vector<std::string> task_names_;
  std::vector<std::string> available_names_;
  double timeout_sec_{0.0};
  std::size_t current_index_{0};
  std::unique_ptr<BaseTask> current_task_;
  double task_start_sec_{0.0};
  bool running_{false};
};

}  // namespace hardware
