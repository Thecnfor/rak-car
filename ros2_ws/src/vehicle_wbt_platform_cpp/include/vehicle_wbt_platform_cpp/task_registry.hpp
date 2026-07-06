// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// TaskRegistry — singleton registry mapping task names to factory functions.
//
// Tasks self-register via the REGISTER_TASK macro. The MissionRunnerNode
// looks up tasks by name (e.g. "seeding", "pest_scout") at runtime, so
// adding a new task = add 1 .cpp file with REGISTER_TASK("name", ClassName).
//
// This file is pure C++ (no rclcpp) so it can be unit-tested in 0.03s.

#pragma once

#include "vehicle_wbt_platform_cpp/base_task.hpp"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace vehicle_wbt_platform_cpp
{

class TaskRegistry
{
public:
  // Factory: takes a rclcpp::Node* (for topic creation) and returns
  // a new task instance. Returning nullptr indicates factory failure.
  using Factory = std::function<std::unique_ptr<BaseTask>(rclcpp::Node *)>;

  static TaskRegistry & instance();

  // Register a task factory. Throws if name already registered.
  void register_task(const std::string & name, Factory factory);

  // Create a task instance by name. Returns nullptr if name not found
  // or factory returned nullptr.
  std::unique_ptr<BaseTask> create(const std::string & name, rclcpp::Node * node) const;

  // All registered task names (alphabetical).
  std::vector<std::string> list() const;

  // Number of registered tasks (useful for tests).
  std::size_t size() const { return factories_.size(); }

  // Remove all registrations (tests only).
  void clear_for_testing() { factories_.clear(); }

private:
  TaskRegistry() = default;
  std::map<std::string, Factory> factories_;
};

// Self-registering macro. Place in the .cpp file of a task:
//   REGISTER_TASK("seeding", SeedingTask)
// This creates a static initializer that runs at program startup,
// calling TaskRegistry::instance().register_task("seeding", ...).
#define REGISTER_TASK(task_name, class_name)                          \
  namespace                                                            \
  {                                                                    \
  struct task_register_##class_name {                                  \
    task_register_##class_name()                                       \
    {                                                                  \
      ::vehicle_wbt_platform_cpp::TaskRegistry::instance().register_task( \
        task_name, [](::rclcpp::Node * node) -> std::unique_ptr<::vehicle_wbt_platform_cpp::BaseTask> { \
          return std::make_unique<class_name>(node);                    \
        });                                                            \
    }                                                                  \
  };                                                                   \
  static task_register_##class_name _task_register_instance_##class_name; \
  }  // namespace

}  // namespace vehicle_wbt_platform_cpp
