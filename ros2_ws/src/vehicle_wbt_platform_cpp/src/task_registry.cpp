// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// TaskRegistry implementation — thread-safe singleton (one global registry).
// All TaskRegistry method implementations live here so the header stays
// header-only for tests (the registry class itself is header-only;
// only the static instance lives here).

#include "vehicle_wbt_platform_cpp/task_registry.hpp"

#include <algorithm>
#include <mutex>
#include <stdexcept>

namespace vehicle_wbt_platform_cpp
{

TaskRegistry & TaskRegistry::instance()
{
  static TaskRegistry inst;
  return inst;
}

void TaskRegistry::register_task(const std::string & name, Factory factory)
{
  if (name.empty()) {
    throw std::invalid_argument("TaskRegistry::register_task: empty name");
  }
  if (!factory) {
    throw std::invalid_argument("TaskRegistry::register_task: null factory for " + name);
  }
  if (factories_.count(name) > 0) {
    throw std::runtime_error("TaskRegistry: task '" + name + "' already registered");
  }
  factories_[name] = std::move(factory);
}

std::unique_ptr<BaseTask> TaskRegistry::create(
  const std::string & name, rclcpp::Node * node) const
{
  auto it = factories_.find(name);
  if (it == factories_.end()) {
    return nullptr;
  }
  if (!it->second) {
    return nullptr;
  }
  return it->second(node);
}

std::vector<std::string> TaskRegistry::list() const
{
  std::vector<std::string> names;
  names.reserve(factories_.size());
  for (const auto & [name, _] : factories_) {
    names.push_back(name);
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace vehicle_wbt_platform_cpp
