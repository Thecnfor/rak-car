// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MissionRunner implementation — see mission_runner.hpp for the contract.

#include "hardware/mission_runner.hpp"

#include <algorithm>
#include <utility>

namespace hardware
{

MissionRunner::MissionRunner(TaskFactory factory, ProgressCallback progress)
: factory_(std::move(factory))
, progress_(std::move(progress))
{
}

bool MissionRunner::start(
  const std::vector<std::string> & task_names,
  double per_task_timeout_sec,
  const std::vector<std::string> & available_names)
{
  task_names_ = task_names;
  available_names_ = available_names;
  timeout_sec_ = per_task_timeout_sec;
  current_index_ = 0;
  current_task_.reset();
  running_ = false;

  if (task_names_.empty()) {
    report("idle: empty task_list");
    return false;
  }

  for (const auto & name : task_names_) {
    if (std::find(available_names_.begin(), available_names_.end(), name) ==
      available_names_.end())
    {
      report("failed: unknown task '" + name + "'");
      return false;
    }
  }

  running_ = true;
  report("started: " + std::to_string(task_names_.size()) + " tasks");
  return true;
}

bool MissionRunner::tick(double now_sec)
{
  if (!running_) {
    return false;
  }

  if (current_index_ >= task_names_.size()) {
    // Mission complete.
    report("completed: " + std::to_string(task_names_.size()) + " tasks");
    running_ = false;
    return false;
  }

  // Lazy-init the current task on its first tick.
  if (!current_task_) {
    const std::string & name = task_names_[current_index_];
    current_task_ = factory_(name);
    if (!current_task_) {
      report("failed: factory for '" + name + "' returned null");
      running_ = false;
      return false;
    }

    TaskContext ctx;
    ctx.node = node_;
    ctx.timeout_sec = timeout_sec_;
    ctx.task_index = static_cast<int>(current_index_);
    task_start_sec_ = now_sec;

    const auto init_status = current_task_->on_init(ctx);
    if (init_status != TaskStatus::RUNNING &&
        init_status != TaskStatus::SUCCESS)
    {
      report(
        "failed: " + name + " on_init returned " + to_string(init_status));
      safe_cleanup();
      running_ = false;
      return false;
    }

    if (init_status == TaskStatus::SUCCESS) {
      // Single-step task: done immediately, advance on the next tick.
      safe_cleanup();
      ++current_index_;
      return true;
    }
  }

  // One tick of the current task.
  const std::string & name = task_names_[current_index_];
  const auto status = current_task_->execute();

  // Timeout check (after execute, matching the pre-refactor node).
  if (timeout_sec_ > 0.0 && (now_sec - task_start_sec_) > timeout_sec_) {
    report(
      "timeout: " + name + " (elapsed " +
      std::to_string(now_sec - task_start_sec_) + "s)");
    safe_cleanup();
    running_ = false;
    return false;
  }

  if (status == TaskStatus::RUNNING) {
    return true;  // keep ticking
  }

  // Task finished (SUCCESS / FAILED / TIMEOUT returned from execute()).
  report(std::string(to_string(status)) + ": " + name);
  safe_cleanup();

  if (status == TaskStatus::FAILED || status == TaskStatus::TIMEOUT) {
    running_ = false;  // abort mission
    return false;
  }

  ++current_index_;
  return true;
}

void MissionRunner::report(const std::string & msg) const
{
  if (progress_) {
    progress_(msg);
  }
}

void MissionRunner::safe_cleanup()
{
  if (!current_task_) {
    return;
  }
  const std::string name = current_task_->name();
  try {
    current_task_->on_cleanup();
  } catch (const std::exception & e) {
    report("warn: " + name + " on_cleanup threw: " + e.what());
  } catch (...) {
    report("warn: " + name + " on_cleanup threw unknown error");
  }
  current_task_.reset();
}

}  // namespace hardware
