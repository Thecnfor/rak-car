// Copyright 6 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MissionRunnerNode — drives a list of BaseTask instances through their
// lifecycle. Reads a string[] param 'task_list' (e.g. ["seeding",
// "pest_scout", "harvest"]) and runs each one in sequence.
//
// This is the bridge between "tasks are pure C++ classes with no
// ROS2 knowledge" and "actually executing them on a robot". Tasks
// focus on domain logic (seeding, harvesting); this node focuses
// on ROS2 plumbing (pubs, subs, params, lifecycle, mission state).
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型
//
// Usage:
//   ros2 run vehicle_wbt_platform_cpp mission_runner_node --ros-args \
//     -p task_list:='[seeding, pest_scout, shoot_pest, harvest]'

#include "vehicle_wbt_platform_cpp/base_task.hpp"
#include "vehicle_wbt_platform_cpp/task_registry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace vwpc = vehicle_wbt_platform_cpp;

class MissionRunnerNode : public rclcpp::Node
{
public:
  explicit MissionRunnerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("mission_runner_node", options)
  {
    // Parameter: ordered list of task names to run
    this->declare_parameter<std::vector<std::string>>(
      "task_list", std::vector<std::string>{});
    // Per-task timeout in seconds (0 = use task default)
    this->declare_parameter<double>("task_timeout_sec", 30.0);

    progress_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/vehicle_wbt/v1/state/mission_progress", 10);

    // Start the mission after construction (when rclcpp::spin is up).
    init_timer_ = this->create_wall_timer(
      100ms,
      [this]() { this->init_timer_->cancel(); this->start_mission(); });
  }

private:
  void start_mission()
  {
    const auto task_names = this->get_parameter("task_list").as_string_array();
    const double timeout_sec = this->get_parameter("task_timeout_sec").as_double();

    if (task_names.empty()) {
      RCLCPP_WARN(this->get_logger(), "task_list is empty; nothing to do");
      publish_progress("idle: empty task_list");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Starting mission with %zu tasks: [%s]",
      task_names.size(),
      [&]() {
        std::string joined;
        for (std::size_t i = 0; i < task_names.size(); ++i) {
          if (i) joined += ", ";
          joined += task_names[i];
        }
        return joined;
      }().c_str());

    const auto available = vwpc::TaskRegistry::instance().list();
    for (const auto & name : task_names) {
      if (std::find(available.begin(), available.end(), name) == available.end()) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Task '%s' is not registered. Available: [%s]",
          name.c_str(),
          [&]() {
            std::string joined;
            for (std::size_t i = 0; i < available.size(); ++i) {
              if (i) joined += ", ";
              joined += available[i];
            }
            return joined;
          }().c_str());
        publish_progress("failed: unknown task '" + name + "'");
        return;
      }
    }

    current_task_index_ = 0;
    current_task_.reset();
    publish_progress("started: " + std::to_string(task_names.size()) + " tasks");

    // Tick the mission at 20 Hz
    mission_timer_ = this->create_wall_timer(
      50ms,
      [this, timeout_sec]() { this->tick(timeout_sec); });
  }

  void tick(double per_task_timeout_sec)
  {
    if (current_task_index_ >= task_list_.size()) {
      // Mission complete
      publish_progress("completed: " + std::to_string(task_list_.size()) + " tasks");
      mission_timer_->cancel();
      return;
    }

    // Lazy-init the current task
    if (!current_task_) {
      const std::string & name = task_list_[current_task_index_];
      current_task_ = vwpc::TaskRegistry::instance().create(name, this);
      if (!current_task_) {
        publish_progress("failed: factory for '" + name + "' returned null");
        mission_timer_->cancel();
        return;
      }

      vwpc::TaskContext ctx;
      ctx.node = this;
      ctx.timeout_sec = per_task_timeout_sec;
      ctx.task_index = current_task_index_;
      task_start_time_ = this->now();
      const auto init_status = current_task_->on_init(ctx);
      if (init_status != vwpc::TaskStatus::RUNNING &&
          init_status != vwpc::TaskStatus::SUCCESS)
      {
        publish_progress(
          "failed: " + name + " on_init returned " +
          vwpc::to_string(init_status));
        mission_timer_->cancel();
        return;
      }
      RCLCPP_INFO(
        this->get_logger(), "Task %zu/%zu: %s (init OK)",
        current_task_index_ + 1, task_list_.size(), name.c_str());

      // init_status == SUCCESS means single-step task done; fall through
      if (init_status == vwpc::TaskStatus::SUCCESS) {
        current_task_->on_cleanup();
        current_task_.reset();
        ++current_task_index_;
        return;
      }
    }

    // Execute one tick of the current task
    const auto status = current_task_->execute();

    // Check timeout
    if (per_task_timeout_sec > 0.0) {
      const double elapsed = (this->now() - task_start_time_).seconds();
      if (elapsed > per_task_timeout_sec) {
        publish_progress(
          "timeout: " + task_list_[current_task_index_] +
          " (elapsed " + std::to_string(elapsed) + "s)");
        current_task_->on_cleanup();
        current_task_.reset();
        mission_timer_->cancel();
        return;
      }
    }

    if (status == vwpc::TaskStatus::RUNNING) {
      return;  // keep ticking
    }

    // Task finished (either SUCCESS / FAILED / TIMEOUT)
    publish_progress(
      std::string(vwpc::to_string(status)) + ": " + task_list_[current_task_index_]);
    RCLCPP_INFO(
      this->get_logger(), "Task %s: %s",
      task_list_[current_task_index_].c_str(), vwpc::to_string(status));
    current_task_->on_cleanup();
    current_task_.reset();

    if (status == vwpc::TaskStatus::FAILED ||
        status == vwpc::TaskStatus::TIMEOUT)
    {
      mission_timer_->cancel();
      return;
    }

    ++current_task_index_;
  }

  void publish_progress(const std::string & msg)
  {
    auto m = std::make_unique<std_msgs::msg::String>();
    m->data = msg;
    progress_pub_->publish(std::move(m));
    RCLCPP_INFO(this->get_logger(), "mission: %s", msg.c_str());
  }

  // Cached after start_mission() so the tick() lambda doesn't have to
  // re-read the parameter every iteration.
  std::vector<std::string> task_list_;
  std::size_t current_task_index_{0};
  std::unique_ptr<vwpc::BaseTask> current_task_;
  rclcpp::Time task_start_time_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr progress_pub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
