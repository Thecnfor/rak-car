// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MissionRunnerNode — thin ROS2 shell around MissionRunner (the pure
// mission state machine in mission_runner.hpp).
//
// Reads a string[] param 'task_list' (e.g. ["seeding", "pest_scout",
// "harvest"]) and runs each task in sequence. All state-machine logic
// (ordering, timeouts, abort-on-fail) lives in MissionRunner, which is
// unit-tested with fake tasks. This node only declares parameters, wires
// the TaskRegistry factory + a progress publisher into the runner, and
// ticks it from a timer.
//
// Progress is published on /rak/state/mission_progress as
// plain strings: "started: N tasks", "SUCCESS: <task>", "completed: N",
// etc. (see mission_runner.hpp for the full format).
//
// Usage:
//   ros2 run hardware mission_runner_node --ros-args \
//     -p task_list:='[seeding, pest_scout, shoot_pest, harvest]'

#include "hardware/mission_runner.hpp"
#include "hardware/task_registry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace vwh = hardware;

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
      "/rak/state/mission_progress", 10);

    runner_ = std::make_unique<vwh::MissionRunner>(
      // Factory: resolve each task name via the registry.
      [this](const std::string & name) {
        return vwh::TaskRegistry::instance().create(name, this);
      },
      // Progress: publish + log every event.
      [this](const std::string & msg) { publish_progress(msg); });
    runner_->set_node(this);

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
    const auto available = vwh::TaskRegistry::instance().list();

    // start() reports the reason via the progress callback on failure.
    if (!runner_->start(task_names, timeout_sec, available)) {
      return;
    }

    // Tick the mission at 20 Hz; runner_->tick() reports done/abort.
    mission_timer_ = this->create_wall_timer(
      50ms,
      [this]() { runner_->tick(this->now().seconds()); });
  }

  void publish_progress(const std::string & msg)
  {
    auto m = std::make_unique<std_msgs::msg::String>();
    m->data = msg;
    progress_pub_->publish(std::move(m));
    RCLCPP_INFO(this->get_logger(), "mission: %s", msg.c_str());
  }

  std::unique_ptr<vwh::MissionRunner> runner_;
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
