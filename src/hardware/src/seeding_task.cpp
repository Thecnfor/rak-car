// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SeedingTask — 三站种植任务, 行为层第一个真任务。
//
// 从 stub 升级为真任务: 通过 BehaviorClient 指挥底盘 (ChassisNavigate →
// /rak/cmd/vel_raw → safety_gate → vel_safe → chassis) 和机械臂
// (ArmCartesianMove → ros2_control → MC602)。不再直接发 /cmd/vel_safe,
// 安全门永远在环内 (见 CLAUDE.md Critical warning #1 的架构约定)。
//
// 每站流程 (状态机, 非阻塞 tick 驱动):
//   1. drive_to_pose   : 底盘开到站点的 (x, y, theta), odom 系
//   2. arm_plant       : 手臂到种植位姿, 松手 (gripper=2)
//   3. arm_retract     : 手臂回缩位, 夹紧 (gripper=1)
//   4. 下一站; 全部完成 → SUCCESS
//
// 参数 (由 launch 覆盖):
//   seeding_stations  : 站点 [x, y, theta]*N, 单位 m / rad
//   seeding_arm_poses : 种植位姿 [x_mm, z_mm, yaw_deg]*N (缺省用站点索引回退)
//   arm_home          : 回缩位 [x_mm, z_mm, yaw_deg]
//
// Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §组件模型

#include "hardware/base_task.hpp"
#include "hardware/behavior_client.hpp"
#include "hardware/task_registry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace vwh = hardware;

class SeedingTask : public vwh::BaseTask
{
public:
  explicit SeedingTask(rclcpp::Node * node)
  : node_(node)
  {
  }

  std::string name() const override { return "seeding"; }

  vwh::TaskStatus on_init(const vwh::TaskContext & ctx) override
  {
    // 站点: [x, y, theta] 三元组 (odom 系, m/rad)。
    const auto st = node_->declare_parameter<std::vector<double>>(
      "seeding_stations", std::vector<double>{});
    const auto arm_poses = node_->declare_parameter<std::vector<double>>(
      "seeding_arm_poses", std::vector<double>{});
    const auto home = node_->declare_parameter<std::vector<double>>(
      "arm_home", std::vector<double>{100.0, 180.0, 0.0});

    if (st.size() < 3 || st.size() % 3 != 0) {
      RCLCPP_ERROR(node_->get_logger(), "SeedingTask: bad seeding_stations (need [x,y,theta]*N)");
      return vwh::TaskStatus::FAILED;
    }
    for (std::size_t i = 0; i + 2 < st.size(); i += 3) {
      stations_.push_back(vwh::Waypoint{st[i], st[i + 1], st[i + 2]});
    }
    if (arm_poses.size() >= 3 && arm_poses.size() % 3 == 0) {
      for (std::size_t i = 0; i + 2 < arm_poses.size(); i += 3) {
        arm_poses_.push_back({arm_poses[i], arm_poses[i + 1], arm_poses[i + 2]});
      }
    }
    home_ = home.size() >= 3
      ? ArmPose{home[0], home[1], home[2]}
      : ArmPose{100.0, 180.0, 0.0};

    behavior_ = std::make_unique<vwh::BehaviorClient>(node_);
    RCLCPP_INFO(node_->get_logger(),
      "SeedingTask initialized: %zu stations, timeout=%.1fs, backends=[%s]",
      stations_.size(), ctx.timeout_sec, behavior_->backend_report().c_str());
    return vwh::TaskStatus::RUNNING;
  }

  vwh::TaskStatus execute() override
  {
    switch (phase_) {
      case Phase::kDriveToStation:
        return run_drive();
      case Phase::kArmPlant:
        return run_arm(arm_pose_for(station_idx_), 2 /* release/plant */);
      case Phase::kArmRetract:
        return run_arm(home_, 1 /* grip */);
      case Phase::kDone:
        return vwh::TaskStatus::SUCCESS;
    }
    return vwh::TaskStatus::FAILED;
  }

  void on_cleanup() override
  {
    behavior_.reset();
    RCLCPP_INFO(node_->get_logger(), "SeedingTask cleaned up");
  }

  std::string info() const override
  {
    return "seeding: " + std::to_string(stations_.size()) +
      " stations, drive + plant + retract each";
  }

private:
  struct ArmPose
  {
    double x_mm{0.0};
    double z_mm{0.0};
    double yaw_deg{0.0};
  };

  ArmPose arm_pose_for(std::size_t i) const
  {
    if (!arm_poses_.empty()) {
      return arm_poses_[std::min(i, arm_poses_.size() - 1)];
    }
    return home_;  // 无种植位姿参数时, 用回缩位做占位 (至少验证链路通)。
  }

  vwh::TaskStatus run_drive()
  {
    if (!started_) {
      if (!behavior_->start_drive_to_pose(stations_[station_idx_], 15.0)) {
        RCLCPP_ERROR(node_->get_logger(), "SeedingTask: drive start failed: %s",
                     behavior_->last_error().c_str());
        return vwh::TaskStatus::FAILED;
      }
      started_ = true;
      return vwh::TaskStatus::RUNNING;
    }
    const auto r = behavior_->poll();
    if (r.running()) { return vwh::TaskStatus::RUNNING; }
    started_ = false;
    if (!r.ok()) {
      RCLCPP_ERROR(node_->get_logger(), "SeedingTask: drive to station %zu failed: %s",
                   station_idx_, r.error.c_str());
      return vwh::TaskStatus::FAILED;
    }
    RCLCPP_INFO(node_->get_logger(), "SeedingTask: arrived at station %zu", station_idx_);
    phase_ = Phase::kArmPlant;
    return vwh::TaskStatus::RUNNING;
  }

  vwh::TaskStatus run_arm(const ArmPose & pose, uint8_t gripper)
  {
    if (!started_) {
      if (!behavior_->start_arm_move_to(pose.x_mm, pose.z_mm, pose.yaw_deg,
                                        gripper, 15.0)) {
        RCLCPP_ERROR(node_->get_logger(), "SeedingTask: arm start failed: %s",
                     behavior_->last_error().c_str());
        return vwh::TaskStatus::FAILED;
      }
      started_ = true;
      return vwh::TaskStatus::RUNNING;
    }
    const auto r = behavior_->poll();
    if (r.running()) { return vwh::TaskStatus::RUNNING; }
    started_ = false;
    if (!r.ok()) {
      RCLCPP_ERROR(node_->get_logger(), "SeedingTask: arm op failed: %s", r.error.c_str());
      return vwh::TaskStatus::FAILED;
    }
    if (phase_ == Phase::kArmPlant) {
      phase_ = Phase::kArmRetract;
      return vwh::TaskStatus::RUNNING;
    }
    // kArmRetract 完成 → 下一站或收尾。
    ++station_idx_;
    if (station_idx_ >= stations_.size()) {
      phase_ = Phase::kDone;
      RCLCPP_INFO(node_->get_logger(), "SeedingTask: all %zu stations done", station_idx_);
      return vwh::TaskStatus::RUNNING;
    }
    phase_ = Phase::kDriveToStation;
    return vwh::TaskStatus::RUNNING;
  }

  enum class Phase : uint8_t { kDriveToStation = 0, kArmPlant, kArmRetract, kDone };

  rclcpp::Node * node_;
  std::unique_ptr<vwh::BehaviorClient> behavior_;
  std::vector<vwh::Waypoint> stations_;
  std::vector<ArmPose> arm_poses_;
  ArmPose home_;
  std::size_t station_idx_{0};
  Phase phase_{Phase::kDriveToStation};
  bool started_{false};
};

// Self-registration: at program startup, register "seeding" -> SeedingTask factory
REGISTER_TASK("seeding", SeedingTask)
