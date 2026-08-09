// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BehaviorClient — task-facing behavior adapters over the rak action contract.
//
// WHY: 任务层 (BaseTask) 需要"去某个位姿 / 走一串航点 / 手臂到位"这类高级
// 行为,但底层有多套实现。这一层把"选哪套后端"藏在接口后面,任务只面对一个
// 稳定 API,协议层 (action 定义 / 话题) 一律不动。
//
// 后端选择 (构造时按环境解析,无栈时自动降级):
//   Chassis
//     - kNav2   : nav2_msgs/action/FollowWaypoints (Orin 真栈, 高级)
//     - kLocalPid: 本地 ChassisNavigate action (单点 P 环, 无 nav2 降级)
//   Arm
//     - kMoveIt : moveit_msgs move_group 路径 (Orin, 后续增量接入)
//     - kLocalIk : 本地 ArmCartesianMove action (闭式 IK, 无 moveit 降级)
//
// 关键设计: 非阻塞 tick 驱动。BaseTask::execute() 每 50ms 被调一次,execute()
// 里不能阻塞等 action 响应(单线程 executor 会死锁——响应回调由同一 executor
// 处理)。所以 BehaviorClient 提供 start_*() 异步发起 + poll() 每次 tick 查询,
// 与任务状态机完美咬合。
//
// 生命周期: 任务在 on_init() 里构造(拿到 rclcpp::Node*), 在 on_cleanup()
// 里释放。action clients 在该节点 executor 的空隙里处理响应。

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Forward declarations (rclcpp-free header 保持可单测, 同 base_task.hpp 风格).
namespace rclcpp
{
class Node;
}

namespace hardware
{

// 行为调用结果 — 底盘/机械臂共用, 任务按 status 分支。
struct BehaviorResult
{
  enum class Status : uint8_t
  {
    RUNNING = 0,   // 进行中, 任务应继续返回 RUNNING
    SUCCESS = 1,   // 成功到达/完成
    FAILED  = 2,   // 失败 (不可达/超时/取消/无后端)
    NO_STACK = 3,  // 目标后端不可用且无降级路径
  };

  Status status{Status::RUNNING};
  std::string error;

  bool running() const { return status == Status::RUNNING; }
  bool ok() const { return status == Status::SUCCESS; }
};

// 底盘航点 — odom 系, theta 单位弧度。
struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

class BehaviorClient
{
public:
  enum class ChassisBackend : uint8_t { kNone = 0, kLocalPid, kNav2 };
  enum class ArmBackend : uint8_t { kNone = 0, kLocalIk, kMoveIt };

  // node: 任务的宿主机 (mission_runner_node)。只存指针, 不长期持有。
  // action 名/超时走参数, 允许 launch 覆盖; 后端探测是尽力而为——
  // 起不来就降级, 不 throw (任务不该因导航栈未装而挂掉)。
  explicit BehaviorClient(rclcpp::Node * node);

  // ---- Chassis -----------------------------------------------------------
  // 沿 odom 系航点序列行驶 (最后一段结束时对齐 theta)。
  // 返回 false 仅在参数非法时; 真正的失败由 poll() 报告。
  bool start_follow_waypoints(const std::vector<Waypoint> & waypoints,
                              double timeout_sec);

  // 单点到位 + 对齐朝向 (use_pose 语义, 与 nav2 NavigateToPose 等价)。
  bool start_drive_to_pose(const Waypoint & target, double timeout_sec);

  // ---- Arm --------------------------------------------------------------
  // 任务空间到位: x/z 单位 mm, yaw_deg 单位度, gripper: 0=不动 1=抓 2=放。
  bool start_arm_move_to(double x_mm, double z_mm, double yaw_deg,
                         uint8_t gripper_action, double timeout_sec);

  // ---- 通用 -------------------------------------------------------------
  // 每次任务 tick 调用一次。返回 RUNNING 表示还在跑; SUCCESS/FAILED 终结。
  BehaviorResult poll();

  // 取消当前行为 (幂等)。
  void cancel();

  // 当前生效的后端, 用于日志 / info() 展示。
  std::string backend_report() const;

  // 最近一次失败/拒绝的原因 (start_* 返回 false 或 poll() 返回 FAILED 后读)。
  std::string last_error() const;

private:
  // 后端解析 (构造时调用一次)。
  void resolve_backends();

  // 各后端的内部实现。
  bool start_nav2_waypoints(const std::vector<Waypoint> & waypoints,
                            double timeout_sec);
  bool start_local_waypoints(const std::vector<Waypoint> & waypoints,
                             double timeout_sec);
  bool start_local_pose(const Waypoint & target, double timeout_sec);
  bool start_local_arm(double x_mm, double z_mm, double yaw_deg,
                       uint8_t gripper_action, double timeout_sec);

  BehaviorResult poll_nav2();
  BehaviorResult poll_local_chassis();
  BehaviorResult poll_local_arm();

  rclcpp::Node * node_{nullptr};
  ChassisBackend chassis_{ChassisBackend::kNone};
  ArmBackend arm_{ArmBackend::kNone};

  // 动作客户端 (惰性创建, 指针持有以保持头文件 rclcpp-free)。
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace hardware
