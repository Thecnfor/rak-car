// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BehaviorClient — see behavior_client.hpp. 非阻塞 tick 驱动的行为适配层。

#include "hardware/behavior_client.hpp"
#include "hardware/arm_cartesian_planner.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "msgs/action/arm_cartesian_move.hpp"
#include "msgs/action/chassis_navigate.hpp"

// nav2 是可选栈: 有 RAK_HAVE_NAV2 (CMake 在找到 nav2_msgs 时定义) 才编译
// FollowWaypoints 路径, 否则降级到本地 ChassisNavigate。见 CLAUDE.md 的
// "可选依赖" 模式 (ros2_control QUIET 同款)。
#if defined(RAK_HAVE_NAV2)
#include <nav2_msgs/action/follow_waypoints.hpp>
#endif
// moveit 同理 (moveit_msgs 存在时编译 MoveGroup 后端, 否则降级本地 IK)。
#if defined(RAK_HAVE_MOVEIT)
#include <moveit_msgs/action/move_group.hpp>
#endif

#include <chrono>
#include <cmath>
#include <memory>
#include <type_traits>
#include <utility>

using namespace std::chrono_literals;

namespace hardware
{

namespace
{

constexpr char kChassisNavigateAction[] = "/rak/control/chassis/navigate";
constexpr char kArmCartesianMoveAction[] = "/rak/control/arm/cartesian_move";
#if defined(RAK_HAVE_NAV2)
constexpr char kFollowWaypointsAction[] = "/follow_waypoints";
#endif

// 每 tick 探测后端时给 action server 的等待上限。
constexpr auto kProbeTimeout = 200ms;

using ChassisClient = rclcpp_action::Client<msgs::action::ChassisNavigate>;
using ChassisGoalHandle = rclcpp_action::ClientGoalHandle<msgs::action::ChassisNavigate>;
using ArmClient = rclcpp_action::Client<msgs::action::ArmCartesianMove>;
using ArmGoalHandle = rclcpp_action::ClientGoalHandle<msgs::action::ArmCartesianMove>;
#if defined(RAK_HAVE_NAV2)
using Nav2Client = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>;
using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
#endif
#if defined(RAK_HAVE_MOVEIT)
constexpr char kMoveGroupAction[] = "/move_group";
using MoveItClient = rclcpp_action::Client<moveit_msgs::action::MoveGroup>;
using MoveItGoalHandle = rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>;
#endif

}  // anonymous namespace

// Humble 的 ClientGoalHandle 有 is_accepted(); Jazzy+/Lyrical 没有——拒绝时
// goal_response_callback 收到 nullptr。用 SFINAE 检测该成员存在性, 两端都编译。
namespace
{
template <typename T, typename = void>
struct HasIsAccepted : std::false_type {};
template <typename T>
struct HasIsAccepted<T, std::void_t<decltype(std::declval<T>().is_accepted())>>
: std::true_type {};

template <typename GoalHandleT>
bool goal_was_accepted(const std::shared_ptr<GoalHandleT> & gh)
{
  if (!gh) { return false; }
  if constexpr (HasIsAccepted<GoalHandleT>::value) {
    return gh->is_accepted();
  }
  return true;  // 新 API: 非空句柄即已接受 (reject → nullptr)
}
}  // anonymous namespace

// 当前在跑的单个操作 + 其状态机。
struct BehaviorClient::Impl
{
  enum class Op : uint8_t { kNone = 0, kNav2, kLocalChassis, kLocalArm, kMoveIt };

  Op op{Op::kNone};

  // action clients (构造时建, 探测后留在成员里)。
  std::shared_ptr<ChassisClient> local_chassis;
  std::shared_ptr<ArmClient> local_arm;
#if defined(RAK_HAVE_NAV2)
  std::shared_ptr<Nav2Client> nav2;
#endif
#if defined(RAK_HAVE_MOVEIT)
  std::shared_ptr<MoveItClient> moveit;
#endif

  // 本地底盘顺序航点队列。
  std::vector<Waypoint> pending_wps;
  std::size_t wp_idx{0};

  // 机械臂航点路径队列 (goal→waypoint→goal, 顺序驱动)。
  std::vector<ArmWaypoint> pending_arm_wps;
  std::size_t arm_wp_idx{0};
  bool arm_path_has_deadline{false};
  std::chrono::steady_clock::time_point arm_path_deadline{};

  // 单次 goal 的状态。
  bool goal_responded{false};
  bool goal_accepted{false};
  bool have_result{false};
  bool succeeded{false};
  std::string error;

  // goal handle (取消用)。
  std::shared_ptr<ChassisGoalHandle> local_chassis_gh;
  std::shared_ptr<ArmGoalHandle> local_arm_gh;
#if defined(RAK_HAVE_NAV2)
  std::shared_ptr<Nav2GoalHandle> nav2_gh;
#endif
#if defined(RAK_HAVE_MOVEIT)
  std::shared_ptr<MoveItGoalHandle> moveit_gh;
#endif

  // 一次操作结束 → 清空为 kNone, 让下一个 start_* 干净开始。
  void reset_op()
  {
    op = Op::kNone;
    pending_wps.clear();
    wp_idx = 0;
    pending_arm_wps.clear();
    arm_wp_idx = 0;
    arm_path_has_deadline = false;
    arm_path_deadline = {};
    goal_responded = false;
    goal_accepted = false;
    have_result = false;
    succeeded = false;
    error.clear();
    local_chassis_gh.reset();
    local_arm_gh.reset();
#if defined(RAK_HAVE_NAV2)
    nav2_gh.reset();
#endif
#if defined(RAK_HAVE_MOVEIT)
    moveit_gh.reset();
#endif
  }
};

BehaviorClient::BehaviorClient(rclcpp::Node * node)
: node_(node), impl_(std::make_shared<Impl>())
{
  resolve_backends();
}

void BehaviorClient::resolve_backends()
{
  if (node_ == nullptr) {
    return;
  }
  impl_->local_chassis = rclcpp_action::create_client<msgs::action::ChassisNavigate>(
    node_, kChassisNavigateAction);
  impl_->local_arm = rclcpp_action::create_client<msgs::action::ArmCartesianMove>(
    node_, kArmCartesianMoveAction);

  if (impl_->local_chassis->wait_for_action_server(kProbeTimeout)) {
    chassis_ = ChassisBackend::kLocalPid;
  }
  if (impl_->local_arm->wait_for_action_server(kProbeTimeout)) {
    arm_ = ArmBackend::kLocalIk;
  }

#if defined(RAK_HAVE_MOVEIT)
  impl_->moveit = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
    node_, kMoveGroupAction);
  // moveit 优先: 有 move_group 就用规划栈, 否则保持本地闭式 IK。
  if (impl_->moveit->wait_for_action_server(kProbeTimeout)) {
    arm_ = ArmBackend::kMoveIt;
  }
#endif

#if defined(RAK_HAVE_NAV2)
  impl_->nav2 = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    node_, kFollowWaypointsAction);
  // nav2 优先: 有真栈就用高级路径, 否则保持本地 P 环。
  if (impl_->nav2->wait_for_action_server(kProbeTimeout)) {
    chassis_ = ChassisBackend::kNav2;
  }
#endif
}

// ---------------------------------------------------------------------------
// 底盘
// ---------------------------------------------------------------------------

bool BehaviorClient::start_follow_waypoints(const std::vector<Waypoint> & waypoints,
                                            double timeout_sec)
{
  if (waypoints.empty()) {
    if (impl_) { impl_->error = "empty waypoint list"; }
    return false;
  }
  switch (chassis_) {
    case ChassisBackend::kNav2:
      return start_nav2_waypoints(waypoints, timeout_sec);
    case ChassisBackend::kLocalPid:
      return start_local_waypoints(waypoints, timeout_sec);
    case ChassisBackend::kNone:
    default:
      if (impl_) { impl_->error = "no chassis backend available"; }
      return false;
  }
}

bool BehaviorClient::start_drive_to_pose(const Waypoint & target, double timeout_sec)
{
  switch (chassis_) {
    case ChassisBackend::kNav2: {
      // nav2 单点 = 单航点 FollowWaypoints。
      return start_nav2_waypoints(std::vector<Waypoint>{target}, timeout_sec);
    }
    case ChassisBackend::kLocalPid:
      return start_local_pose(target, timeout_sec);
    case ChassisBackend::kNone:
    default:
      if (impl_) { impl_->error = "no chassis backend available"; }
      return false;
  }
}

bool BehaviorClient::start_nav2_waypoints(const std::vector<Waypoint> & waypoints,
                                          double timeout_sec)
{
#if defined(RAK_HAVE_NAV2)
  if (!impl_ || !impl_->nav2 || node_ == nullptr) {
    return false;
  }
  auto goal = nav2_msgs::action::FollowWaypoints::Goal();
  // Humble FollowWaypoints 只有 poses 字段 (无 number_of_loops)。
  goal.poses.resize(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    goal.poses[i].header.frame_id = "odom";
    goal.poses[i].header.stamp = node_->now();
    goal.poses[i].pose.position.x = waypoints[i].x;
    goal.poses[i].pose.position.y = waypoints[i].y;
    goal.poses[i].pose.orientation.z = std::sin(waypoints[i].theta / 2.0);
    goal.poses[i].pose.orientation.w = std::cos(waypoints[i].theta / 2.0);
  }
  (void)timeout_sec;  // nav2 侧超时由 FollowWaypoints 行为树/参数控制

  impl_->op = Impl::Op::kNav2;
  impl_->goal_responded = false;
  impl_->goal_accepted = false;
  impl_->have_result = false;
  impl_->succeeded = false;
  impl_->error.clear();

  auto client = impl_->nav2;
  auto impl = impl_;
  typename Nav2Client::SendGoalOptions options;
  options.goal_response_callback =
    [client, impl](typename Nav2GoalHandle::SharedPtr gh) {
      impl->goal_responded = true;
      if (goal_was_accepted(gh)) {
        impl->goal_accepted = true;
        impl->nav2_gh = gh;
        client->async_get_result(
          gh,
          [impl](const typename Nav2Client::WrappedResult & wr) {
            impl->have_result = true;
            // Humble FollowWaypoints 结果只有 missed_waypoints:
            // 空 = 全部航点到达; 非空 = 未到达的航点下标。
            const bool ok =
              (wr.code == rclcpp_action::ResultCode::SUCCEEDED &&
               wr.result != nullptr && wr.result->missed_waypoints.empty());
            impl->succeeded = ok;
            if (!ok) {
              const std::size_t missed =
                wr.result ? wr.result->missed_waypoints.size() : 0;
              impl->error = "nav2 FollowWaypoints: " +
                std::to_string(missed) + " waypoint(s) missed";
            }
          });
      } else {
        impl->goal_accepted = false;
        impl->succeeded = false;
        impl->error = "nav2 FollowWaypoints goal rejected";
      }
    };
  client->async_send_goal(goal, options);
  return true;
#else
  (void)waypoints;
  (void)timeout_sec;
  if (impl_) { impl_->error = "nav2 support not compiled in"; }
  return false;
#endif
}

bool BehaviorClient::start_local_waypoints(const std::vector<Waypoint> & waypoints,
                                           double timeout_sec)
{
  if (!impl_ || impl_->pending_wps.empty() == false) {
    return false;  // 上一次还没结束
  }
  impl_->pending_wps = waypoints;
  impl_->wp_idx = 0;
  return start_local_pose(impl_->pending_wps[0], timeout_sec);
}

bool BehaviorClient::start_local_pose(const Waypoint & target, double timeout_sec)
{
  if (!impl_ || !impl_->local_chassis || node_ == nullptr) {
    if (impl_) { impl_->error = "no local chassis client"; }
    return false;
  }

  auto goal = msgs::action::ChassisNavigate::Goal();
  goal.use_pose = true;
  goal.target_pose.x = static_cast<float>(target.x);
  goal.target_pose.y = static_cast<float>(target.y);
  goal.target_pose.theta = static_cast<float>(target.theta);
  // 容差/限速走参数, 由 launch 覆盖; 这里给保守默认。
  goal.linear_tolerance = static_cast<float>(node_->get_parameter_or("linear_tolerance", 0.03));
  goal.angular_tolerance = static_cast<float>(node_->get_parameter_or("angular_tolerance", 0.05));
  goal.timeout_sec = static_cast<float>(timeout_sec);

  impl_->op = Impl::Op::kLocalChassis;
  impl_->goal_responded = false;
  impl_->goal_accepted = false;
  impl_->have_result = false;
  impl_->succeeded = false;
  impl_->error.clear();

  auto client = impl_->local_chassis;
  auto impl = impl_;
  typename ChassisClient::SendGoalOptions options;
  options.goal_response_callback =
    [client, impl](typename ChassisGoalHandle::SharedPtr gh) {
      impl->goal_responded = true;
      if (goal_was_accepted(gh)) {
        impl->goal_accepted = true;
        impl->local_chassis_gh = gh;
        client->async_get_result(
          gh,
          [impl](const typename ChassisClient::WrappedResult & wr) {
            impl->have_result = true;
            impl->succeeded =
              (wr.code == rclcpp_action::ResultCode::SUCCEEDED &&
               wr.result->success);
            if (!impl->succeeded) {
              impl->error = "chassis navigate: " +
                (wr.result ? wr.result->error : std::string("no result"));
            }
          });
      } else {
        impl->goal_accepted = false;
        impl->succeeded = false;
        impl->error = "chassis navigate goal rejected";
      }
    };
  client->async_send_goal(goal, options);
  return true;
}

// ---------------------------------------------------------------------------
// 机械臂
// ---------------------------------------------------------------------------

bool BehaviorClient::start_arm_move_to(double x_mm, double z_mm, double yaw_deg,
                                       uint8_t gripper_action, double timeout_sec)
{
  switch (arm_) {
    case ArmBackend::kLocalIk: {
      if (!impl_ || !impl_->local_arm || node_ == nullptr) {
        if (impl_) { impl_->error = "no local arm client"; }
        return false;
      }
      auto goal = msgs::action::ArmCartesianMove::Goal();
      goal.frame_id = "arm_base";
      goal.x = static_cast<float>(x_mm);
      goal.z = static_cast<float>(z_mm);
      goal.yaw_deg = static_cast<float>(yaw_deg);
      goal.y_enabled = false;
      goal.y = 0.0f;
      goal.gripper_action = gripper_action;
      goal.velocity_scale = 0.5f;
      goal.position_tolerance = 5.0f;
      goal.timeout_sec = static_cast<float>(timeout_sec);

      impl_->op = Impl::Op::kLocalArm;
      impl_->goal_responded = false;
      impl_->goal_accepted = false;
      impl_->have_result = false;
      impl_->succeeded = false;
      impl_->error.clear();

      auto client = impl_->local_arm;
      auto impl = impl_;
      typename ArmClient::SendGoalOptions options;
      options.goal_response_callback =
        [client, impl](typename ArmGoalHandle::SharedPtr gh) {
          impl->goal_responded = true;
          if (goal_was_accepted(gh)) {
            impl->goal_accepted = true;
            impl->local_arm_gh = gh;
            client->async_get_result(
              gh,
              [impl](const typename ArmClient::WrappedResult & wr) {
                impl->have_result = true;
                impl->succeeded =
                  (wr.code == rclcpp_action::ResultCode::SUCCEEDED &&
                   wr.result->status == 0);  // 0 = REACHED
                if (!impl->succeeded) {
                  impl->error = "arm move: " +
                    (wr.result ? wr.result->error : std::string("no result"));
                }
              });
          } else {
            impl->goal_accepted = false;
            impl->succeeded = false;
            impl->error = "arm move goal rejected";
          }
        };
      client->async_send_goal(goal, options);
      return true;
    }
    case ArmBackend::kMoveIt: {
#if defined(RAK_HAVE_MOVEIT)
      if (!impl_ || !impl_->moveit || node_ == nullptr) {
        if (impl_) { impl_->error = "no moveit client"; }
        return false;
      }
      // 闭式 IK 出关节目标, moveit 做规划/校验/执行 (MoveGroup joint goal)。
      auto planner = ArmCartesianPlanner::make_default();
      const std::vector<double> seed(4, 0.0);
      auto plan = planner.plan("arm_base", x_mm, z_mm, yaw_deg,
                               false, 0.0, gripper_action,
                               0.5, 5.0, seed);
      if (plan.status != ArmCartesianPlanner::Plan::Status::OK) {
        impl_->error = "moveit IK failed: " + plan.error;
        return false;
      }

      auto goal = moveit_msgs::action::MoveGroup::Goal();
      goal.request.group_name = "arm";
      goal.request.allowed_planning_time = 2.0;
      goal.request.num_planning_attempts = 5;
      goal.request.max_velocity_scaling_factor = 0.5;
      moveit_msgs::msg::Constraints tc;
      static const std::vector<std::string> kJointNames = {
        "arm_horiz_joint", "arm_vert_joint",
        "arm_hand_rotate_joint", "arm_hand_grip_joint"};
      for (std::size_t i = 0; i < kJointNames.size(); ++i) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = kJointNames[i];
        jc.position = plan.q_target[i];
        jc.tolerance_above = 0.01;
        jc.tolerance_below = 0.01;
        jc.weight = 1.0;
        tc.joint_constraints.push_back(jc);
      }
      goal.request.goal_constraints.push_back(tc);

      impl_->op = Impl::Op::kMoveIt;
      impl_->goal_responded = false;
      impl_->goal_accepted = false;
      impl_->have_result = false;
      impl_->succeeded = false;
      impl_->error.clear();

      auto client = impl_->moveit;
      auto impl = impl_;
      typename MoveItClient::SendGoalOptions options;
      options.goal_response_callback =
        [client, impl](typename MoveItGoalHandle::SharedPtr gh) {
          impl->goal_responded = true;
          if (goal_was_accepted(gh)) {
            impl->goal_accepted = true;
            impl->moveit_gh = gh;
            client->async_get_result(
              gh,
              [impl](const typename MoveItClient::WrappedResult & wr) {
                impl->have_result = true;
                // MoveItErrorCodes: SUCCESS == 1
                impl->succeeded =
                  (wr.code == rclcpp_action::ResultCode::SUCCEEDED &&
                   wr.result->error_code.val == 1);
                if (!impl->succeeded) {
                  impl->error = "moveit move_group: error_code=" +
                    std::to_string(wr.result->error_code.val);
                }
              });
          } else {
            impl->goal_accepted = false;
            impl->succeeded = false;
            impl->error = "moveit move_group goal rejected";
          }
        };
      client->async_send_goal(goal, options);
      return true;
#else
      if (impl_) { impl_->error = "moveit support not compiled in"; }
      return false;
#endif
    }
    case ArmBackend::kNone:
    default:
      if (impl_) { impl_->error = "no arm backend available"; }
      return false;
  }
}

// ---------------------------------------------------------------------------
// 通用
// ---------------------------------------------------------------------------

BehaviorResult BehaviorClient::poll()
{
  if (!impl_ || impl_->op == Impl::Op::kNone) {
    return BehaviorResult{BehaviorResult::Status::SUCCESS};
  }

  switch (impl_->op) {
    case Impl::Op::kNav2:
      return poll_nav2();
    case Impl::Op::kLocalChassis:
      return poll_local_chassis();
    case Impl::Op::kLocalArm:
      return poll_local_arm();
    case Impl::Op::kMoveIt:
      return poll_moveit();
    case Impl::Op::kNone:
    default:
      return BehaviorResult{BehaviorResult::Status::SUCCESS};
  }
}

BehaviorResult BehaviorClient::poll_nav2()
{
#if defined(RAK_HAVE_NAV2)
  if (!impl_->goal_responded) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (!impl_->goal_accepted) {
    const auto err = impl_->error;
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, err};
  }
  if (!impl_->have_result) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (impl_->succeeded) {
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::SUCCESS};
  }
  const auto err = impl_->error;
  impl_->reset_op();
  return BehaviorResult{BehaviorResult::Status::FAILED, err};
#else
  const auto err = impl_->error;
  impl_->reset_op();
  return BehaviorResult{BehaviorResult::Status::NO_STACK, err};
#endif
}

BehaviorResult BehaviorClient::poll_local_chassis()
{
  if (!impl_->goal_responded) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (!impl_->goal_accepted) {
    const auto err = impl_->error;
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, err};
  }
  if (!impl_->have_result) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (!impl_->succeeded) {
    const auto err = impl_->error;
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, err};
  }

  // 当前航点完成 → 若有下一个, 自动推进; 否则整串结束。
  if (impl_->wp_idx + 1 < impl_->pending_wps.size()) {
    ++impl_->wp_idx;
    // start_local_pose 会重置单次 goal 状态并发起下一个航点。
    if (!start_local_pose(impl_->pending_wps[impl_->wp_idx], 15.0)) {
      const auto err = impl_->error;
      impl_->reset_op();
      return BehaviorResult{BehaviorResult::Status::FAILED, err};
    }
    return BehaviorResult{BehaviorResult::Status::RUNNING};
  }
  impl_->reset_op();
  return BehaviorResult{BehaviorResult::Status::SUCCESS};
}

bool BehaviorClient::start_arm_path(const std::vector<ArmWaypoint> & waypoints,
                                    double timeout_sec)
{
  if (waypoints.empty()) {
    if (impl_) { impl_->error = "empty arm waypoint list"; }
    return false;
  }
  impl_->pending_arm_wps = waypoints;
  impl_->arm_wp_idx = 0;
  impl_->arm_path_has_deadline = timeout_sec > 0.0;
  if (impl_->arm_path_has_deadline) {
    impl_->arm_path_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_sec));
  }
  const auto & wp = waypoints[0];
  if (!start_arm_move_to(wp.x_mm, wp.z_mm, wp.yaw_deg, wp.gripper, timeout_sec)) {
    const auto err = impl_->error;
    impl_->reset_op();
    impl_->error = err;
    return false;
  }
  return true;
}

bool BehaviorClient::advance_arm_path_if_pending()
{
  if (impl_->arm_wp_idx + 1 >= impl_->pending_arm_wps.size()) {
    return false;
  }
  if (impl_->arm_path_has_deadline) {
    const auto remaining = impl_->arm_path_deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::steady_clock::duration::zero()) {
      impl_->error = "arm path timeout";
      return false;
    }
  }
  ++impl_->arm_wp_idx;
  const auto & wp = impl_->pending_arm_wps[impl_->arm_wp_idx];
  double segment_timeout = 0.0;
  if (impl_->arm_path_has_deadline) {
    segment_timeout = std::chrono::duration<double>(
      impl_->arm_path_deadline - std::chrono::steady_clock::now()).count();
  }
  if (!start_arm_move_to(wp.x_mm, wp.z_mm, wp.yaw_deg, wp.gripper, segment_timeout)) {
    return false;
  }
  return true;
}

BehaviorResult BehaviorClient::poll_local_arm()
{
  if (impl_->arm_path_has_deadline &&
      std::chrono::steady_clock::now() >= impl_->arm_path_deadline) {
    if (impl_->local_arm_gh && impl_->local_arm) {
      impl_->local_arm->async_cancel_goal(impl_->local_arm_gh);
    }
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, "arm path timeout"};
  }
  if (!impl_->goal_responded) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (!impl_->goal_accepted) {
    const auto err = impl_->error;
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, err};
  }
  if (!impl_->have_result) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (impl_->succeeded) {
    if (impl_->arm_wp_idx + 1 < impl_->pending_arm_wps.size()) {
      if (advance_arm_path_if_pending()) {
        return BehaviorResult{BehaviorResult::Status::RUNNING};
      }
      const auto err = impl_->error.empty() ? "arm path advance failed" : impl_->error;
      impl_->reset_op();
      return BehaviorResult{BehaviorResult::Status::FAILED, err};
    }
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::SUCCESS};
  }
  const auto err = impl_->error;
  impl_->reset_op();
  return BehaviorResult{BehaviorResult::Status::FAILED, err};
}

BehaviorResult BehaviorClient::poll_moveit()
{
#if defined(RAK_HAVE_MOVEIT)
  if (impl_->arm_path_has_deadline &&
      std::chrono::steady_clock::now() >= impl_->arm_path_deadline) {
    if (impl_->moveit_gh && impl_->moveit) {
      impl_->moveit->async_cancel_goal(impl_->moveit_gh);
    }
    const auto err = std::string("arm path timeout");
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, err};
  }
  if (!impl_->goal_responded) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (!impl_->goal_accepted) {
    const auto err = impl_->error;
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::FAILED, err};
  }
  if (!impl_->have_result) { return BehaviorResult{BehaviorResult::Status::RUNNING}; }
  if (impl_->succeeded) {
    if (impl_->arm_wp_idx + 1 < impl_->pending_arm_wps.size()) {
      if (advance_arm_path_if_pending()) {
        return BehaviorResult{BehaviorResult::Status::RUNNING};
      }
      const auto err = impl_->error.empty() ? "arm path advance failed" : impl_->error;
      impl_->reset_op();
      return BehaviorResult{BehaviorResult::Status::FAILED, err};
    }
    impl_->reset_op();
    return BehaviorResult{BehaviorResult::Status::SUCCESS};
  }
  const auto err = impl_->error;
  impl_->reset_op();
  return BehaviorResult{BehaviorResult::Status::FAILED, err};
#else
  const auto err = impl_->error;
  impl_->reset_op();
  return BehaviorResult{BehaviorResult::Status::NO_STACK, err};
#endif
}

void BehaviorClient::cancel()
{
  if (!impl_ || impl_->op == Impl::Op::kNone) {
    return;
  }
  switch (impl_->op) {
    case Impl::Op::kNav2:
#if defined(RAK_HAVE_NAV2)
      if (impl_->nav2_gh && impl_->nav2) {
        impl_->nav2->async_cancel_goal(impl_->nav2_gh);
      }
#endif
      break;
    case Impl::Op::kLocalChassis:
      if (impl_->local_chassis_gh && impl_->local_chassis) {
        impl_->local_chassis->async_cancel_goal(impl_->local_chassis_gh);
      }
      break;
    case Impl::Op::kLocalArm:
      if (impl_->local_arm_gh && impl_->local_arm) {
        impl_->local_arm->async_cancel_goal(impl_->local_arm_gh);
      }
      break;
    case Impl::Op::kMoveIt:
#if defined(RAK_HAVE_MOVEIT)
      if (impl_->moveit_gh && impl_->moveit) {
        impl_->moveit->async_cancel_goal(impl_->moveit_gh);
      }
#endif
      break;
    case Impl::Op::kNone:
    default:
      break;
  }
  impl_->reset_op();
}

std::string BehaviorClient::backend_report() const
{
  std::string chassis;
  switch (chassis_) {
    case ChassisBackend::kNav2: chassis = "nav2(FollowWaypoints)"; break;
    case ChassisBackend::kLocalPid: chassis = "local(ChassisNavigate P)"; break;
    case ChassisBackend::kNone: chassis = "none"; break;
  }
  std::string arm;
  switch (arm_) {
    case ArmBackend::kLocalIk: arm = "local(ArmCartesianMove IK)"; break;
    case ArmBackend::kMoveIt: arm = "moveit(move_group)"; break;
    case ArmBackend::kNone: arm = "none"; break;
  }
  return "chassis=" + chassis + " arm=" + arm;
}

std::string BehaviorClient::last_error() const
{
  return impl_ ? impl_->error : std::string("no impl");
}

}  // namespace hardware
