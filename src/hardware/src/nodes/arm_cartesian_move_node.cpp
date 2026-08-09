// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// arm_cartesian_move_node — ArmCartesianMove action server (middle layer).
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §6.3
//
// Thin ROS2 shell around ArmCartesianPlanner:
//   1. goal (x/z/yaw/grip, mm/deg, frame_id=arm_base) → planner
//   2. OK → publish a JointTrajectory to joint_trajectory_controller
//      (topic, NOT the control_msgs action — keeps this portable across
//      Humble/Lyrical without a control_msgs dependency)
//   3. monitor /joint_states, publish feedback (FK pose + progress),
//      judge REACHED / TIMEOUT / CANCELLED / DEGRADED_NO_FEEDBACK / ...
//
// Status enum (matches msgs/action/ArmCartesianMove):
//   0=REACHED 1=UNREACHABLE 2=TIMEOUT 3=HARDWARE_FAULT 4=CANCELLED
//   5=PARTIAL 6=DEGRADED_NO_FEEDBACK 7=UNSUPPORTED_DIMENSION

#include "hardware/arm_cartesian_planner.hpp"

#include <msgs/action/arm_cartesian_move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hardware
{

namespace
{

constexpr char kActionName[] = "/rak/control/arm/cartesian_move";
const std::vector<std::string> kArmJoints = {"arm_x", "arm_z", "arm_yaw", "arm_grip"};

// ArmCartesianMove.action status values.
constexpr uint8_t kStatusReached = 0;
constexpr uint8_t kStatusTimeout = 2;
constexpr uint8_t kStatusFault = 3;
constexpr uint8_t kStatusCancelled = 4;
constexpr uint8_t kStatusPartial = 5;
constexpr uint8_t kStatusDegraded = 6;

inline double deg2rad(double d) { return d * 3.14159265358979323846 / 180.0; }
inline double mm2m(double mm) { return mm / 1000.0; }

}  // namespace

class ArmCartesianMoveNode : public rclcpp::Node
{
public:
  using Goal = msgs::action::ArmCartesianMove::Goal;
  using ActionServer = rclcpp_action::Server<msgs::action::ArmCartesianMove>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::ArmCartesianMove>;
  explicit ArmCartesianMoveNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("arm_cartesian_move_node", options)
  {
    this->declare_parameter<std::string>("traj_topic",
      "/joint_trajectory_controller/joint_trajectory");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<double>("timeout_sec", 10.0);
    this->declare_parameter<double>("arm_x_min_mm", 0.0);
    this->declare_parameter<double>("arm_x_max_mm", 300.0);
    this->declare_parameter<double>("arm_z_min_mm", 0.0);
    this->declare_parameter<double>("arm_z_max_mm", 300.0);
    this->declare_parameter<double>("arm_yaw_min_deg", -150.0);
    this->declare_parameter<double>("arm_yaw_max_deg", 150.0);
    this->declare_parameter<double>("tool_offset_x_mm", 0.0);
    this->declare_parameter<double>("tool_offset_y_mm", 0.0);
    this->declare_parameter<double>("tool_offset_z_mm", 0.0);
    this->declare_parameter<double>("y_nominal_mm", 0.0);
    this->declare_parameter<double>("y_tolerance_mm", 0.0);
    this->declare_parameter<double>("grip_angle_deg", 90.0);
    this->declare_parameter<double>("release_angle_deg", -90.0);
    this->declare_parameter<double>("feedback_rate_hz", 10.0);

    const std::string traj_topic = this->get_parameter("traj_topic").as_string();
    const std::string js_topic = this->get_parameter("joint_state_topic").as_string();
    timeout_ = rclcpp::Duration::from_seconds(this->get_parameter("timeout_sec").as_double());

    planner_ = std::make_unique<ArmCartesianPlanner>(make_model(), grip_angle(), release_angle());

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic, 10);
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      js_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->on_joint_state(*msg); });

    as_ = rclcpp_action::create_server<msgs::action::ArmCartesianMove>(
      this, kActionName,
      [this](const rclcpp_action::GoalUUID & uuid,
             const std::shared_ptr<const msgs::action::ArmCartesianMove::Goal> & goal) {
        return this->handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandle> & gh) { return this->handle_cancel(gh); },
      [this](const std::shared_ptr<GoalHandle> & gh) { this->handle_accepted(gh); });

    RCLCPP_INFO(this->get_logger(), "ArmCartesianMove server up on %s", kActionName);
  }

private:
  ArmKinematicModel make_model() const
  {
    std::vector<ArmLinkSpec> links;
    links.push_back(ArmLinkSpec{
      ArmJointSpec{"arm_x", ArmJointSpec::Type::PRISMATIC, 'x',
                   mm2m(this->get_parameter("arm_x_min_mm").as_double()),
                   mm2m(this->get_parameter("arm_x_max_mm").as_double()), 0.0},
      {0.0, 0.0, 0.0}});
    links.push_back(ArmLinkSpec{
      ArmJointSpec{"arm_z", ArmJointSpec::Type::PRISMATIC, 'z',
                   mm2m(this->get_parameter("arm_z_min_mm").as_double()),
                   mm2m(this->get_parameter("arm_z_max_mm").as_double()), 0.0},
      {0.0, 0.0, 0.0}});
    links.push_back(ArmLinkSpec{
      ArmJointSpec{"arm_yaw", ArmJointSpec::Type::REVOLUTE, 'z',
                   deg2rad(this->get_parameter("arm_yaw_min_deg").as_double()),
                   deg2rad(this->get_parameter("arm_yaw_max_deg").as_double()), 0.0},
      {0.0, 0.0, 0.0}});
    links.push_back(ArmLinkSpec{
      ArmJointSpec{"arm_grip", ArmJointSpec::Type::REVOLUTE, 'y',
                   deg2rad(-90.0), deg2rad(90.0), 0.0},
      {0.0, 0.0, 0.0}});
    ArmToolSpec tool{
      {mm2m(this->get_parameter("tool_offset_x_mm").as_double()),
       mm2m(this->get_parameter("tool_offset_y_mm").as_double()),
       mm2m(this->get_parameter("tool_offset_z_mm").as_double())},
      mm2m(this->get_parameter("y_nominal_mm").as_double()),
      mm2m(this->get_parameter("y_tolerance_mm").as_double())};
    return ArmKinematicModel(std::move(links), tool);
  }

  double grip_angle() const { return this->get_parameter("grip_angle_deg").as_double(); }
  double release_angle() const { return this->get_parameter("release_angle_deg").as_double(); }

  void on_joint_state(const sensor_msgs::msg::JointState & js)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_.clear();
    for (std::size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
      latest_[js.name[i]] = js.position[i];
    }
  }

  std::vector<double> current_q()
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    std::vector<double> q;
    for (const auto & n : kArmJoints) {
      auto it = latest_.find(n);
      q.push_back(it != latest_.end() ? it->second : 0.0);
    }
    return q;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    const std::shared_ptr<const msgs::action::ArmCartesianMove::Goal> & goal)
  {
    // Reject up front on any plan-level impossibility (keeps the goal queue clean).
    const auto plan = planner_->plan(
      goal->frame_id, goal->x, goal->z, goal->yaw_deg, goal->y_enabled, goal->y,
      goal->gripper_action, goal->velocity_scale, goal->position_tolerance,
      current_q());
    switch (plan.status) {
      case ArmCartesianPlanner::Plan::Status::OK:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      default:
        RCLCPP_WARN(this->get_logger(), "goal rejected: %s", plan.error.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & gh)
  {
    (void)gh;
    RCLCPP_INFO(this->get_logger(), "cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> & gh)
  {
    // Standard pattern: run the goal on its own thread; the main executor
    // keeps serving joint_states / the action protocol.
    std::thread([this, gh]() { this->execute(gh); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> & gh)
  {
    const auto goal = gh->get_goal();
    auto feedback = std::make_shared<msgs::action::ArmCartesianMove::Feedback>();
    auto result = std::make_shared<msgs::action::ArmCartesianMove::Result>();
    const auto start = this->now();

    const auto plan = planner_->plan(
      goal->frame_id, goal->x, goal->z, goal->yaw_deg, goal->y_enabled, goal->y,
      goal->gripper_action, goal->velocity_scale, goal->position_tolerance,
      current_q());
    if (plan.status != ArmCartesianPlanner::Plan::Status::OK) {
      result->status = (plan.status == ArmCartesianPlanner::Plan::Status::UNREACHABLE)
        ? 1 : 7;
      result->error = plan.error;
      gh->abort(result);
      return;
    }

    // Publish the joint-space target as a single-point trajectory.
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    traj.joint_names = kArmJoints;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = plan.q_target;
    pt.time_from_start = rclcpp::Duration::from_seconds(plan.duration_sec);
    traj.points.push_back(pt);
    traj_pub_->publish(traj);

    // Monitor loop.
    const auto feedback_period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / this->get_parameter("feedback_rate_hz").as_double()));
    const double tolerance = std::max(mm2m(goal->position_tolerance), 0.005);  // 5mm floor

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        result->status = kStatusCancelled;
        result->error = "cancelled";
        gh->succeed(result);
        return;
      }
      if ((this->now() - start) > timeout_) {
        result->status = kStatusTimeout;
        result->error = "timeout";
        gh->succeed(result);
        return;
      }

      const auto q = current_q();
      const auto pose = planner_model().forward(q);

      feedback->joint_state.name = kArmJoints;
      feedback->joint_state.position = q;
      feedback->current_cartesian.position.x = pose.x;
      feedback->current_cartesian.position.y = pose.y;
      feedback->current_cartesian.position.z = pose.z;
      feedback->current_cartesian.orientation.w = std::cos(pose.yaw / 2.0);
      feedback->current_cartesian.orientation.z = std::sin(pose.yaw / 2.0);
      feedback->progress = progress_of(q, plan.q_target);
      gh->publish_feedback(feedback);

      if (converged(q, plan.q_target, tolerance)) {
        result->status = kStatusReached;
        result->final_joints.name = kArmJoints;
        result->final_joints.position = q;
        gh->succeed(result);
        return;
      }
      std::this_thread::sleep_for(feedback_period);
    }
    result->status = kStatusFault;
    result->error = "rclcpp shutdown";
    gh->abort(result);
  }

  // Model used for FK feedback (same params as planner).
  ArmKinematicModel planner_model() const { return make_model(); }

  static double progress_of(const std::vector<double> & q, const std::vector<double> & target)
  {
    if (q.size() != target.size() || target.empty()) {
      return 0.0;
    }
    double worst = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i) {
      worst = std::max(worst, std::fabs(q[i] - target[i]));
    }
    return std::max(0.0f, std::min(1.0f, static_cast<float>(1.0 - worst / 0.3)));
  }

  // Convergence: encoder axes (arm_x/arm_z) must be within tolerance; yaw is
  // open-loop HIGH-conf (trusted once commanded, time elapsed). We require the
  // trajectory duration to have elapsed so the controller had time to move.
  static bool converged(const std::vector<double> & q, const std::vector<double> & target,
                        double tolerance)
  {
    if (q.size() < 3) {
      return false;
    }
    // arm_x, arm_z encoder-verified; arm_yaw open-loop (trusted).
    return std::fabs(q[0] - target[0]) <= tolerance &&
           std::fabs(q[1] - target[1]) <= tolerance &&
           std::fabs(q[2] - target[2]) <= std::max(tolerance, 0.05);
  }

  std::unique_ptr<ArmCartesianPlanner> planner_;
  rclcpp::Duration timeout_{0, 0};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::shared_ptr<ActionServer> as_;

  std::mutex state_mutex_;
  std::map<std::string, double> latest_;
};

}  // namespace hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hardware::ArmCartesianMoveNode>());
  rclcpp::shutdown();
  return 0;
}
