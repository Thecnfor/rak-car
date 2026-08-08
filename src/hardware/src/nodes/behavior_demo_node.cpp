// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BehaviorDemoNode — 行为层编排示例:调组件 Action,发 TaskStatus。
// Spec: docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md §9
//
// 演示"行为层 = 订阅 + 调组件 Action + 发任务状态"的契约形态:
//   1. 调 ChassisNavigate(前进 0.5m)
//   2. 成功后调 ArmExecuteTrajectory(手爪合拢 grip_s7=+1)
//   3. 全程发 /rak/state/task/demo/status
//
// 用非阻塞 result_callback 状态机推进(不在 worker 线程嵌套 spin):
// 结果回调跑在 executor 线程,安全地推进相位并下发下一个 goal。

#include <msgs/action/arm_execute_trajectory.hpp>
#include <msgs/action/chassis_navigate.hpp>
#include <msgs/msg/task_status.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace
{
constexpr float kMoveMeters = 0.5f;       // 前进距离
constexpr float kGripPosition = 1.0f;     // grip_s7 = +1 → 合拢
}  // namespace

class BehaviorDemoNode : public rclcpp::Node
{
public:
  using NavAction = msgs::action::ChassisNavigate;
  using ArmAction = msgs::action::ArmExecuteTrajectory;
  using ClientNav = rclcpp_action::Client<NavAction>;
  using ClientArm = rclcpp_action::Client<ArmAction>;

  explicit BehaviorDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("behavior_demo_node", options)
  {
    status_pub_ = this->create_publisher<msgs::msg::TaskStatus>(
      "/rak/state/task/demo/status", rclcpp::QoS(1).transient_local());

    nav_client_ = rclcpp_action::create_client<NavAction>(
      this, "/rak/chassis/navigate");
    arm_client_ = rclcpp_action::create_client<ArmAction>(
      this, "/rak/arm/main/execute_trajectory");

    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/rak/behavior/demo/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
        std::lock_guard<std::mutex> lock(mu_);
        if (phase_ != Phase::IDLE) {
          resp->success = false;
          resp->message = "demo already running";
          return;
        }
        if (!nav_client_->wait_for_action_server(1s) ||
            !arm_client_->wait_for_action_server(1s)) {
          resp->success = false;
          resp->message = "action server(s) not available (chassis/arm)";
          return;
        }
        phase_ = Phase::NAVIGATE;
        publish_status("RUNNING", "navigate", 0.1f, "started");
        send_nav_goal();
        resp->success = true;
        resp->message = "demo started";
      });

    RCLCPP_INFO(this->get_logger(), "BehaviorDemoNode up — call /rak/behavior/demo/start");
  }

private:
  enum class Phase { IDLE, NAVIGATE, GRIP, DONE };

  void publish_status(const std::string & state, const std::string & step,
                      float progress, const std::string & msg)
  {
    msgs::msg::TaskStatus st;
    st.header.stamp = this->now();
    st.task_id = "demo";
    st.state = state;
    st.current_step = step;
    st.progress = progress;
    st.message = msg;
    status_pub_->publish(st);
    RCLCPP_INFO(this->get_logger(), "task=demo state=%s step=%s p=%.2f %s",
      state.c_str(), step.c_str(), progress, msg.c_str());
  }

  void send_nav_goal()
  {
    auto goal = NavAction::Goal();
    goal.target_pose.x = kMoveMeters;
    goal.target_pose.y = 0.0f;
    goal.target_pose.theta = 0.0f;
    goal.max_linear_speed = 0.2f;
    goal.max_angular_speed = 1.0f;
    goal.tolerance_lin = 0.02f;
    goal.tolerance_ang = 0.05f;
    goal.timeout_sec = 10.0f;

    typename ClientNav::SendGoalOptions opt;
    opt.result_callback =
      [this](const typename ClientNav::GoalHandle::WrappedResult & wr) {
        std::lock_guard<std::mutex> lock(mu_);
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
          publish_status("RUNNING", "grip", 0.5f, "nav done");
          phase_ = Phase::GRIP;
          send_arm_goal();
        } else {
          const std::string err =
            wr.result ? wr.result->error : "navigate aborted";
          publish_status("FAILED", "navigate", 0.3f, err);
          phase_ = Phase::DONE;
        }
      };
    nav_client_->async_send_goal(goal, opt);
  }

  void send_arm_goal()
  {
    auto goal = ArmAction::Goal();
    goal.arm_id = "main";
    goal.trajectory.joint_names = {
      "horiz_m6", "vert_stepper3", "rotate_s3", "grip_s7"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {0.0, 0.0, 0.0, kGripPosition};   // 手爪合拢
    goal.trajectory.points.push_back(pt);
    goal.max_execution_time = 5.0f;

    typename ClientArm::SendGoalOptions opt;
    opt.result_callback =
      [this](const typename ClientArm::GoalHandle::WrappedResult & wr) {
        std::lock_guard<std::mutex> lock(mu_);
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
          publish_status("SUCCEEDED", "done", 1.0f, "demo complete");
        } else {
          const std::string err = wr.result ? wr.result->error : "grip aborted";
          publish_status("FAILED", "grip", 0.7f, err);
        }
        phase_ = Phase::DONE;
      };
    arm_client_->async_send_goal(goal, opt);
  }

  // --- Members ---
  std::mutex mu_;
  Phase phase_ = Phase::IDLE;

  rclcpp::Publisher<msgs::msg::TaskStatus>::SharedPtr status_pub_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::Client<ArmAction>::SharedPtr arm_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(std::make_shared<BehaviorDemoNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
