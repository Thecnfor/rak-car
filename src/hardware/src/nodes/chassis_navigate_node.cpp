// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// chassis_navigate_node — ChassisNavigate action server (middle layer).
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §5
//
// Thin pose controller:
//   * use_pose=true  → close the loop on odom to reach target_pose (odom frame)
//   * use_pose=false → stream target_velocity until timeout / cancel
// Output goes to /rak/cmd/vel_raw (through the safety gate → /rak/cmd/vel_safe
// → mecanum_drive_controller). First version is a simple P controller — no
// Nav2 dependency.

#include <msgs/action/chassis_navigate.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace hardware
{

namespace
{

constexpr char kActionName[] = "/rak/control/chassis/navigate";

}  // namespace

class ChassisNavigateNode : public rclcpp::Node
{
public:
  using ActionServer = rclcpp_action::Server<msgs::action::ChassisNavigate>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<msgs::action::ChassisNavigate>;

  explicit ChassisNavigateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("chassis_navigate_node", options)
  {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("cmd_topic", "/rak/cmd/vel_raw");
    this->declare_parameter<double>("kp_linear", 0.8);
    this->declare_parameter<double>("kp_angular", 1.5);
    this->declare_parameter<double>("max_linear", 0.3);
    this->declare_parameter<double>("max_angular", 0.5);
    this->declare_parameter<double>("feedback_rate_hz", 20.0);

    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    const std::string cmd_topic = this->get_parameter("cmd_topic").as_string();
    kp_lin_ = this->get_parameter("kp_linear").as_double();
    kp_ang_ = this->get_parameter("kp_angular").as_double();
    max_lin_ = this->get_parameter("max_linear").as_double();
    max_ang_ = this->get_parameter("max_angular").as_double();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->on_odom(*msg); });
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    as_ = rclcpp_action::create_server<msgs::action::ChassisNavigate>(
      this, kActionName,
      [this](const rclcpp_action::GoalUUID & uuid,
             const std::shared_ptr<const msgs::action::ChassisNavigate::Goal> & goal) {
        return this->handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandle> & gh) { return this->handle_cancel(gh); },
      [this](const std::shared_ptr<GoalHandle> & gh) { this->handle_accepted(gh); });

    RCLCPP_INFO(this->get_logger(), "ChassisNavigate server up on %s -> %s",
      kActionName, cmd_topic.c_str());
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & odom)
  {
    std::lock_guard<std::mutex> lk(mu_);
    const auto & p = odom.pose.pose;
    // tf2 quaternion → yaw about z.
    const double w = p.orientation.w, z = p.orientation.z;
    pose_.x = p.position.x;
    pose_.y = p.position.y;
    pose_.theta = std::atan2(2.0 * (w * z), 1.0 - 2.0 * z * z);
    have_odom_ = true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    const std::shared_ptr<const msgs::action::ChassisNavigate::Goal> & /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & gh)
  {
    (void)gh;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> & gh)
  {
    std::thread([this, gh]() { this->execute(gh); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> & gh)
  {
    const auto goal = gh->get_goal();
    auto feedback = std::make_shared<msgs::action::ChassisNavigate::Feedback>();
    auto result = std::make_shared<msgs::action::ChassisNavigate::Result>();
    const auto start = this->now();
    const auto timeout = rclcpp::Duration::from_seconds(goal->timeout_sec);

    const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / this->get_parameter("feedback_rate_hz").as_double()));

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        result->success = false;
        result->error = "cancelled";
        gh->succeed(result);
        return;
      }
      if (goal->timeout_sec > 0.0 && (this->now() - start) > timeout) {
        result->success = false;
        result->error = "timeout";
        gh->succeed(result);
        return;
      }

      geometry_msgs::msg::Twist cmd;
      bool done = false;
      {
        std::lock_guard<std::mutex> lk(mu_);
        if (have_odom_) {
          feedback->current_pose.x = pose_.x;
          feedback->current_pose.y = pose_.y;
          feedback->current_pose.theta = pose_.theta;

          if (goal->use_pose) {
            const double ex = goal->target_pose.x - pose_.x;
            const double ey = goal->target_pose.y - pose_.y;
            const double dtheta = std::remainder(goal->target_pose.theta - pose_.theta,
                                                2.0 * M_PI);
            // Rotate the world-frame error into the body frame.
            const double c = std::cos(pose_.theta), s = std::sin(pose_.theta);
            const double bx = c * ex + s * ey;
            const double by = -s * ex + c * ey;
            cmd.linear.x = clamp(kp_lin_ * bx, max_lin_);
            cmd.linear.y = clamp(kp_lin_ * by, max_lin_);
            cmd.angular.z = clamp(kp_ang_ * dtheta, max_ang_);
            feedback->remaining_distance = std::sqrt(ex * ex + ey * ey);
            done = feedback->remaining_distance <= goal->linear_tolerance &&
                   std::fabs(dtheta) <= goal->angular_tolerance;
          } else {
            cmd = goal->target_velocity;
            cmd.linear.x = clamp(cmd.linear.x, max_lin_);
            cmd.linear.y = clamp(cmd.linear.y, max_lin_);
            cmd.angular.z = clamp(cmd.angular.z, max_ang_);
            feedback->remaining_distance = 0.0;
          }
        } else {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "no odom yet");
        }
      }

      cmd_pub_->publish(cmd);
      gh->publish_feedback(feedback);
      if (done) {
        result->success = true;
        result->travelled_distance = 0.0f;  // odom accumulates externally
        result->duration_sec = static_cast<float>((this->now() - start).seconds());
        gh->succeed(result);
        // Stop the base once the goal is reached.
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
        return;
      }
      std::this_thread::sleep_for(period);
    }
    result->success = false;
    result->error = "shutdown";
    gh->abort(result);
  }

  static double clamp(double v, double max_abs)
  {
    return std::max(-max_abs, std::min(max_abs, v));
  }

  struct Pose { double x{0}, y{0}, theta{0}; };
  std::mutex mu_;
  Pose pose_;
  bool have_odom_{false};
  double kp_lin_{0.8}, kp_ang_{1.5}, max_lin_{0.3}, max_ang_{0.5};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::shared_ptr<ActionServer> as_;
};

}  // namespace hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hardware::ChassisNavigateNode>());
  rclcpp::shutdown();
  return 0;
}
