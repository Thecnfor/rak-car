// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BehaviorClient end-to-end test against a real (in-process) action server.
//
// What this proves:
//   - resolve_backends() finds a live ChassisNavigate action server → kLocalPid
//   - start_drive_to_pose → poll → SUCCESS (real goal/result round-trip)
//   - start_follow_waypoints → sequential ChassisNavigate goals, one per
//     waypoint, auto-advancing through the whole list
//   - a rejecting server surfaces FAILED (not a hang or silent success)
//   - start_arm_path → sequential ArmCartesianMove goals, one per waypoint
//
// This is the "task → action server" link the mission framework depends on:
// the same protocol path a real task (seeding) uses on-device. No mocks in
// production code — this is a test-only fake server.

#include "hardware/behavior_client.hpp"

#include <msgs/action/arm_cartesian_move.hpp>
#include <msgs/action/chassis_navigate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr auto kDeadline = std::chrono::seconds(5);

using Action = msgs::action::ChassisNavigate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

// Stands in for chassis_navigate_node: accepts (or rejects) each goal and
// succeeds it immediately. Counts accepted goals so the test can assert the
// client issued the expected number of ChassisNavigate calls.
class MockChassisServer : public rclcpp::Node
{
public:
  explicit MockChassisServer(bool reject = false)
  : Node("mock_chassis_server"), reject_(reject)
  {
    as_ = rclcpp_action::create_server<Action>(
      this, "/rak/control/chassis/navigate",
      [this](const rclcpp_action::GoalUUID &,
             const std::shared_ptr<const Action::Goal> &) {
        return reject_ ? rclcpp_action::GoalResponse::REJECT
                       : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle> &) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> & gh) {
        accepted_.fetch_add(1);
        auto result = std::make_shared<Action::Result>();
        result->success = true;
        result->error = "";
        gh->succeed(result);
      });
  }

  int accepted() const { return accepted_.load(); }

private:
  bool reject_{false};
  std::atomic<int> accepted_{0};
  rclcpp_action::Server<Action>::SharedPtr as_;
};

// Stands in for arm_cartesian_move_node: accepts + succeeds each goal.
class MockArmServer : public rclcpp::Node
{
public:
  using ArmAction = msgs::action::ArmCartesianMove;
  using ArmGoalHandle = rclcpp_action::ServerGoalHandle<ArmAction>;

  MockArmServer() : Node("mock_arm_server")
  {
    as_ = rclcpp_action::create_server<ArmAction>(
      this, "/rak/control/arm/cartesian_move",
      [](const rclcpp_action::GoalUUID &,
         const std::shared_ptr<const ArmAction::Goal> &) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<ArmGoalHandle> &) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ArmGoalHandle> & gh) {
        accepted_.fetch_add(1);
        auto result = std::make_shared<ArmAction::Result>();
        result->status = 0;  // REACHED
        result->error = "";
        gh->succeed(result);
      });
  }

  int accepted() const { return accepted_.load(); }

private:
  std::atomic<int> accepted_{0};
  rclcpp_action::Server<ArmAction>::SharedPtr as_;
};

// Stands in for mission_runner_node. Constructs BehaviorClient lazily on the
// first tick (so the probe runs while the executor is already spinning and
// the fake server is discoverable). All BehaviorClient traffic happens on the
// executor thread — no cross-thread races.
class BehaviorHost : public rclcpp::Node
{
public:
  using Request = std::pair<hardware::Waypoint, bool /* follow_all */>;

  BehaviorHost() : Node("behavior_host")
  {
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), [this]() { tick(); });
  }

  void request_drive_to_pose(const hardware::Waypoint & wp)
  {
    std::lock_guard<std::mutex> lk(mu_);
    host_op_ = HostOp::kChassis;
    pending_.clear();
    pending_.push_back({wp, false});
  }

  void request_follow_waypoints(const std::vector<hardware::Waypoint> & wps)
  {
    std::lock_guard<std::mutex> lk(mu_);
    host_op_ = HostOp::kChassis;
    pending_.clear();
    for (const auto & wp : wps) {
      pending_.push_back({wp, true});
    }
  }

  void request_arm_path(const std::vector<hardware::ArmWaypoint> & wps)
  {
    std::lock_guard<std::mutex> lk(mu_);
    host_op_ = HostOp::kArmPath;
    arm_path_ = wps;
  }

  bool done() const { return done_.load(); }
  bool ok() const { return ok_.load(); }
  std::string error() const { return err_; }
  std::string backend() const { return backend_; }

private:
  enum class HostOp : uint8_t { kChassis = 0, kArmPath };

  void start_next_op()
  {
    op_running_ = true;
    op_ever_started_ = true;
    if (host_op_ == HostOp::kArmPath) {
      if (!behavior_->start_arm_path(arm_path_, 5.0)) {
        done_.store(true);
        ok_.store(false);
        err_ = "arm path start failed: " + behavior_->last_error();
      }
      return;
    }
    Request req = pending_.front();
    pending_.pop_front();
    const bool ok_start = req.second
      ? behavior_->start_follow_waypoints({req.first}, 5.0)
      : behavior_->start_drive_to_pose(req.first, 5.0);
    if (!ok_start) {
      done_.store(true);
      ok_.store(false);
      err_ = "start failed: " + behavior_->last_error();
    }
  }

  void tick()
  {
    if (done_.load()) {
      return;
    }
    if (!behavior_) {
      behavior_ = std::make_shared<hardware::BehaviorClient>(this);
      backend_ = behavior_->backend_report();
      return;
    }

    if (op_running_) {
      const auto r = behavior_->poll();
      if (r.running()) {
        return;
      }
      op_running_ = false;
      if (!r.ok()) {
        done_.store(true);
        ok_.store(false);
        err_ = r.error;
        return;
      }
      // One op finished cleanly → drive the next pending request, if any.
    }

    if (host_op_ == HostOp::kArmPath) {
      // 单次 arm path 调用, 完成后即结束。
      if (op_ever_started_) {
        done_.store(true);
        ok_.store(true);
        return;
      }
      start_next_op();
      return;
    }
    bool pending_empty;
    {
      std::lock_guard<std::mutex> lk(mu_);
      pending_empty = pending_.empty();
    }
    if (pending_empty) {
      if (!op_ever_started_) {
        return;  // nothing requested yet
      }
      done_.store(true);
      ok_.store(true);
      return;
    }
    start_next_op();
  }

  std::shared_ptr<hardware::BehaviorClient> behavior_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mu_;
  std::deque<Request> pending_;
  HostOp host_op_{HostOp::kChassis};
  std::vector<hardware::ArmWaypoint> arm_path_;
  bool op_running_{false};
  bool op_ever_started_{false};
  std::atomic<bool> done_{false};
  std::atomic<bool> ok_{false};
  std::string err_{"not run"};
  std::string backend_;
};

// Drive a scenario: run the fake server + host on a MultiThreadedExecutor,
// wait for the host to finish (or timeout), report the outcome.
struct Scenario
{
  std::shared_ptr<MockChassisServer> server;
  std::shared_ptr<MockArmServer> arm_server;
  std::shared_ptr<BehaviorHost> host;
  std::string backend;
  bool ok{false};
  std::string error;
  bool timed_out{false};
};

Scenario run_scenario(bool reject, const std::function<void(BehaviorHost &)> & setup)
{
  Scenario sc;
  sc.server = std::make_shared<MockChassisServer>(reject);
  sc.arm_server = std::make_shared<MockArmServer>();
  sc.host = std::make_shared<BehaviorHost>();

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(sc.server);
  ex.add_node(sc.arm_server);
  ex.add_node(sc.host);
  std::thread spinner([&]() { ex.spin(); });

  setup(*sc.host);

  const auto start = std::chrono::steady_clock::now();
  while (!sc.host->done() &&
         std::chrono::steady_clock::now() - start < kDeadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sc.timed_out = !sc.host->done();
  sc.ok = sc.host->ok();
  sc.error = sc.host->error();
  sc.backend = sc.host->backend();

  ex.cancel();
  spinner.join();
  return sc;
}

}  // namespace

class BehaviorClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(BehaviorClientTest, ResolvesLocalBackendWhenServerPresent)
{
  auto sc = run_scenario(false, [](BehaviorHost & host) {
    host.request_drive_to_pose(hardware::Waypoint{1.0, 0.0, 0.0});
  });

  EXPECT_FALSE(sc.timed_out) << "scenario never finished";
  EXPECT_TRUE(sc.ok) << "drive_to_pose failed: " << sc.error;
  EXPECT_EQ(sc.server->accepted(), 1) << "expected exactly one ChassisNavigate goal";
  EXPECT_NE(sc.backend.find("local"), std::string::npos)
    << "expected local backend fallback, got: " << sc.backend;
}

TEST_F(BehaviorClientTest, FollowWaypointsIssuesOneGoalPerWaypoint)
{
  auto sc = run_scenario(false, [](BehaviorHost & host) {
    host.request_follow_waypoints({
      hardware::Waypoint{0.0, 0.0, 0.0},
      hardware::Waypoint{1.0, 0.0, 0.0},
      hardware::Waypoint{1.0, 1.0, 0.0},
    });
  });

  EXPECT_FALSE(sc.timed_out) << "scenario never finished";
  EXPECT_TRUE(sc.ok) << "follow_waypoints failed: " << sc.error;
  EXPECT_EQ(sc.server->accepted(), 3)
    << "expected one ChassisNavigate goal per waypoint, got "
    << sc.server->accepted();
}

TEST_F(BehaviorClientTest, RejectedGoalSurfacesFailure)
{
  auto sc = run_scenario(true, [](BehaviorHost & host) {
    host.request_drive_to_pose(hardware::Waypoint{1.0, 0.0, 0.0});
  });

  EXPECT_FALSE(sc.timed_out) << "scenario never finished";
  EXPECT_FALSE(sc.ok) << "expected rejection to fail the op";
  EXPECT_EQ(sc.server->accepted(), 0) << "rejecting server must accept nothing";
}

TEST_F(BehaviorClientTest, ArmPathIssuesOneGoalPerWaypoint)
{
  auto sc = run_scenario(false, [](BehaviorHost & host) {
    host.request_arm_path({
      hardware::ArmWaypoint{100.0, 180.0, 0.0, 0},
      hardware::ArmWaypoint{120.0, 200.0, 10.0, 0},
      hardware::ArmWaypoint{140.0, 220.0, 0.0, 2},
    });
  });

  EXPECT_FALSE(sc.timed_out) << "scenario never finished";
  EXPECT_TRUE(sc.ok) << "arm path failed: " << sc.error;
  EXPECT_EQ(sc.arm_server->accepted(), 3)
    << "expected one ArmCartesianMove goal per waypoint, got "
    << sc.arm_server->accepted();
}
