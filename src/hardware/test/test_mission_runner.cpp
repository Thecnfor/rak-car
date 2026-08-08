// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for MissionRunner (pure mission state machine).
//
// MissionRunner is rclcpp-free by design (it only forwards an opaque node
// pointer into TaskContext), so these tests drive it with fake tasks, a
// fake factory, and synthetic wall-clock seconds.
//
// Coverage:
//   - empty task list        -> "idle: empty task_list", not running
//   - unknown task name      -> "failed: unknown task 'X'", not running
//   - sequential execution   -> tasks run in order, then "completed: N"
//   - SUCCESS advances       -> next task is created
//   - FAILED aborts          -> mission stops, no "completed"
//   - timeout aborts         -> "timeout: X (elapsed Ns)"
//   - on_init FAILED         -> aborts with "failed: X on_init returned ..."
//   - single-step task       -> on_init SUCCESS advances without execute()
//   - cleanup throwing       -> guarded ("warn: ... on_cleanup threw"), mission continues
//   - factory returning null -> "failed: factory for 'X' returned null"

#include "hardware/mission_runner.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vwh = hardware;

namespace
{

// Runs `ticks` execute() calls returning RUNNING, then returns `final`.
class TicksTask : public vwh::BaseTask
{
public:
  TicksTask(std::string name, int ticks, vwh::TaskStatus final_status)
  : name_(std::move(name)), ticks_(ticks), final_(final_status) {}

  std::string name() const override { return name_; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::RUNNING; }
  vwh::TaskStatus execute() override
  {
    if (ticks_ > 0) { --ticks_; return vwh::TaskStatus::RUNNING; }
    return final_;
  }
  void on_cleanup() override { cleaned = true; }

  bool cleaned = false;

private:
  std::string name_;
  int ticks_;
  vwh::TaskStatus final_;
};

// on_init returns SUCCESS immediately — a single-step task whose execute()
// must never be called.
class InstantTask : public vwh::BaseTask
{
public:
  explicit InstantTask(std::string name) : name_(std::move(name)) {}
  std::string name() const override { return name_; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::SUCCESS; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::FAILED; }  // must not run
  void on_cleanup() override { cleaned = true; }

  bool cleaned = false;

private:
  std::string name_;
};

// on_init returns FAILED.
class BadInitTask : public vwh::BaseTask
{
public:
  explicit BadInitTask(std::string name) : name_(std::move(name)) {}
  std::string name() const override { return name_; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::FAILED; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::RUNNING; }
  void on_cleanup() override { cleaned = true; }

  bool cleaned = false;

private:
  std::string name_;
};

// on_cleanup throws — the runner must not propagate it.
class ThrowCleanupTask : public vwh::BaseTask
{
public:
  explicit ThrowCleanupTask(std::string name) : name_(std::move(name)) {}
  std::string name() const override { return name_; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::RUNNING; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::SUCCESS; }
  void on_cleanup() override { throw std::runtime_error("boom"); }

private:
  std::string name_;
};

// Runs forever (for timeout tests).
class ForeverTask : public vwh::BaseTask
{
public:
  explicit ForeverTask(std::string name) : name_(std::move(name)) {}
  std::string name() const override { return name_; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::RUNNING; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::RUNNING; }
  void on_cleanup() override { cleaned = true; }

  bool cleaned = false;

private:
  std::string name_;
};

// Test harness: owns a runner with a scripted factory + a progress collector.
class MissionRunnerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    progress_.clear();
    created_.clear();
  }

  // factory_map: task name -> creator. A name with no entry returns nullptr.
  void build_runner(
    const std::map<std::string,
      std::function<std::unique_ptr<vwh::BaseTask>()>> & factory_map)
  {
    runner_ = std::make_unique<vwh::MissionRunner>(
      [factory_map, this](const std::string & name) -> std::unique_ptr<vwh::BaseTask> {
        created_.push_back(name);
        auto it = factory_map.find(name);
        return it == factory_map.end() ? nullptr : it->second();
      },
      [this](const std::string & msg) { progress_.push_back(msg); });
  }

  std::unique_ptr<vwh::MissionRunner> runner_;
  std::vector<std::string> progress_;
  std::vector<std::string> created_;
};

}  // namespace

// ------------------------------------------------------------------
// start() semantics
// ------------------------------------------------------------------

TEST_F(MissionRunnerTest, EmptyTaskListDoesNotStart)
{
  build_runner({});
  EXPECT_FALSE(runner_->start({}, 30.0, {"seeding"}));
  EXPECT_FALSE(runner_->running());
  EXPECT_THAT(progress_, testing::Contains("idle: empty task_list"));
}

TEST_F(MissionRunnerTest, UnknownTaskNameFailsStart)
{
  build_runner({});
  EXPECT_FALSE(runner_->start({"nope"}, 30.0, {"seeding"}));
  EXPECT_FALSE(runner_->running());
  EXPECT_THAT(progress_, testing::Contains("failed: unknown task 'nope'"));
}

TEST_F(MissionRunnerTest, StartEmitsStartedMessage)
{
  build_runner({{"a", [] { return std::make_unique<TicksTask>("a", 0, vwh::TaskStatus::SUCCESS); }}});
  ASSERT_TRUE(runner_->start({"a"}, 30.0, {"a"}));
  EXPECT_TRUE(runner_->running());
  EXPECT_THAT(progress_, testing::Contains("started: 1 tasks"));
}

// ------------------------------------------------------------------
// Sequential execution
// ------------------------------------------------------------------

TEST_F(MissionRunnerTest, RunsTasksInOrderThenCompletes)
{
  build_runner({
    {"a", [] { return std::make_unique<TicksTask>("a", 1, vwh::TaskStatus::SUCCESS); }},
    {"b", [] { return std::make_unique<TicksTask>("b", 0, vwh::TaskStatus::SUCCESS); }},
  });
  ASSERT_TRUE(runner_->start({"a", "b"}, 30.0, {"a", "b"}));

  // a: 1 RUNNING tick, then SUCCESS.
  EXPECT_TRUE(runner_->tick(0.0));    // create a, on_init; execute -> RUNNING
  EXPECT_TRUE(runner_->tick(0.1));    // execute -> SUCCESS, advance to b
  // b: single execute -> SUCCESS, then complete on the next tick.
  EXPECT_TRUE(runner_->tick(0.2));    // create b, on_init; execute -> SUCCESS
  EXPECT_FALSE(runner_->tick(0.3));   // mission complete

  EXPECT_THAT(created_, testing::ElementsAre("a", "b"));
  EXPECT_THAT(progress_, testing::Contains("SUCCESS: a"));
  EXPECT_THAT(progress_, testing::Contains("SUCCESS: b"));
  EXPECT_THAT(progress_, testing::Contains("completed: 2 tasks"));
  EXPECT_FALSE(runner_->running());
}

TEST_F(MissionRunnerTest, FailedTaskAbortsMission)
{
  build_runner({
    {"a", [] { return std::make_unique<TicksTask>("a", 0, vwh::TaskStatus::FAILED); }},
    {"b", [] { return std::make_unique<TicksTask>("b", 0, vwh::TaskStatus::SUCCESS); }},
  });
  ASSERT_TRUE(runner_->start({"a", "b"}, 30.0, {"a", "b"}));

  // ticks=0 + FAILED: the first tick completes the task (execute returns
  // FAILED immediately, runner cleans up, aborts, returns false).
  EXPECT_FALSE(runner_->tick(0.0));
  EXPECT_FALSE(runner_->running());
  EXPECT_FALSE(runner_->tick(0.1));  // no-op once stopped

  EXPECT_THAT(created_, testing::ElementsAre("a"));  // b never created
  EXPECT_THAT(progress_, testing::Contains("FAILED: a"));
  EXPECT_THAT(progress_, testing::Not(testing::Contains("completed: 2 tasks")));
}

TEST_F(MissionRunnerTest, SingleStepTaskAdvancesWithoutExecute)
{
  build_runner({
    {"instant", [] { return std::make_unique<InstantTask>("instant"); }},
    {"next", [] { return std::make_unique<TicksTask>("next", 0, vwh::TaskStatus::SUCCESS); }},
  });
  ASSERT_TRUE(runner_->start({"instant", "next"}, 30.0, {"instant", "next"}));

  EXPECT_TRUE(runner_->tick(0.0));   // instant: on_init -> SUCCESS, advance
  EXPECT_TRUE(runner_->tick(0.1));   // next: on_init, execute -> SUCCESS
  EXPECT_FALSE(runner_->tick(0.2));  // complete

  EXPECT_THAT(created_, testing::ElementsAre("instant", "next"));
  EXPECT_THAT(progress_, testing::Contains("SUCCESS: next"));
}

// ------------------------------------------------------------------
// Failure handling
// ------------------------------------------------------------------

TEST_F(MissionRunnerTest, TimeoutAbortsMission)
{
  build_runner({{"long", [] { return std::make_unique<ForeverTask>("long"); }}});
  ASSERT_TRUE(runner_->start({"long"}, 1.0, {"long"}));

  EXPECT_TRUE(runner_->tick(0.0));   // create, on_init at t=0
  EXPECT_TRUE(runner_->tick(0.5));   // RUNNING (elapsed 0.5 < 1.0)
  EXPECT_FALSE(runner_->tick(2.0));  // elapsed 2.0 > 1.0 -> timeout

  EXPECT_FALSE(runner_->running());
  EXPECT_THAT(progress_, testing::Contains(testing::HasSubstr("timeout: long")));
}

TEST_F(MissionRunnerTest, BadInitAbortsMission)
{
  build_runner({{"bad", [] { return std::make_unique<BadInitTask>("bad"); }}});
  ASSERT_TRUE(runner_->start({"bad"}, 30.0, {"bad"}));

  EXPECT_FALSE(runner_->tick(0.0));  // on_init -> FAILED, abort

  EXPECT_FALSE(runner_->running());
  EXPECT_THAT(progress_, testing::Contains("failed: bad on_init returned FAILED"));
}

TEST_F(MissionRunnerTest, FactoryReturningNullAborts)
{
  // No entry for "ghost" -> factory returns nullptr.
  build_runner({});
  ASSERT_TRUE(runner_->start({"ghost"}, 30.0, {"ghost"}));

  EXPECT_FALSE(runner_->tick(0.0));
  EXPECT_FALSE(runner_->running());
  EXPECT_THAT(progress_, testing::Contains("failed: factory for 'ghost' returned null"));
}

TEST_F(MissionRunnerTest, CleanupThrowingIsGuardedAndMissionContinues)
{
  build_runner({
    {"boom", [] { return std::make_unique<ThrowCleanupTask>("boom"); }},
    {"fine", [] { return std::make_unique<TicksTask>("fine", 0, vwh::TaskStatus::SUCCESS); }},
  });
  ASSERT_TRUE(runner_->start({"boom", "fine"}, 30.0, {"boom", "fine"}));

  EXPECT_TRUE(runner_->tick(0.0));   // boom execute -> SUCCESS, cleanup throws (guarded), advance
  EXPECT_TRUE(runner_->tick(0.1));   // fine runs
  EXPECT_FALSE(runner_->tick(0.2));  // complete

  EXPECT_THAT(progress_, testing::Contains(testing::HasSubstr("warn: boom on_cleanup threw")));
  EXPECT_THAT(progress_, testing::Contains("completed: 2 tasks"));
}

// ------------------------------------------------------------------
// Lifecycle hygiene
// ------------------------------------------------------------------

TEST_F(MissionRunnerTest, CleanupRunsOnSuccessAndIsGuarded)
{
  // ThrowCleanupTask above already proves on_cleanup runs on SUCCESS and
  // that a throwing cleanup is caught. Here we additionally assert that
  // after cleanup the runner is still healthy and can complete the mission.
  build_runner({
    {"a", [] { return std::make_unique<ThrowCleanupTask>("a"); }},
    {"b", [] { return std::make_unique<TicksTask>("b", 0, vwh::TaskStatus::SUCCESS); }},
  });
  ASSERT_TRUE(runner_->start({"a", "b"}, 30.0, {"a", "b"}));

  EXPECT_TRUE(runner_->tick(0.0));   // a: execute -> SUCCESS, throwing cleanup guarded
  EXPECT_TRUE(runner_->tick(0.1));   // b: runs
  EXPECT_FALSE(runner_->tick(0.2));  // complete

  EXPECT_THAT(progress_, testing::Contains(testing::HasSubstr("warn: a on_cleanup threw")));
  EXPECT_THAT(progress_, testing::Contains("completed: 2 tasks"));
}
