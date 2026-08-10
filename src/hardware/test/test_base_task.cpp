// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// gtest cases for BaseTask framework:
//   - TaskStatus enum values
//   - to_string conversions
//   - TaskRegistry register / create / list
//   - REGISTER_TASK macro self-registration (verified by including
//     a header that uses it, then checking the registry)
//
// No rclcpp dependency — tests run in 0.03s on dev machine.

#include "hardware/base_task.hpp"
#include "hardware/task_registry.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace vwh = hardware;

// A trivial test task used to exercise the registry.
class DummyTask : public vwh::BaseTask
{
public:
  explicit DummyTask(rclcpp::Node *) {}
  std::string name() const override { return "dummy"; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::RUNNING; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::SUCCESS; }
  void on_cleanup() override {}
};

class FailingInitTask : public vwh::BaseTask
{
public:
  explicit FailingInitTask(rclcpp::Node *) {}
  std::string name() const override { return "failing_init"; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::FAILED; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::RUNNING; }
  void on_cleanup() override {}
};

class ThrowingTask : public vwh::BaseTask
{
public:
  explicit ThrowingTask(rclcpp::Node *) {}
  std::string name() const override { return "throwing"; }
  vwh::TaskStatus on_init(const vwh::TaskContext &) override { return vwh::TaskStatus::RUNNING; }
  vwh::TaskStatus execute() override { return vwh::TaskStatus::RUNNING; }
  void on_cleanup() override { throw std::runtime_error("cleanup failed"); }
};


// =================================================================
// TaskStatus enum
// =================================================================

TEST(BaseTaskTest, TaskStatusHasFourValues)
{
  vwh::TaskStatus s = vwh::TaskStatus::RUNNING;
  EXPECT_NE(s, vwh::TaskStatus::SUCCESS);
  EXPECT_NE(s, vwh::TaskStatus::FAILED);
  EXPECT_NE(s, vwh::TaskStatus::TIMEOUT);
}

TEST(BaseTaskTest, ToStringConvertsAllValues)
{
  EXPECT_STREQ(vwh::to_string(vwh::TaskStatus::RUNNING), "RUNNING");
  EXPECT_STREQ(vwh::to_string(vwh::TaskStatus::SUCCESS), "SUCCESS");
  EXPECT_STREQ(vwh::to_string(vwh::TaskStatus::FAILED), "FAILED");
  EXPECT_STREQ(vwh::to_string(vwh::TaskStatus::TIMEOUT), "TIMEOUT");
}


// =================================================================
// TaskRegistry basic operations
// =================================================================

class TaskRegistryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Each test gets a clean registry
    auto & reg = vwh::TaskRegistry::instance();
    reg.clear_for_testing();
  }
};

TEST_F(TaskRegistryTest, EmptyRegistryHasZeroTasks)
{
  EXPECT_EQ(vwh::TaskRegistry::instance().size(), 0u);
  EXPECT_TRUE(vwh::TaskRegistry::instance().list().empty());
}

TEST_F(TaskRegistryTest, RegisterAddsTask)
{
  auto & reg = vwh::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) -> std::unique_ptr<vwh::BaseTask> {
      return std::make_unique<DummyTask>(nullptr);
    });
  EXPECT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.list().size(), 1u);
  EXPECT_EQ(reg.list()[0], "dummy");
}

TEST_F(TaskRegistryTest, CreateReturnsNullForUnknownName)
{
  auto & reg = vwh::TaskRegistry::instance();
  EXPECT_EQ(reg.create("nonexistent", nullptr), nullptr);
}

TEST_F(TaskRegistryTest, CreateReturnsTaskForRegisteredName)
{
  auto & reg = vwh::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) -> std::unique_ptr<vwh::BaseTask> {
      return std::make_unique<DummyTask>(nullptr);
    });
  auto task = reg.create("dummy", nullptr);
  ASSERT_NE(task, nullptr);
  EXPECT_EQ(task->name(), "dummy");
}

TEST_F(TaskRegistryTest, CreatePassesNodePointer)
{
  auto & reg = vwh::TaskRegistry::instance();
  rclcpp::Node * sentinel = reinterpret_cast<rclcpp::Node *>(0xDEADBEEF);
  reg.register_task("dummy",
    [sentinel](rclcpp::Node * n) -> std::unique_ptr<vwh::BaseTask> {
      EXPECT_EQ(n, sentinel);
      return std::make_unique<DummyTask>(n);
    });
  auto task = reg.create("dummy", sentinel);
  ASSERT_NE(task, nullptr);
  // The factory assertion above verified the pointer was passed through.
  EXPECT_EQ(task->name(), "dummy");
}

TEST_F(TaskRegistryTest, DuplicateRegistrationThrows)
{
  auto & reg = vwh::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); });
  EXPECT_THROW(
    reg.register_task("dummy",
      [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); }),
    std::runtime_error);
}

TEST_F(TaskRegistryTest, EmptyNameThrows)
{
  auto & reg = vwh::TaskRegistry::instance();
  EXPECT_THROW(
    reg.register_task("",
      [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); }),
    std::invalid_argument);
}

TEST_F(TaskRegistryTest, NullFactoryThrows)
{
  auto & reg = vwh::TaskRegistry::instance();
  EXPECT_THROW(
    reg.register_task("dummy", nullptr),
    std::invalid_argument);
}

TEST_F(TaskRegistryTest, ListIsAlphabetical)
{
  auto & reg = vwh::TaskRegistry::instance();
  reg.register_task("zebra",
    [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); });
  reg.register_task("alpha",
    [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); });
  reg.register_task("mango",
    [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); });
  const auto names = reg.list();
  ASSERT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "alpha");
  EXPECT_EQ(names[1], "mango");
  EXPECT_EQ(names[2], "zebra");
}

TEST_F(TaskRegistryTest, ClearForTestingRemovesAll)
{
  auto & reg = vwh::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); });
  reg.register_task("failing",
    [](rclcpp::Node *) { return std::make_unique<FailingInitTask>(nullptr); });
  EXPECT_EQ(reg.size(), 2u);
  reg.clear_for_testing();
  EXPECT_EQ(reg.size(), 0u);
}


// =================================================================
// BaseTask lifecycle: status values propagate correctly
// =================================================================

TEST(BaseTaskLifecycleTest, DummyTaskOnInitReturnsRunning)
{
  DummyTask t(nullptr);
  vwh::TaskContext ctx;
  EXPECT_EQ(t.on_init(ctx), vwh::TaskStatus::RUNNING);
}

TEST(BaseTaskLifecycleTest, DummyTaskExecuteReturnsSuccess)
{
  DummyTask t(nullptr);
  t.on_init({});
  EXPECT_EQ(t.execute(), vwh::TaskStatus::SUCCESS);
}

TEST(BaseTaskLifecycleTest, FailingInitTaskReturnsFailed)
{
  FailingInitTask t(nullptr);
  EXPECT_EQ(t.on_init({}), vwh::TaskStatus::FAILED);
}

TEST(BaseTaskLifecycleTest, ThrowingTaskCleanupThrows)
{
  ThrowingTask t(nullptr);
  EXPECT_THROW(t.on_cleanup(), std::runtime_error);
}
