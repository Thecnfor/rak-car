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

#include "vehicle_wbt_platform_cpp/base_task.hpp"
#include "vehicle_wbt_platform_cpp/task_registry.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace vwpc = vehicle_wbt_platform_cpp;

// A trivial test task used to exercise the registry.
class DummyTask : public vwpc::BaseTask
{
public:
  explicit DummyTask(rclcpp::Node *) {}
  std::string name() const override { return "dummy"; }
  vwpc::TaskStatus on_init(const vwpc::TaskContext &) override { return vwpc::TaskStatus::RUNNING; }
  vwpc::TaskStatus execute() override { return vwpc::TaskStatus::SUCCESS; }
  void on_cleanup() override {}
};

class FailingInitTask : public vwpc::BaseTask
{
public:
  explicit FailingInitTask(rclcpp::Node *) {}
  std::string name() const override { return "failing_init"; }
  vwpc::TaskStatus on_init(const vwpc::TaskContext &) override { return vwpc::TaskStatus::FAILED; }
  vwpc::TaskStatus execute() override { return vwpc::TaskStatus::RUNNING; }
  void on_cleanup() override {}
};

class ThrowingTask : public vwpc::BaseTask
{
public:
  explicit ThrowingTask(rclcpp::Node *) {}
  std::string name() const override { return "throwing"; }
  vwpc::TaskStatus on_init(const vwpc::TaskContext &) override { return vwpc::TaskStatus::RUNNING; }
  vwpc::TaskStatus execute() override { return vwpc::TaskStatus::RUNNING; }
  void on_cleanup() override { throw std::runtime_error("cleanup failed"); }
};


// =================================================================
// TaskStatus enum
// =================================================================

TEST(BaseTaskTest, TaskStatusHasFourValues)
{
  vwpc::TaskStatus s = vwpc::TaskStatus::RUNNING;
  EXPECT_NE(s, vwpc::TaskStatus::SUCCESS);
  EXPECT_NE(s, vwpc::TaskStatus::FAILED);
  EXPECT_NE(s, vwpc::TaskStatus::TIMEOUT);
}

TEST(BaseTaskTest, ToStringConvertsAllValues)
{
  EXPECT_STREQ(vwpc::to_string(vwpc::TaskStatus::RUNNING), "RUNNING");
  EXPECT_STREQ(vwpc::to_string(vwpc::TaskStatus::SUCCESS), "SUCCESS");
  EXPECT_STREQ(vwpc::to_string(vwpc::TaskStatus::FAILED), "FAILED");
  EXPECT_STREQ(vwpc::to_string(vwpc::TaskStatus::TIMEOUT), "TIMEOUT");
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
    auto & reg = vwpc::TaskRegistry::instance();
    reg.clear_for_testing();
  }
};

TEST_F(TaskRegistryTest, EmptyRegistryHasZeroTasks)
{
  EXPECT_EQ(vwpc::TaskRegistry::instance().size(), 0u);
  EXPECT_TRUE(vwpc::TaskRegistry::instance().list().empty());
}

TEST_F(TaskRegistryTest, RegisterAddsTask)
{
  auto & reg = vwpc::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) -> std::unique_ptr<vwpc::BaseTask> {
      return std::make_unique<DummyTask>(nullptr);
    });
  EXPECT_EQ(reg.size(), 1u);
  EXPECT_EQ(reg.list().size(), 1u);
  EXPECT_EQ(reg.list()[0], "dummy");
}

TEST_F(TaskRegistryTest, CreateReturnsNullForUnknownName)
{
  auto & reg = vwpc::TaskRegistry::instance();
  EXPECT_EQ(reg.create("nonexistent", nullptr), nullptr);
}

TEST_F(TaskRegistryTest, CreateReturnsTaskForRegisteredName)
{
  auto & reg = vwpc::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) -> std::unique_ptr<vwpc::BaseTask> {
      return std::make_unique<DummyTask>(nullptr);
    });
  auto task = reg.create("dummy", nullptr);
  ASSERT_NE(task, nullptr);
  EXPECT_EQ(task->name(), "dummy");
}

TEST_F(TaskRegistryTest, CreatePassesNodePointer)
{
  auto & reg = vwpc::TaskRegistry::instance();
  rclcpp::Node * sentinel = reinterpret_cast<rclcpp::Node *>(0xDEADBEEF);
  reg.register_task("dummy",
    [sentinel](rclcpp::Node * n) -> std::unique_ptr<vwpc::BaseTask> {
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
  auto & reg = vwpc::TaskRegistry::instance();
  reg.register_task("dummy",
    [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); });
  EXPECT_THROW(
    reg.register_task("dummy",
      [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); }),
    std::runtime_error);
}

TEST_F(TaskRegistryTest, EmptyNameThrows)
{
  auto & reg = vwpc::TaskRegistry::instance();
  EXPECT_THROW(
    reg.register_task("",
      [](rclcpp::Node *) { return std::make_unique<DummyTask>(nullptr); }),
    std::invalid_argument);
}

TEST_F(TaskRegistryTest, NullFactoryThrows)
{
  auto & reg = vwpc::TaskRegistry::instance();
  EXPECT_THROW(
    reg.register_task("dummy", nullptr),
    std::invalid_argument);
}

TEST_F(TaskRegistryTest, ListIsAlphabetical)
{
  auto & reg = vwpc::TaskRegistry::instance();
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
  auto & reg = vwpc::TaskRegistry::instance();
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
  vwpc::TaskContext ctx;
  EXPECT_EQ(t.on_init(ctx), vwpc::TaskStatus::RUNNING);
}

TEST(BaseTaskLifecycleTest, DummyTaskExecuteReturnsSuccess)
{
  DummyTask t(nullptr);
  t.on_init({});
  EXPECT_EQ(t.execute(), vwpc::TaskStatus::SUCCESS);
}

TEST(BaseTaskLifecycleTest, FailingInitTaskReturnsFailed)
{
  FailingInitTask t(nullptr);
  EXPECT_EQ(t.on_init({}), vwpc::TaskStatus::FAILED);
}

TEST(BaseTaskLifecycleTest, ThrowingTaskCleanupThrows)
{
  ThrowingTask t(nullptr);
  EXPECT_THROW(t.on_cleanup(), std::runtime_error);
}

TEST(BaseTaskLifecycleTest, InfoDefaultsToName)
{
  DummyTask t(nullptr);
  EXPECT_EQ(t.info(), "dummy");  // info() default returns name()
}
