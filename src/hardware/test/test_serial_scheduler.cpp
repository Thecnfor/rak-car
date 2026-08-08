// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SerialScheduler unit tests — priority ordering, single-owner FIFO
// serialization, per-job timeout. Jobs are submitted BEFORE start() so the
// io thread observes a deterministic queue (no enqueue race).

#include "hardware/serial_scheduler.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace hardware;

namespace
{

struct FakeIO final : public SerialByteIO
{
  std::mutex mu;
  std::vector<std::vector<uint8_t>> written;
  bool fail = false;

  std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & frame,
    std::chrono::milliseconds /*timeout*/) override
  {
    if (fail) {
      throw std::runtime_error("simulated timeout");
    }
    std::lock_guard<std::mutex> lk(mu);
    written.push_back(frame);
    return frame;  // echo
  }
};

std::shared_ptr<SchedulerJob> make_job(uint8_t priority, uint8_t marker,
                                       int timeout_ms = 50)
{
  auto j = std::make_shared<SchedulerJob>();
  j->priority = priority;
  j->frame = {0x77, 0x68, 0x05, marker, 0x0A};  // marker at frame[3]
  j->timeout = std::chrono::milliseconds(timeout_ms);
  return j;
}

bool wait_done(const std::atomic<int> & done, int target)
{
  for (int i = 0; i < 2000 && done.load() < target; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return done.load() == target;
}

}  // namespace

TEST(SerialSchedulerTest, SequentialFifoInSubmissionOrder)
{
  auto io = std::make_unique<FakeIO>();
  auto * io_raw = io.get();
  SerialScheduler sched(std::move(io));

  std::atomic<int> done{0};
  auto a = make_job(ExchangeOpts::NORMAL, 0x11);
  auto b = make_job(ExchangeOpts::NORMAL, 0x22);
  auto c = make_job(ExchangeOpts::NORMAL, 0x33);
  a->on_done = b->on_done = c->on_done =
    [&](const std::shared_ptr<SchedulerJob> &) { ++done; };

  sched.submit(a);
  sched.submit(b);
  sched.submit(c);
  sched.start();

  EXPECT_TRUE(wait_done(done, 3));
  sched.stop();

  ASSERT_EQ(io_raw->written.size(), 3u);
  EXPECT_EQ(io_raw->written[0][3], 0x11);
  EXPECT_EQ(io_raw->written[1][3], 0x22);
  EXPECT_EQ(io_raw->written[2][3], 0x33);

  EXPECT_TRUE(a->ok && b->ok && c->ok);
  EXPECT_TRUE(a->error.empty());
}

TEST(SerialSchedulerTest, UrgentJumpsAheadOfEarlierNormal)
{
  auto io = std::make_unique<FakeIO>();
  auto * io_raw = io.get();
  SerialScheduler sched(std::move(io));

  std::atomic<int> done{0};
  auto normal = make_job(ExchangeOpts::NORMAL, 0xAA);
  auto urgent = make_job(ExchangeOpts::URGENT, 0xBB);
  normal->on_done = urgent->on_done =
    [&](const std::shared_ptr<SchedulerJob> &) { ++done; };

  sched.submit(normal);  // queued first...
  sched.submit(urgent);  // ...but URGENT must go first
  sched.start();

  EXPECT_TRUE(wait_done(done, 2));
  sched.stop();

  ASSERT_EQ(io_raw->written.size(), 2u);
  EXPECT_EQ(io_raw->written[0][3], 0xBB);
  EXPECT_EQ(io_raw->written[1][3], 0xAA);
}

TEST(SerialSchedulerTest, ReadPriorityAfterNormal)
{
  auto io = std::make_unique<FakeIO>();
  auto * io_raw = io.get();
  SerialScheduler sched(std::move(io));

  std::atomic<int> done{0};
  auto normal = make_job(ExchangeOpts::NORMAL, 0xCC);
  auto read = make_job(ExchangeOpts::READ, 0xDD);
  normal->on_done = read->on_done =
    [&](const std::shared_ptr<SchedulerJob> &) { ++done; };

  sched.submit(normal);
  sched.submit(read);
  sched.start();

  EXPECT_TRUE(wait_done(done, 2));
  sched.stop();

  ASSERT_EQ(io_raw->written.size(), 2u);
  EXPECT_EQ(io_raw->written[0][3], 0xCC);  // NORMAL before READ
  EXPECT_EQ(io_raw->written[1][3], 0xDD);
}

TEST(SerialSchedulerTest, TransportFailureMarksJobTimeout)
{
  auto io = std::make_unique<FakeIO>();
  io->fail = true;
  SerialScheduler sched(std::move(io));

  std::atomic<int> done{0};
  auto job = make_job(ExchangeOpts::NORMAL, 0xEE, 5);
  job->on_done = [&](const std::shared_ptr<SchedulerJob> &) { ++done; };

  sched.submit(job);
  sched.start();

  EXPECT_TRUE(wait_done(done, 1));
  sched.stop();

  EXPECT_FALSE(job->ok);
  EXPECT_EQ(job->error, "timeout");
  EXPECT_TRUE(job->response.empty());
}

TEST(SerialSchedulerTest, StatsCountFrames)
{
  auto io = std::make_unique<FakeIO>();
  SerialScheduler sched(std::move(io));

  std::atomic<int> done{0};
  auto a = make_job(ExchangeOpts::NORMAL, 0x11);
  auto b = make_job(ExchangeOpts::READ, 0x22);
  a->on_done = b->on_done =
    [&](const std::shared_ptr<SchedulerJob> &) { ++done; };

  sched.submit(a);
  sched.submit(b);
  sched.start();
  EXPECT_TRUE(wait_done(done, 2));
  sched.stop();

  EXPECT_EQ(sched.stats().frames, 2u);
  EXPECT_EQ(sched.stats().timeouts, 0u);
}
