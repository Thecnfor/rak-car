// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SerialScheduler — single-owner serial bus scheduler for the mc602_bridge.
//
// Minimal and event-driven (no sleeps, no polling):
//   1. One io thread owns the bus — multi-process frame interleaving gone.
//   2. Priority: URGENT (e-stop / all-zero wheel speed) jumps the queue,
//      NORMAL and READ are FIFO within their class.
//   3. Per-job timeout: a stalled transaction fails fast instead of blocking
//      100ms.
//
// Multi-frame "control-cycle packing" is done producer-side (MC602Adapter
// begin_burst/commit_burst → one service call with N frames) — the scheduler
// processes each frame as its own transaction, so no batching/sleep window is
// needed here. This mirrors how RoboMaster stacks pack one command frame per
// control cycle: explicit, deterministic, event-driven.
//
// rclcpp-free: depends only on the injectable SerialByteIO (real = SerialPort
// in the bridge node; fake = gtest). Mirrors the MissionRunner "pure class +
// thin shell" pattern.

#pragma once

#include "hardware/serial_transport.hpp"  // ExchangeOpts priority constants

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hardware
{

// Byte-level I/O the scheduler talks to. Real impl wraps SerialPort (frame in
// → full response frame out, honoring the timeout); tests inject a fake.
class SerialByteIO
{
public:
  virtual ~SerialByteIO() = default;
  virtual std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & frame,
    std::chrono::milliseconds timeout) = 0;
};

// One queued transaction. Ownership is shared; the io thread completes it and
// fires on_done. Consumers may also poll job->done under job->mutex.
struct SchedulerJob
{
  uint8_t priority = 1;              // ExchangeOpts::URGENT/NORMAL/READ
  std::vector<uint8_t> frame;        // full MC602 frame (0x77 0x68 ... 0x0A)
  std::chrono::milliseconds timeout{100};

  mutable std::mutex mutex;
  bool done = false;
  bool ok = false;
  std::string error;                 // "" | "timeout" | "transport"
  std::vector<uint8_t> response;     // full response frame

  std::function<void(const std::shared_ptr<SchedulerJob> &)> on_done;
};

class SerialScheduler
{
public:
  explicit SerialScheduler(std::unique_ptr<SerialByteIO> io);
  ~SerialScheduler();

  SerialScheduler(const SerialScheduler &) = delete;
  SerialScheduler & operator=(const SerialScheduler &) = delete;

  void start();
  void stop();

  // Enqueue a transaction. Returns immediately; completion via job->on_done
  // (called from the io thread) or by polling job->done.
  void submit(const std::shared_ptr<SchedulerJob> & job);

  // Enqueue a burst atomically (one lock, one wakeup): the io thread then
  // sees the whole burst contiguous in the queue — true control-cycle packing
  // (e.g. an arm tick's N frames can't be split by other producers' jobs).
  void submit_batch(const std::vector<std::shared_ptr<SchedulerJob>> & jobs);

  struct Stats
  {
    uint64_t frames = 0;     // physical frames written to the bus
    uint64_t timeouts = 0;
    uint64_t errors = 0;
  };
  Stats stats() const;

private:
  void io_loop();
  void complete(std::shared_ptr<SchedulerJob> job, bool ok, std::string error,
                std::vector<uint8_t> response);

  std::unique_ptr<SerialByteIO> io_;
  std::thread thread_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  bool stop_ = false;
  bool started_ = false;

  // Per-priority FIFO queues (index = ExchangeOpts::Priority).
  std::deque<std::shared_ptr<SchedulerJob>> queues_[3];

  Stats stats_;
};

}  // namespace hardware
