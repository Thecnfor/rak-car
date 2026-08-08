// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SerialScheduler — see header for design notes.

#include "hardware/serial_scheduler.hpp"

#include <stdexcept>
#include <utility>

namespace hardware
{

SerialScheduler::SerialScheduler(std::unique_ptr<SerialByteIO> io)
: io_(std::move(io))
{
}

SerialScheduler::~SerialScheduler()
{
  stop();
}

void SerialScheduler::start()
{
  std::lock_guard<std::mutex> lk(mu_);
  if (started_) {
    return;
  }
  started_ = true;
  thread_ = std::thread(&SerialScheduler::io_loop, this);
}

void SerialScheduler::stop()
{
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!started_) {
      return;
    }
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void SerialScheduler::submit(const std::shared_ptr<SchedulerJob> & job)
{
  if (!job) {
    throw std::runtime_error("SerialScheduler::submit: null job");
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    queues_[job->priority].push_back(job);
  }
  cv_.notify_one();
}

void SerialScheduler::submit_batch(
  const std::vector<std::shared_ptr<SchedulerJob>> & jobs)
{
  {
    std::lock_guard<std::mutex> lk(mu_);
    for (const auto & j : jobs) {
      if (j) {
        queues_[j->priority].push_back(j);
      }
    }
  }
  cv_.notify_all();
}

SerialScheduler::Stats SerialScheduler::stats() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return stats_;
}

void SerialScheduler::complete(std::shared_ptr<SchedulerJob> job, bool ok,
                               std::string error, std::vector<uint8_t> response)
{
  {
    std::lock_guard<std::mutex> lk(job->mutex);
    job->done = true;
    job->ok = ok;
    job->error = std::move(error);
    job->response = std::move(response);
  }
  if (job->on_done) {
    job->on_done(job);
  }
}

void SerialScheduler::io_loop()
{
  while (true) {
    std::shared_ptr<SchedulerJob> job;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [this] {
        if (stop_) return true;
        for (const auto & q : queues_) {
          if (!q.empty()) return true;
        }
        return false;
      });
      if (stop_) {
        // Fail remaining jobs so callers never hang on shutdown.
        for (auto & q : queues_) {
          while (!q.empty()) {
            complete(std::move(q.front()), false, "transport", {});
            q.pop_front();
          }
        }
        break;
      }

      // Pop highest-priority FIFO.
      for (auto & q : queues_) {
        if (!q.empty()) {
          job = q.front();
          q.pop_front();
          break;
        }
      }
    }
    if (!job) {
      continue;
    }

    try {
      auto resp = io_->exchange(job->frame, job->timeout);
      ++stats_.frames;
      complete(std::move(job), true, "", std::move(resp));
    } catch (const std::exception & e) {
      ++stats_.timeouts;
      complete(std::move(job), false, "timeout", {});
    }
  }
}

}  // namespace hardware
