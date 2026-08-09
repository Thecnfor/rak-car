// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BridgeTransport — remote SerialTransport via the mc602_bridge service.
//
// Consumer nodes (chassis / arm / IR) use this instead of owning a serial fd.
// It sends the full MC602 frame + scheduling hints (priority / coalesce /
// share / timeout) to /rak/hw/mc602/transaction; the bridge owns the bus and
// the scheduler. This is what removes cross-process frame interleaving.
//
// EXECUTOR SAFETY (spec 2026-08-09 §7.1, 方案 A):
//   The caller (a timer/subscription callback under the consumer's own
//   executor) must NOT recursively spin its own node. Instead this transport
//   owns:
//     1. a dedicated client node (same rclcpp context, unique name) that is
//        NEVER added to the consumer's executor, and
//     2. a single worker thread that owns that node.
//   exchange_burst() posts a job and blocks on a per-job condition variable;
//   the worker thread does the service call and spin_until_future_complete()
//   on the dedicated node. No nested executor, no re-entrancy — safe under
//   both SingleThreaded and MultiThreaded consumer executors.

#pragma once

#include "hardware/serial_transport.hpp"

#include <msgs/srv/mc602_transaction.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace hardware
{

class BridgeTransport : public SerialTransport
{
public:
  // `node` may be nullptr (e.g. the ros2_control hardware plugin): the
  // dedicated client node then uses the global default context. The consumer
  // node is only consulted for its context; it is never added to any executor
  // by this transport.
  BridgeTransport(rclcpp::Node * node, std::string service_name,
                  std::chrono::milliseconds default_timeout =
                    std::chrono::milliseconds(1500));

  ~BridgeTransport() override;

  BridgeTransport(const BridgeTransport &) = delete;
  BridgeTransport & operator=(const BridgeTransport &) = delete;

  void open() override;
  void close() override;
  bool is_open() const override;
  std::string serial_port() const override;
  uint32_t baud() const override;

  std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & frame, const ExchangeOpts & opts = {}) override;

  // Control-cycle packing: all frames go in ONE service call (one DDS hop,
  // atomic wrt other consumers). Returns per-frame response frames in order.
  std::vector<std::vector<uint8_t>> exchange_burst(
    const std::vector<std::vector<uint8_t>> & frames,
    const ExchangeOpts & opts = {}) override;

private:
  // One in-flight transaction, completed by the worker thread.
  struct PendingJob
  {
    std::shared_ptr<msgs::srv::Mc602Transaction::Request> request;
    std::chrono::milliseconds timeout;
    std::shared_ptr<msgs::srv::Mc602Transaction::Response> response;
    std::string error;  // "" = ok; else throw message from the bridge path

    std::mutex m;
    std::condition_variable cv;
    bool done = false;
  };

  void worker_loop();
  void process_job(const std::shared_ptr<PendingJob> & job);

  std::shared_ptr<rclcpp::Node> client_node_;  // never added to consumer executor
  std::string service_name_;
  std::chrono::milliseconds default_timeout_;
  bool opened_ = false;
  std::shared_ptr<rclcpp::Client<msgs::srv::Mc602Transaction>> client_;

  std::deque<std::shared_ptr<PendingJob>> jobs_;
  std::mutex mu_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::unique_ptr<std::thread> worker_;
};

}  // namespace hardware
