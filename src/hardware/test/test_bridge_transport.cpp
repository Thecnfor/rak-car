// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BridgeTransport executor-safety regression test (P0, spec §7.1).
//
// The old implementation called rclcpp::spin_until_future_complete() on the
// CONSUMER's node from inside an executor callback — a nested-executor
// re-entrancy hazard. The fix gives BridgeTransport its own dedicated client
// node + worker thread, so an exchange issued from a consumer callback must
// complete without deadlock, even under a single-threaded consumer executor.
//
// This test spins a fake mc602_bridge service, then drives a consumer whose
// timer callback performs an exchange under a SingleThreadedExecutor — the
// exact scenario that used to wedge.

#include "hardware/bridge_transport.hpp"

#include <msgs/srv/mc602_transaction.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr auto kDeadline = std::chrono::seconds(5);

// Stands in for mc602_bridge_node: answers every transaction with a canned
// response frame.
class FakeBridge : public rclcpp::Node
{
public:
  FakeBridge() : Node("fake_mc602_bridge")
  {
    srv_ = this->create_service<msgs::srv::Mc602Transaction>(
      "/rak/hw/mc602/transaction",
      [](const std::shared_ptr<msgs::srv::Mc602Transaction::Request> req,
         std::shared_ptr<msgs::srv::Mc602Transaction::Response> resp) {
        resp->ok = true;
        resp->frames.resize(req->frames.size());
        for (auto & f : resp->frames) {
          f.data = {0x77, 0x68, 0x04, 0x01, 0x0A};  // echo a plausible response
        }
      });
  }

private:
  rclcpp::Service<msgs::srv::Mc602Transaction>::SharedPtr srv_;
};

// Consumer whose TIMER callback runs a BridgeTransport exchange — i.e. the
// call happens inside an executor-managed callback (the pre-fix hazard).
class Consumer : public rclcpp::Node
{
public:
  Consumer() : Node("bridge_consumer")
  {
    transport_ = std::make_shared<hardware::BridgeTransport>(
      nullptr, "/rak/hw/mc602/transaction");
    transport_->open();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        if (done_.load()) {
          return;
        }
        try {
          // exchange() returns the response FRAME BYTES (vector<uint8_t>).
          auto frames = transport_->exchange({0x77, 0x68, 0x05, 0x0A});
          ok_ = (frames.size() == 5);  // the 5-byte echo frame came back
          err_ = "frames=" + std::to_string(frames.size());
        } catch (const std::exception & e) {
          ok_ = false;
          err_ = e.what();
        }
        done_ = true;
      });
  }

  bool done() const { return done_.load(); }
  bool ok() const { return ok_.load(); }
  const std::string & error() const { return err_; }

private:
  std::shared_ptr<hardware::BridgeTransport> transport_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> ok_{false};
  std::atomic<bool> done_{false};
  std::string err_{"not run"};
};

}  // namespace

TEST(BridgeTransportExecutorSafeTest, ExchangeInsideSingleThreadedCallbackWorks)
{
  ASSERT_EQ(rclcpp::ok(), false) << "rclcpp already initialized";
  rclcpp::init(0, nullptr);

  auto bridge = std::make_shared<FakeBridge>();
  auto consumer = std::make_shared<Consumer>();

  // Real deployment topology: the bridge is its own process/executor, the
  // consumer is another. Mirror that — bridge on its own executor thread, the
  // consumer on a single-threaded executor.
  rclcpp::executors::SingleThreadedExecutor bridge_executor;
  bridge_executor.add_node(bridge);
  std::thread bridge_spinner([&]() { bridge_executor.spin(); });

  rclcpp::executors::SingleThreadedExecutor consumer_executor;
  consumer_executor.add_node(consumer);
  std::thread consumer_spinner([&]() { consumer_executor.spin(); });

  const auto start = std::chrono::steady_clock::now();
  while (!consumer->done() &&
         std::chrono::steady_clock::now() - start < kDeadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bridge_executor.cancel();
  consumer_executor.cancel();
  if (bridge_spinner.joinable()) {
    bridge_spinner.join();
  }
  if (consumer_spinner.joinable()) {
    consumer_spinner.join();
  }
  rclcpp::shutdown();

  ASSERT_TRUE(consumer->done()) << "exchange never completed (deadlock?)";
  EXPECT_TRUE(consumer->ok()) << "exchange failed inside consumer callback: "
                              << consumer->error();
}
