// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// mc602_bridge_node — single owner of the MC602 serial bus.
//
// All MCU communication flows through this node: chassis / arm / IR consumers
// submit transactions via the /rak/hw/mc602/transaction service; the bridge
// owns the fd, the SerialScheduler io thread, and the batching/priority/
// coalescing policy. Cross-process frame interleaving is eliminated because
// exactly one process writes to the serial port.
//
// Thin rclcpp shell: the scheduling logic is the rclcpp-free SerialScheduler.
//
// Run:
//   ros2 run hardware mc602_bridge_node --ros-args \
//     -p mc602_serial_port:=/dev/ttyUSB0 -p mc602_baud:=1000000

#include "hardware/serial_scheduler.hpp"

#include <msgs/srv/mc602_transaction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace hardware
{

namespace
{

// SerialByteIO over the real POSIX serial port.
class SerialPortByteIO final : public SerialByteIO
{
public:
  explicit SerialPortByteIO(std::string device, uint32_t baud)
  : port_(std::move(device), baud)
  {
  }

  void open()
  {
    port_.open();
  }

  void close()
  {
    port_.close();
  }

  bool is_open() const
  {
    return port_.is_open();
  }

  std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & frame,
    std::chrono::milliseconds timeout) override
  {
    return port_.exchange(frame, timeout);
  }

private:
  SerialPort port_;
};

}  // namespace

class Mc602BridgeNode : public rclcpp::Node
{
public:
  explicit Mc602BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mc602_bridge_node", options)
  {
    this->declare_parameter<std::string>("mc602_serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("mc602_baud", 1000000);
    this->declare_parameter<int>("default_timeout_ms", 100);

    const auto port = this->get_parameter("mc602_serial_port").as_string();
    const int baud = this->get_parameter("mc602_baud").as_int();
    default_timeout_ = std::chrono::milliseconds(
      this->get_parameter("default_timeout_ms").as_int());

    auto io = std::make_unique<SerialPortByteIO>(port, static_cast<uint32_t>(baud));
    io_ = io.get();
    try {
      io_->open();
    } catch (const std::exception & e) {
      RCLCPP_FATAL(this->get_logger(), "MC602 serial open failed: %s", e.what());
      throw;
    }

    scheduler_ = std::make_unique<SerialScheduler>(std::move(io));
    scheduler_->start();

    // Reentrant group + multi-threaded executor in main(): concurrent service
    // callbacks can enqueue quickly so the scheduler can batch them.
    group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    srv_ = this->create_service<msgs::srv::Mc602Transaction>(
      "/rak/hw/mc602/transaction",
      [this](const std::shared_ptr<msgs::srv::Mc602Transaction::Request> req,
             std::shared_ptr<msgs::srv::Mc602Transaction::Response> resp) {
        this->on_transaction(req, resp);
      },
      rclcpp::ServicesQoS(), group_);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/rak/hw/mc602/status", rclcpp::QoS(1).transient_local());
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_status(); });

    RCLCPP_INFO(this->get_logger(), "MC602 bridge up: %s @ %d baud",
      port.c_str(), baud);
  }

  ~Mc602BridgeNode() override
  {
    if (scheduler_) {
      scheduler_->stop();
    }
    try {
      io_->close();
    } catch (...) {
      // destructor must not throw
    }
  }

private:
  void on_transaction(
    const std::shared_ptr<msgs::srv::Mc602Transaction::Request> req,
    std::shared_ptr<msgs::srv::Mc602Transaction::Response> resp)
  {
    const auto timeout = (req->timeout_sec > 0.0f)
      ? std::chrono::milliseconds(static_cast<int>(req->timeout_sec * 1000.0f))
      : default_timeout_;

    std::vector<std::shared_ptr<SchedulerJob>> jobs;
    jobs.reserve(req->frames.size());
    for (const auto & f : req->frames) {
      auto job = std::make_shared<SchedulerJob>();
      job->priority = req->priority;
      job->frame = f.data;
      job->timeout = timeout;
      jobs.push_back(job);
    }

    // Submit all frames atomically, then wait for every one to complete before
    // answering the client. The io thread processes them sequentially
    // (single owner); submit_batch keeps the burst contiguous in the queue.
    std::mutex m;
    std::condition_variable cv;
    size_t pending = jobs.size();
    for (auto & job : jobs) {
      job->on_done = [&](const std::shared_ptr<SchedulerJob> &) {
        std::lock_guard<std::mutex> lk(m);
        if (--pending == 0) {
          cv.notify_all();
        }
      };
    }
    scheduler_->submit_batch(jobs);
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait(lk, [&] { return pending == 0; });
    }

    resp->ok = true;
    resp->error = "";
    resp->frames.resize(jobs.size());
    for (size_t i = 0; i < jobs.size(); ++i) {
      resp->frames[i].data = jobs[i]->response;
      if (!jobs[i]->ok) {
        resp->ok = false;
        resp->error = jobs[i]->error;
      }
    }
  }

  void publish_status()
  {
    const auto s = scheduler_->stats();
    std_msgs::msg::String msg;
    msg.data = "frames=" + std::to_string(s.frames) +
               " timeouts=" + std::to_string(s.timeouts) +
               " errors=" + std::to_string(s.errors);
    status_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "%s", msg.data.c_str());
  }

  SerialPortByteIO * io_ = nullptr;
  std::unique_ptr<SerialScheduler> scheduler_;
  std::chrono::milliseconds default_timeout_{100};

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<msgs::srv::Mc602Transaction>::SharedPtr srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  auto node = std::make_shared<hardware::Mc602BridgeNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
