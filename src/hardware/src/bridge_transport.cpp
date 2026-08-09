// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BridgeTransport — see header.

#include "hardware/bridge_transport.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <stdexcept>
#include <utility>

namespace hardware
{

namespace
{

// Unique, process-local names so multiple consumers each get their own
// dedicated client node without colliding on the DDS node graph.
std::atomic<uint32_t> g_client_seq{0};

}  // namespace

BridgeTransport::BridgeTransport(rclcpp::Node * node, std::string service_name,
                                 std::chrono::milliseconds default_timeout)
: service_name_(std::move(service_name))
, default_timeout_(default_timeout)
{
  // Dedicated client node: same rclcpp context, but never attached to the
  // consumer's executor. Only the worker thread spins it.
  //
  // `node` may be nullptr (e.g. the ros2_control hardware plugin, which has
  // no consumer node) — then the client node uses the global default context.
  const std::string node_name =
    "mc602_bridge_client_" + std::to_string(g_client_seq.fetch_add(1));
  if (node == nullptr) {
    client_node_ = std::make_shared<rclcpp::Node>(
      node_name, rclcpp::NodeOptions());
  } else {
    client_node_ = std::make_shared<rclcpp::Node>(node_name, node->get_node_options());
  }
  client_ = client_node_->create_client<msgs::srv::Mc602Transaction>(service_name_);
}

BridgeTransport::~BridgeTransport()
{
  close();
}

void BridgeTransport::open()
{
  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error(
      "BridgeTransport: mc602_bridge service '" + service_name_ + "' not available");
  }
  opened_ = true;
  if (!worker_) {
    worker_ = std::make_unique<std::thread>(&BridgeTransport::worker_loop, this);
  }
}

void BridgeTransport::close()
{
  opened_ = false;
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (stop_) {
      return;
    }
    stop_ = true;
  }
  cv_.notify_all();
  if (worker_ && worker_->joinable()) {
    worker_->join();
  }
  worker_.reset();
}

bool BridgeTransport::is_open() const
{
  return opened_ && client_->service_is_ready();
}

std::string BridgeTransport::serial_port() const
{
  return service_name_;
}

uint32_t BridgeTransport::baud() const
{
  return 0;  // unknown — the bridge owns the port
}

std::vector<uint8_t> BridgeTransport::exchange(
  const std::vector<uint8_t> & frame, const ExchangeOpts & opts)
{
  auto burst = exchange_burst({frame}, opts);
  return burst.empty() ? std::vector<uint8_t>{} : std::move(burst[0]);
}

std::vector<std::vector<uint8_t>> BridgeTransport::exchange_burst(
  const std::vector<std::vector<uint8_t>> & frames, const ExchangeOpts & opts)
{
  if (frames.empty()) {
    return {};
  }
  if (!worker_) {
    throw std::runtime_error("BridgeTransport: not open (worker thread not running)");
  }

  auto req = std::make_shared<msgs::srv::Mc602Transaction::Request>();
  req->priority = opts.priority;
  req->frames.resize(frames.size());
  for (size_t i = 0; i < frames.size(); ++i) {
    req->frames[i].data = frames[i];
  }
  req->timeout_sec = (opts.timeout_ms > 0)
    ? static_cast<float>(opts.timeout_ms) / 1000.0f
    : 0.0f;  // 0 → bridge default

  // Client timeout must exceed the bridge's serial timeout so the bridge is
  // the one enforcing the deadline (and reporting it as "timeout").
  const auto client_timeout = (opts.timeout_ms > 0)
    ? std::chrono::milliseconds(opts.timeout_ms + 250)
    : default_timeout_;

  auto job = std::make_shared<PendingJob>();
  job->request = req;
  job->timeout = client_timeout;

  {
    std::lock_guard<std::mutex> lk(mu_);
    jobs_.push_back(job);
  }
  cv_.notify_one();

  // Block the caller until the worker thread finishes this transaction. The
  // worker (not the caller's executor) spins the dedicated client node, so
  // this is executor-safe.
  {
    std::unique_lock<std::mutex> lk(job->m);
    job->cv.wait(lk, [&] { return job->done; });
  }

  if (job->response == nullptr) {
    throw std::runtime_error("mc602 bridge: " + job->error);
  }
  auto resp = job->response;
  if (!resp->ok) {
    throw std::runtime_error("mc602 bridge: " + resp->error);
  }
  std::vector<std::vector<uint8_t>> out;
  out.reserve(resp->frames.size());
  for (const auto & f : resp->frames) {
    out.push_back(f.data);
  }
  return out;
}

void BridgeTransport::worker_loop()
{
  while (true) {
    std::shared_ptr<PendingJob> job;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [&] { return stop_ || !jobs_.empty(); });
      if (stop_ && jobs_.empty()) {
        break;
      }
      job = jobs_.front();
      jobs_.pop_front();
    }
    process_job(job);
  }
}

void BridgeTransport::process_job(const std::shared_ptr<PendingJob> & job)
{
  auto resp_future = client_->async_send_request(job->request);
  const auto status = rclcpp::spin_until_future_complete(
    client_node_->get_node_base_interface(), resp_future, job->timeout);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    job->error =
      "no response within " + std::to_string(job->timeout.count()) + "ms";
  } else {
    job->response = resp_future.get();
  }
  {
    std::lock_guard<std::mutex> lk(job->m);
    job->done = true;
  }
  job->cv.notify_all();
}

}  // namespace hardware
