// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// BridgeTransport — see header.

#include "hardware/bridge_transport.hpp"

#include <stdexcept>
#include <utility>

namespace hardware
{

BridgeTransport::BridgeTransport(rclcpp::Node * node, std::string service_name,
                                 std::chrono::milliseconds default_timeout)
: node_(node)
, service_name_(std::move(service_name))
, default_timeout_(default_timeout)
, client_(node->create_client<msgs::srv::Mc602Transaction>(service_name_))
{
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
}

void BridgeTransport::close()
{
  opened_ = false;
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

  auto resp_future = client_->async_send_request(req);
  const auto status = rclcpp::spin_until_future_complete(
    node_->get_node_base_interface(), resp_future, client_timeout);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error(
      "mc602 bridge: no response within " + std::to_string(client_timeout.count()) + "ms");
  }
  auto resp = resp_future.get();
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

}  // namespace hardware
