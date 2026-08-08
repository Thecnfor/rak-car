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
// rclcpp-based (service client). Requires the consumer node's executor to be
// multi-threaded so responses arrive while the calling callback blocks.

#pragma once

#include "hardware/serial_transport.hpp"

#include <msgs/srv/mc602_transaction.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hardware
{

class BridgeTransport : public SerialTransport
{
public:
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
  rclcpp::Node * node_ = nullptr;
  std::string service_name_;
  std::chrono::milliseconds default_timeout_;
  bool opened_ = false;
  std::shared_ptr<rclcpp::Client<msgs::srv::Mc602Transaction>> client_;
};

}  // namespace hardware
