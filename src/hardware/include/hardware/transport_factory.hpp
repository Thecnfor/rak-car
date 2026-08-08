// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// transport_factory — build the MC602 serial transport a node should use.
//
//   mc602_transport := "bridge"  → BridgeTransport via mc602_bridge service
//                                  (single-bus owner; the competition default)
//   mc602_transport := "direct"  → DirectSerialTransport over a local fd
//                                  (mock / tests / no-bridge fallback)
//
// Consumer nodes (chassis / arm / IR) call this from their constructor.

#pragma once

#include "hardware/bridge_transport.hpp"
#include "hardware/serial_transport.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace hardware
{

inline std::shared_ptr<SerialTransport> make_mc602_transport(
  rclcpp::Node * node, const std::string & mode,
  const std::string & serial_port, uint32_t baud)
{
  if (mode == "bridge") {
    return std::make_shared<BridgeTransport>(node, "/rak/hw/mc602/transaction");
  }
  if (mode == "direct") {
    return std::make_shared<DirectSerialTransport>(serial_port, baud);
  }
  throw std::invalid_argument(
    "mc602_transport must be 'bridge' or 'direct', got '" + mode + "'");
}

}  // namespace hardware
