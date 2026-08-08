// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SerialTransport — byte-mover abstraction for MC602 frames.
//
// Layering (see docs/hardware-comm.md §MC602):
//   MC602Adapter      = protocol codec (builds full frames, parses payloads)
//   SerialTransport   = moves a full frame in → full response frame out
//   mc602_bridge      = scheduler (single fd owner, priority/coalesce/batch)
//
// The transport does NOT understand frame internals. Two implementations:
//   DirectSerialTransport — local POSIX fd (SerialPort). Used by the bridge
//                           itself and by unit tests.
//   BridgeTransport       — remote, via mc602_bridge service (hardware/src/
//                           bridge_transport.cpp). Sends the frame + scheduling
//                           hints; the bridge owns the fd and the bus.
//
// Independent of ROS2 for the Direct case — testable in isolation.

#pragma once

#include "hardware/serial_port.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hardware
{

// Scheduling hints for one MC602 transaction. Consumed by the mc602_bridge
// scheduler; ignored by direct transports.
struct ExchangeOpts
{
  enum Priority : uint8_t
  {
    URGENT = 0,  // e-stop / all-zero wheel speed: jump the queue
    NORMAL = 1,  // regular SET writes (batchable)
    READ = 2,    // GET reads (issued individually, shareable)
  };

  uint8_t priority = NORMAL;
  std::string coalesce_key;  // write merge: same key keeps only the newest
                             // frame in a batch (absolute set-point semantics)
  std::string share_key;     // read share: same key concurrent reads issue one
                             // physical frame and broadcast the result
  int timeout_ms = 0;        // 0 = transport/bridge default (10ms)
};

class SerialTransport
{
public:
  virtual ~SerialTransport() = default;

  virtual void open() = 0;   // throws std::runtime_error on failure
  virtual void close() = 0;  // must never throw
  virtual bool is_open() const = 0;
  virtual std::string serial_port() const = 0;
  virtual uint32_t baud() const = 0;

  // One request-response transaction. `frame` is a full MC602 frame
  // (0x77 0x68 len payload 0x0A); returns the full response frame.
  // Throws std::runtime_error on timeout / transport error.
  virtual std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & frame, const ExchangeOpts & opts = {}) = 0;

  // Control-cycle packing: send 1..N frames as one logical transaction
  // (e.g. an arm tick's several joint writes in one service call).
  // Returns the per-frame response frames in order. Default: sequential
  // single exchanges; BridgeTransport overrides to a single service call.
  virtual std::vector<std::vector<uint8_t>> exchange_burst(
    const std::vector<std::vector<uint8_t>> & frames,
    const ExchangeOpts & opts = {})
  {
    std::vector<std::vector<uint8_t>> out;
    out.reserve(frames.size());
    for (const auto & f : frames) {
      out.push_back(exchange(f, opts));
    }
    return out;
  }
};

// Local transport over a POSIX serial fd. Thin wrapper over SerialPort.
// `opts` are ignored (no scheduler). Used by the bridge node and tests.
class DirectSerialTransport : public SerialTransport
{
public:
  DirectSerialTransport(std::string device, uint32_t baud);
  ~DirectSerialTransport() override;

  DirectSerialTransport(const DirectSerialTransport &) = delete;
  DirectSerialTransport & operator=(const DirectSerialTransport &) = delete;

  void open() override;
  void close() override;
  bool is_open() const override;
  std::string serial_port() const override;
  uint32_t baud() const override;

  std::vector<uint8_t> exchange(
    const std::vector<uint8_t> & frame, const ExchangeOpts & opts = {}) override;

private:
  SerialPort serial_;
  std::string device_;
  uint32_t baud_;
};

}  // namespace hardware
