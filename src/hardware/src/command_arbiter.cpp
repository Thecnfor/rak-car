// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// CommandArbiter — see header.

#include "hardware/command_arbiter.hpp"

namespace hardware
{

ArbiterResult CommandArbiter::acquire(ControlResource res, ControlPriority prio)
{
  // Short resources never contend with long actions.
  if (!is_long(res)) {
    return ArbiterResult{ArbiterResult::Status::GRANTED, ""};
  }
  if (res == ControlResource::CHASSIS || res == ControlResource::ARM) {
    if (!holder_.has_value()) {
      holder_ = res;
      return ArbiterResult{ArbiterResult::Status::GRANTED, ""};
    }
    if (prio == ControlPriority::URGENT) {
      holder_ = res;  // preempt the current holder
      return ArbiterResult{ArbiterResult::Status::GRANTED, "preempted"};
    }
    return ArbiterResult{ArbiterResult::Status::BUSY, "another long action holds the bus"};
  }
  return ArbiterResult{ArbiterResult::Status::REJECTED, "unknown resource"};
}

void CommandArbiter::release(ControlResource res)
{
  if (holder_.has_value() && holder_.value() == res) {
    holder_.reset();
  }
}

}  // namespace hardware
