// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// Unit tests for CommandArbiter — long-action mutual exclusion + URGENT preempt.

#include "hardware/command_arbiter.hpp"

#include <gtest/gtest.h>

using hardware::ArbiterResult;
using hardware::CommandArbiter;
using hardware::ControlPriority;
using hardware::ControlResource;

TEST(CommandArbiterTest, ChassisAndArmMutuallyExclusive)
{
  CommandArbiter a;
  EXPECT_EQ(a.acquire(ControlResource::CHASSIS).status, ArbiterResult::Status::GRANTED);
  EXPECT_EQ(a.acquire(ControlResource::ARM).status, ArbiterResult::Status::BUSY);
  a.release(ControlResource::CHASSIS);
  EXPECT_EQ(a.acquire(ControlResource::ARM).status, ArbiterResult::Status::GRANTED);
  EXPECT_EQ(a.acquire(ControlResource::CHASSIS).status, ArbiterResult::Status::BUSY);
}

TEST(CommandArbiterTest, ArmCanReacquireAfterRelease)
{
  CommandArbiter a;
  a.acquire(ControlResource::ARM);
  a.release(ControlResource::ARM);
  EXPECT_EQ(a.acquire(ControlResource::ARM).status, ArbiterResult::Status::GRANTED);
}

TEST(CommandArbiterTest, UrgentPreemptsLongHolder)
{
  CommandArbiter a;
  a.acquire(ControlResource::ARM);
  auto res = a.acquire(ControlResource::CHASSIS, ControlPriority::URGENT);
  EXPECT_EQ(res.status, ArbiterResult::Status::GRANTED);
  EXPECT_EQ(a.holder(), ControlResource::CHASSIS);
}

TEST(CommandArbiterTest, PeripheralAndPumpAlwaysGranted)
{
  CommandArbiter a;
  a.acquire(ControlResource::ARM);  // long holder
  EXPECT_EQ(a.acquire(ControlResource::PERIPHERAL).status, ArbiterResult::Status::GRANTED);
  EXPECT_EQ(a.acquire(ControlResource::PUMP).status, ArbiterResult::Status::GRANTED);
  EXPECT_EQ(a.holder(), ControlResource::ARM);  // long holder untouched
}

TEST(CommandArbiterTest, ReleaseOnlyByHolder)
{
  CommandArbiter a;
  a.acquire(ControlResource::ARM);
  a.release(ControlResource::CHASSIS);  // wrong resource — no-op
  EXPECT_EQ(a.holder(), ControlResource::ARM);
  a.release(ControlResource::ARM);
  EXPECT_FALSE(a.holder().has_value());
}
