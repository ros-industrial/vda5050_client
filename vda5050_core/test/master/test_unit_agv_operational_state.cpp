/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "test_fixture_agv.hpp"

namespace vda5050_core::master::test {

using AGVOperationalStateTestFixture = AGVTestFixture;

namespace {

// Poll instead of fixed-sleep so heartbeat-timer assertions survive TSan
// stretching the nominal 1s heartbeat.
template <typename Pred>
bool wait_for_state(Pred pred, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  return pred();
}

constexpr std::chrono::milliseconds kHeartbeatTimeoutWait{8000};

}  // namespace

// =============================================================================
// State Message Tests
// =============================================================================

TEST_F(
  AGVOperationalStateTestFixture,
  OperationalStateAvailableAfterReceivingStateMessage)
{
  auto& agv = create_agv();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVOperationalStateTestFixture, CachedStateMessageIsStored)
{
  auto& agv = create_agv();

  EXPECT_FALSE(agv->get_last_state().has_value());

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  auto cached = agv->get_last_state();
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->order_id, "test_order");
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

// =============================================================================
// State Heartbeat Timeout Tests
// =============================================================================

TEST_F(
  AGVOperationalStateTestFixture, StateHeartbeatReceivingMessagesPreventTimeout)
{
  auto& agv = create_agv_with_heartbeat_interval(2);

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // A state each interval keeps the heartbeat alive.
  for (int i = 0; i < 3; ++i)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    agv->handle_state(create_state_msg());
    EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
  }

  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture,
  TransitionAvailableToUnknownViaTimeoutThenRecover)
{
  auto& agv = create_agv_with_heartbeat_interval(1);

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  ASSERT_TRUE(wait_for_state(
    [&] { return agv->get_operational_state() == AGVState::STATE_UNKNOWN; },
    kHeartbeatTimeoutWait));

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

// =============================================================================
// Connection State Affects Operational State Tests
// =============================================================================

TEST_F(
  AGVOperationalStateTestFixture,
  ConnectionOfflineSetsOperationalStateToUnavailable)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture,
  ConnectionBrokenSetsOperationalStateToUnavailable)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture, RecoverFromUnavailableAfterConnectionRestored)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  // Reconnecting to ONLINE does not by itself restore AVAILABLE — a State does.
  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

// =============================================================================
// Operational-state precedence
// =============================================================================
// A connection-loss UNAVAILABLE outranks the heartbeat timer's STATE_UNKNOWN
// write. ERROR precedence isn't exercised — nothing sets AGVState::ERROR yet.

TEST_F(
  AGVOperationalStateTestFixture, StateUnknownTimeoutDoesNotClobberUnavailable)
{
  auto& agv = create_agv_with_heartbeat_interval(1);

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  // Fixed sleep, not poll: asserting the timer does NOT fire a transition.
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture,
  StateUnknownTransitionsToUnavailableOnConnectionDrop)
{
  // From STATE_UNKNOWN, a connection drop may still elevate to UNAVAILABLE
  // (the higher-priority write).
  auto& agv = create_agv_with_heartbeat_interval(1);

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  ASSERT_TRUE(wait_for_state(
    [&] { return agv->get_operational_state() == AGVState::STATE_UNKNOWN; },
    kHeartbeatTimeoutWait));

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

// =============================================================================
// Thread Safety Tests
// =============================================================================

TEST_F(AGVOperationalStateTestFixture, ConcurrentOperationalStateAccessIsSafe)
{
  auto& agv = create_agv();
  std::atomic_bool stop{false};

  std::thread reader([&]() {
    while (!stop.load())
    {
      auto op_state = agv->get_operational_state();
      (void)op_state;
    }
  });

  for (int i = 0; i < 100; ++i)
  {
    agv->handle_connection(create_connection_msg("ONLINE"));
    agv->handle_state(create_state_msg());
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    agv->handle_connection(create_connection_msg("OFFLINE"));
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  stop.store(true);
  reader.join();

  SUCCEED();
}

// =============================================================================
// Initial State Tests
// =============================================================================

TEST_F(AGVOperationalStateTestFixture, InitialStatesBeforeAnyMessages)
{
  auto& agv = create_agv();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

}  // namespace vda5050_core::master::test
