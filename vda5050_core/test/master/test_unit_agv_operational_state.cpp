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

// Use the shared AGV test fixture
using AGVOperationalStateTestFixture = AGVTestFixture;

namespace {

// Poll until predicate is true or the deadline elapses. Used in place
// of fixed sleeps so heartbeat-timer assertions stay robust under TSan
// instrumentation, which can stretch the nominal 1s heartbeat well past
// the original 2.5s margin.
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
// Initial State Tests
// =============================================================================

TEST_F(AGVOperationalStateTestFixture, InitialOperationalStateIsUnknown)
{
  auto& agv = create_agv();
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

// =============================================================================
// State Message Tests
// =============================================================================

TEST_F(
  AGVOperationalStateTestFixture,
  OperationalStateAvailableAfterReceivingStateMessage)
{
  auto& agv = create_agv();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // First establish connection to enable state heartbeat
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Now receive state message
  agv->handle_state(create_state_msg());

  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVOperationalStateTestFixture, CachedStateMessageIsStored)
{
  auto& agv = create_agv();

  // Initially no cached state
  EXPECT_FALSE(agv->get_last_state().has_value());

  // Establish connection and receive state
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  // Verify cached state
  auto cached = agv->get_last_state();
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->order_id, "test_order");

  // Verify timestamp was recorded
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

// =============================================================================
// State Heartbeat Timeout Tests
// =============================================================================

TEST_F(
  AGVOperationalStateTestFixture,
  StateHeartbeatTimeoutTransitionsToStateUnknown)
{
  // Create AGV with short state heartbeat interval (1 second)
  auto& agv = create_agv_with_heartbeat_interval(1);

  // Establish connection to start heartbeat
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Receive state message to go AVAILABLE
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Wait for the state heartbeat timeout to trigger (polled — robust to
  // sanitizer slowdown).
  ASSERT_TRUE(wait_for_state(
    [&] { return agv->get_operational_state() == AGVState::STATE_UNKNOWN; },
    kHeartbeatTimeoutWait));
}

TEST_F(
  AGVOperationalStateTestFixture, StateHeartbeatReceivingMessagesPreventTimeout)
{
  // Create AGV with short state heartbeat interval (2 seconds)
  auto& agv = create_agv_with_heartbeat_interval(2);

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Receive initial state
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Send state messages periodically to keep operational state alive
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
  // Create AGV with short state heartbeat interval (1 second)
  auto& agv = create_agv_with_heartbeat_interval(1);

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Initial state is UNKNOWN
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // Receive state message to go AVAILABLE
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Wait for timeout (polled — robust to sanitizer slowdown).
  ASSERT_TRUE(wait_for_state(
    [&] { return agv->get_operational_state() == AGVState::STATE_UNKNOWN; },
    kHeartbeatTimeoutWait));

  // Recover by receiving state message
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

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Receive OFFLINE connection message
  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture,
  ConnectionBrokenSetsOperationalStateToUnavailable)
{
  auto& agv = create_agv();

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Receive CONNECTIONBROKEN connection message
  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture, RecoverFromUnavailableAfterConnectionRestored)
{
  auto& agv = create_agv();

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Go OFFLINE
  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  // Reconnect
  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  // Operational state is still UNAVAILABLE until state message received
  // (because going OFFLINE changes operational state to UNAVAILABLE,
  // but going back ONLINE doesn't automatically restore AVAILABLE)

  // Receive state message to recover to AVAILABLE
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

// =============================================================================
// Operational-state precedence (Task #27)
// =============================================================================
//
// The state-heartbeat timer's STATE_UNKNOWN write must not clobber a
// connection-loss-driven UNAVAILABLE. A connection drop is the more
// authoritative signal — the AGV is offline, not merely silent. Without
// the precedence rule, pre-send rejections would mislead operators
// ("STATE_UNKNOWN" instead of the real "connection lost") during
// reconnection windows.
//
// ERROR-vs-STATE_UNKNOWN precedence is not exercised here because
// nothing in the current master codebase sets AGVState::ERROR — the
// state is reserved for error-classification work post-V0 (#26). The
// suppression rule in agv.cpp covers ERROR symmetrically; it will be
// validated when the ERROR-setting path lands.

TEST_F(
  AGVOperationalStateTestFixture, StateUnknownTimeoutDoesNotClobberUnavailable)
{
  // Heartbeat interval short enough to fire the timer mid-test.
  auto& agv = create_agv_with_heartbeat_interval(1);

  // Bring the AGV up: connection ONLINE + valid State → AVAILABLE.
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Drop the connection — operational_state flips to UNAVAILABLE.
  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  // Wait long enough for the state-heartbeat timer to fire its
  // STATE_UNKNOWN write. With the precedence rule, the existing
  // UNAVAILABLE must remain. Fixed sleep is required here — we're
  // asserting the *absence* of a transition, so polling would just
  // return immediately. Use a generous margin for sanitizer slowdown.
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture, UnavailableTransitionsToAvailableOnRecovery)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  // Recovery edge: ONLINE + State should restore AVAILABLE. The
  // precedence rule must not block the legitimate
  // UNAVAILABLE → AVAILABLE transition (only STATE_UNKNOWN is
  // suppressed — AVAILABLE / ERROR / UNAVAILABLE writes pass through).
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(
  AGVOperationalStateTestFixture,
  StateUnknownTransitionsToUnavailableOnConnectionDrop)
{
  // Drive AGV into STATE_UNKNOWN via heartbeat timeout, then drop the
  // connection. The OFFLINE write should elevate the state to
  // UNAVAILABLE — that direction is allowed (UNAVAILABLE is more
  // authoritative; STATE_UNKNOWN is the lower-priority write).
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
// Enum Value Tests
// =============================================================================

TEST_F(AGVOperationalStateTestFixture, AGVStateEnumValues)
{
  EXPECT_NE(AGVState::STATE_UNKNOWN, AGVState::AVAILABLE);
  EXPECT_NE(AGVState::STATE_UNKNOWN, AGVState::UNAVAILABLE);
  EXPECT_NE(AGVState::STATE_UNKNOWN, AGVState::ERROR);
  EXPECT_NE(AGVState::AVAILABLE, AGVState::UNAVAILABLE);
  EXPECT_NE(AGVState::AVAILABLE, AGVState::ERROR);
  EXPECT_NE(AGVState::UNAVAILABLE, AGVState::ERROR);
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

  // Both start in their initial states
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

}  // namespace vda5050_core::master::test
