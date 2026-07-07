/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
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

#include <thread>

#include "test_fixture_agv.hpp"

namespace vda5050_core::master::test {

using AGVLifecycleTestFixture = AGVTestFixture;

// =============================================================================
// Stop Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, StopResetsConnectionStatusToOffline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->stop();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
}

TEST_F(AGVLifecycleTestFixture, StopResetsOperationalStateToUnknown)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->stop();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVLifecycleTestFixture, StopPreservesCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->stop();

  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
  EXPECT_TRUE(agv->get_last_connection_time().has_value());
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

TEST_F(AGVLifecycleTestFixture, StopIsIdempotent)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));

  agv->stop();
  agv->stop();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVLifecycleTestFixture, StopOnFreshAgvIsNoOp)
{
  auto& agv = create_agv();

  agv->stop();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

// =============================================================================
// Restart Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, RestartClearsAllCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  agv->handle_factsheet(create_factsheet_msg());
  agv->handle_visualization(create_visualization_msg());
  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
  EXPECT_TRUE(agv->get_last_factsheet().has_value());
  EXPECT_TRUE(agv->get_last_visualization().has_value());

  agv->restart();

  EXPECT_FALSE(agv->get_last_connection().has_value());
  EXPECT_FALSE(agv->get_last_state().has_value());
  EXPECT_FALSE(agv->get_last_factsheet().has_value());
  EXPECT_FALSE(agv->get_last_visualization().has_value());
  EXPECT_FALSE(agv->get_last_connection_time().has_value());
  EXPECT_FALSE(agv->get_last_state_time().has_value());
  EXPECT_FALSE(agv->get_last_factsheet_time().has_value());
  EXPECT_FALSE(agv->get_last_visualization_time().has_value());
}

TEST_F(AGVLifecycleTestFixture, RestartResetsConnectionStatus)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->restart();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
}

TEST_F(AGVLifecycleTestFixture, RestartResetsOperationalState)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->restart();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVLifecycleTestFixture, RestartAllowsNewConnectionCycle)
{
  auto& agv = create_agv();

  // First lifecycle
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->restart();

  // Verify initial state after restart
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // Second lifecycle
  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVLifecycleTestFixture, RestartPreservesIdentity)
{
  auto& agv = create_agv();

  agv->restart();

  EXPECT_EQ(agv->get_manufacturer(), manufacturer_);
  EXPECT_EQ(agv->get_serial_number(), serial_number_);
  EXPECT_EQ(agv->get_agv_id(), agv_id_);
}

// =============================================================================
// Pause Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, PauseSetsStatesToOfflineAndUnavailable)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->pause();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(AGVLifecycleTestFixture, PausePreservesCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();

  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
  EXPECT_TRUE(agv->get_last_connection_time().has_value());
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

TEST_F(AGVLifecycleTestFixture, PauseIsIdempotent)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));

  agv->pause();
  agv->pause();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

// =============================================================================
// Resume Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, ResumeAfterPauseAllowsStateMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->pause();
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  agv->resume();

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVLifecycleTestFixture, ResumePreservesCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();
  agv->resume();

  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
}

TEST_F(AGVLifecycleTestFixture, ResumeOnFreshAgvIsNoOp)
{
  auto& agv = create_agv();

  agv->resume();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

// =============================================================================
// Pause/Resume Cycle Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, MultiplePauseResumeCycles)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  for (int i = 0; i < 3; ++i)
  {
    agv->pause();
    EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
    EXPECT_EQ(
      agv->get_connection_status(),
      vda5050_core::types::ConnectionState::OFFLINE);

    agv->resume();

    agv->handle_state(create_state_msg());
    EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
  }
}

// =============================================================================
// Stop After Pause Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, StopAfterPauseResetsOperationalState)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  agv->stop();
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVLifecycleTestFixture, RestartAfterPauseClearsEverything)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();

  agv->restart();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_FALSE(agv->get_last_connection().has_value());
  EXPECT_FALSE(agv->get_last_state().has_value());
}

// =============================================================================
// Queue Processor Stop/Pause Behavior Tests
// =============================================================================

TEST_F(AGVLifecycleTestFixture, PausePreservesQueuesWhileStopClears)
{
  auto& agv = create_agv();

  // Queue messages while offline (processor not running)
  agv->send_order(create_test_order("order_1"));
  agv->send_order(create_test_order("order_2"));
  agv->send_instant_actions(create_test_instant_actions(1));

  EXPECT_EQ(agv->get_pending_order_count(), 2u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Pause should not clear the queues
  agv->pause();
  EXPECT_EQ(agv->get_pending_order_count(), 2u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Stop should clear the queues
  agv->stop();
  EXPECT_EQ(agv->get_pending_order_count(), 0u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 0u);
}

TEST_F(AGVLifecycleTestFixture, ResumeAfterPauseCanProcessRemainingQueue)
{
  auto& agv = create_agv();

  // Queue messages while offline
  agv->send_order(create_test_order("order_1"));
  agv->send_instant_actions(create_test_instant_actions(1));

  EXPECT_EQ(agv->get_pending_order_count(), 1u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Pause (queues preserved)
  agv->pause();
  EXPECT_EQ(agv->get_pending_order_count(), 1u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Add more messages while paused
  agv->send_order(create_test_order("order_2"));
  EXPECT_EQ(agv->get_pending_order_count(), 2u);

  // Resume - queue processor starts and can process remaining items
  agv->resume();

  // After resume, we should still be able to add to queues
  agv->send_order(create_test_order("order_3"));
  EXPECT_GE(agv->get_pending_order_count(), 1u);
}

}  // namespace vda5050_core::master::test
