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

/**
 * @file agv_order_lifecycle_test.cpp
 * @brief Wire-up tests verifying AGV routes incoming State messages into
 *        the OrderLifecycleManager and that restart() clears the lifecycle.
 *
 * Pure logic tests — no broker, nullptr ProtocolAdapter (publish path is
 * unreachable, but lifecycle observation through handle_state IS exercised).
 */

#include <gtest/gtest.h>

#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/types/state.hpp"

#include "test_fixture_agv.hpp"

namespace vda5050_core::master::test {

using AGVOrderLifecycleTest = AGVTestFixture;

TEST_F(AGVOrderLifecycleTest, HandleStateUpdatesLifecycleSequenceId)
{
  auto& agv = create_agv();

  // Snapshot before any state — defaults all zero.
  auto before = agv->active_order_snapshot();
  EXPECT_EQ(before.last_node_sequence_id, 0u);
  EXPECT_FALSE(before.has_active);

  // Send a state with a non-default last_node_sequence_id; the AGV must
  // forward it into the lifecycle manager.
  auto state = create_state_msg();
  state.last_node_id = "N7";
  state.last_node_sequence_id = 42;
  state.order_update_id = 3;
  agv->handle_state(state);

  auto after = agv->active_order_snapshot();
  EXPECT_EQ(after.last_node_sequence_id, 42u);
  EXPECT_EQ(after.state_order_update_id, 3u);
}

TEST_F(AGVOrderLifecycleTest, HandleStateSetsNeedsMoreBaseFlag)
{
  auto& agv = create_agv();
  EXPECT_FALSE(agv->active_order_needs_more_base());

  auto state = create_state_msg();
  state.new_base_request = true;
  agv->handle_state(state);

  EXPECT_TRUE(agv->active_order_needs_more_base())
    << "AGV must forward newBaseRequest into lifecycle manager";
}

TEST_F(AGVOrderLifecycleTest, RestartClearsLifecycle)
{
  auto& agv = create_agv();

  // Drive the lifecycle into a non-default state via handle_state.
  auto state = create_state_msg();
  state.last_node_sequence_id = 99;
  state.new_base_request = true;
  agv->handle_state(state);
  ASSERT_TRUE(agv->active_order_needs_more_base());
  ASSERT_EQ(agv->active_order_snapshot().last_node_sequence_id, 99u);

  agv->restart();

  EXPECT_FALSE(agv->active_order_needs_more_base());
  EXPECT_EQ(agv->active_order_snapshot().last_node_sequence_id, 0u);
  EXPECT_FALSE(agv->has_active_order());
  EXPECT_EQ(agv->pending_update_count(), 0u);
}

}  // namespace vda5050_core::master::test
