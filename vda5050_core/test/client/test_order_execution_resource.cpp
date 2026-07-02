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

#include <gmock/gmock.h>

#include <memory>
#include <typeindex>

#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/updates/order.hpp"
#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/types/state.hpp"

namespace {

using namespace vda5050_core::execution;  // NOLINT
using namespace vda5050_core::client;     // NOLINT
namespace types = vda5050_core::types;

// Test 1: Ensure OrderExecutionResource correctly stores runtime execution state.
TEST(OrderExecutionResourceTest, CarriesExecutionSnapshot)
{
  OrderExecutionResource resource;
  resource.set_executing_order(true);
  resource.set_awaiting_order_update(true);

  types::State state;
  state.order_id = "order_1";
  state.order_update_id = 1;
  state.last_node_id = "node_0";
  state.last_node_sequence_id = 0;
  resource.set_state(state);

  EXPECT_TRUE(resource.is_executing_order());
  EXPECT_TRUE(resource.is_awaiting_order_update());

  auto snapshot = resource.get_state();
  EXPECT_EQ(snapshot.order_id, "order_1");
  EXPECT_EQ(snapshot.order_update_id, 1u);
  EXPECT_EQ(snapshot.last_node_id, "node_0");
  EXPECT_EQ(snapshot.last_node_sequence_id, 0u);
}

// Test 2: Check OrderExecutionResource reports its type through ResourceBase.
TEST(OrderExecutionResourceTest, ReportsTypeThroughBase)
{
  std::shared_ptr<ResourceBase> base =
    std::make_shared<OrderExecutionResource>();

  EXPECT_EQ(base->get_type(), std::type_index(typeid(OrderExecutionResource)));
}

// Test 3: Make sure each resource/update type must have unique runtime type identity.
TEST(OrderExecutionResourceTest, HasDistinctTypeIndex)
{
  EXPECT_NE(
    std::type_index(typeid(OrderExecutionResource)),
    std::type_index(typeid(OrderUpdate)));

  EXPECT_NE(
    std::type_index(typeid(OrderExecutionResource)),
    std::type_index(typeid(HeaderConfigResource)));
}

// Test 4: A freshly constructed resource exposes default (empty) state.
TEST(OrderExecutionResourceTest, GettersReturnDefaultsWhenFresh)
{
  OrderExecutionResource resource;

  EXPECT_FALSE(resource.is_executing_order());
  EXPECT_FALSE(resource.is_awaiting_order_update());

  auto snapshot = resource.get_state();
  EXPECT_EQ(snapshot.order_id, "");
  EXPECT_EQ(snapshot.order_update_id, 0u);
  EXPECT_TRUE(snapshot.node_states.empty());
  EXPECT_TRUE(snapshot.edge_states.empty());
  EXPECT_TRUE(snapshot.action_states.empty());
}

// Test 5: get_state surfaces the stored node states.
TEST(OrderExecutionResourceTest, ExposesNodeStates)
{
  OrderExecutionResource resource;

  types::NodeState node_a;
  node_a.node_id = "node_a";
  node_a.sequence_id = 0;
  node_a.released = true;

  types::State state;
  state.node_states.push_back(node_a);
  resource.set_state(state);

  auto retrieved_nodes = resource.get_state().node_states;
  ASSERT_EQ(retrieved_nodes.size(), 1u);
  EXPECT_EQ(retrieved_nodes[0].node_id, "node_a");
  EXPECT_TRUE(retrieved_nodes[0].released);
}

// Test 6: get_state surfaces the stored edge states.
TEST(OrderExecutionResourceTest, ExposesEdgeStates)
{
  OrderExecutionResource resource;

  types::EdgeState edge_ab;
  edge_ab.edge_id = "edge_ab";
  edge_ab.sequence_id = 1;
  edge_ab.released = false;

  types::State state;
  state.edge_states.push_back(edge_ab);
  resource.set_state(state);

  auto retrieved_edges = resource.get_state().edge_states;
  ASSERT_EQ(retrieved_edges.size(), 1u);
  EXPECT_EQ(retrieved_edges[0].edge_id, "edge_ab");
  EXPECT_FALSE(retrieved_edges[0].released);
}

// Test 7: get_state surfaces the stored action states.
TEST(OrderExecutionResourceTest, ExposesActionStates)
{
  OrderExecutionResource resource;

  types::ActionState action_pick;
  action_pick.action_id = "action_pick";
  action_pick.action_status = types::ActionStatus::RUNNING;

  types::State state;
  state.action_states.push_back(action_pick);
  resource.set_state(state);

  auto retrieved_actions = resource.get_state().action_states;
  ASSERT_EQ(retrieved_actions.size(), 1u);
  EXPECT_EQ(retrieved_actions[0].action_id, "action_pick");
  EXPECT_EQ(retrieved_actions[0].action_status, types::ActionStatus::RUNNING);
}

// Test 8: get_state/set_state round-trips the full snapshot.
TEST(OrderExecutionResourceTest, GetAndSetStateRoundTrips)
{
  OrderExecutionResource resource;

  types::State state;
  state.order_id = "order_abc";
  state.order_update_id = 99;
  resource.set_state(state);

  auto snapshot = resource.get_state();
  EXPECT_EQ(snapshot.order_id, "order_abc");
  EXPECT_EQ(snapshot.order_update_id, 99u);
}

}  // namespace
