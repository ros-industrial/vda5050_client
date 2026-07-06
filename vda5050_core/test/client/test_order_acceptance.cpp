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
#include <string>
#include <utility>

#include "vda5050_core/client/contexts/agv_context.hpp"
#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/strategies/order_acceptance.hpp"
#include "vda5050_core/client/updates/order.hpp"
#include "vda5050_core/types/order.hpp"

namespace {

using OrderAcceptance = vda5050_core::client::OrderAcceptance;
using AGVContext = vda5050_core::client::AGVContext;
using OrderExecutionResource = vda5050_core::client::OrderExecutionResource;
using OrderUpdate = vda5050_core::client::OrderUpdate;
namespace execution = vda5050_core::execution;
namespace types = vda5050_core::types;

std::shared_ptr<AGVContext> make_context()
{
  auto config = std::make_shared<vda5050_core::client::HeaderConfigResource>(
    "uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);
  context->init();
  return context;
}

types::Node make_node(const std::string& node_id, uint32_t sequence_id)
{
  types::Node node;
  node.node_id = node_id;
  node.sequence_id = sequence_id;
  node.released = true;
  return node;
}

types::Edge make_edge(
  const std::string& edge_id, uint32_t sequence_id,
  const std::string& start_node_id, const std::string& end_node_id)
{
  types::Edge edge;
  edge.edge_id = edge_id;
  edge.sequence_id = sequence_id;
  edge.start_node_id = start_node_id;
  edge.end_node_id = end_node_id;
  edge.released = true;
  return edge;
}

types::Action make_action(const std::string& action_id)
{
  types::Action action;
  action.action_id = action_id;
  action.action_type = "pick";
  return action;
}

// Test 1: A valid new order populates the execution state and marks executing.
TEST(OrderAcceptanceTest, AcceptsAndPopulatesNewOrder)
{
  OrderAcceptance strategy;
  auto context = make_context();

  types::Order order;
  order.order_id = "o1";
  order.order_update_id = 0;
  auto node0 = make_node("node_0", 0);
  node0.actions.push_back(make_action("act_0"));
  order.nodes.push_back(node0);
  order.nodes.push_back(make_node("node_2", 2));
  order.edges.push_back(make_edge("edge_1", 1, "node_0", "node_2"));

  context->provider()->push<OrderUpdate>(order);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  EXPECT_TRUE(execution->is_executing_order());
  const auto state = execution->get_state();
  EXPECT_EQ(state.order_id, "o1");
  EXPECT_EQ(state.order_update_id, 0u);
  EXPECT_EQ(state.node_states.size(), 2u);
  EXPECT_EQ(state.edge_states.size(), 1u);
  EXPECT_EQ(state.action_states.size(), 1u);
  EXPECT_TRUE(state.errors.empty());
}

// Test 2: A structurally invalid order is rejected; errors are recorded and the
// execution state is left untouched.
TEST(OrderAcceptanceTest, RejectsInvalidOrderAndRecordsErrors)
{
  OrderAcceptance strategy;
  auto context = make_context();

  types::Order order;
  order.order_id = "bad";
  order.order_update_id = 0;
  order.nodes.push_back(make_node("node_0", 0));
  order.nodes.push_back(make_node("node_2", 2));  // 2 nodes, 0 edges -> invalid

  context->provider()->push<OrderUpdate>(order);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  EXPECT_FALSE(execution->is_executing_order());
  const auto state = execution->get_state();
  EXPECT_TRUE(state.order_id.empty());
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_FALSE(state.errors.empty());
}

// Test 3: A valid update appends the new node onto the running base.
TEST(OrderAcceptanceTest, AppliesUpdateByAppending)
{
  OrderAcceptance strategy;
  auto context = make_context();

  // Seed a running base: order "o1" update 1, sitting at node_a (seq 10).
  {
    auto execution = context->get_resource<OrderExecutionResource>();
    execution->set_executing_order(true);
    types::State state = execution->get_state();
    state.order_id = "o1";
    state.order_update_id = 1;
    state.last_node_id = "node_a";
    state.last_node_sequence_id = 10;
    state.node_states.push_back([] {
      types::NodeState n;
      n.node_id = "node_a";
      n.sequence_id = 10;
      n.released = true;
      return n;
    }());
    execution->set_state(std::move(state));
  }

  types::Order update;
  update.order_id = "o1";
  update.order_update_id = 2;
  update.nodes.push_back(make_node("node_a", 10));  // re-sent stitch node
  update.nodes.push_back(make_node("node_b", 12));
  update.edges.push_back(make_edge("edge_ab", 11, "node_a", "node_b"));

  context->provider()->push<OrderUpdate>(update);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_EQ(state.order_update_id, 2u);
  ASSERT_EQ(state.node_states.size(), 2u);
  EXPECT_EQ(state.node_states[0].node_id, "node_a");
  EXPECT_EQ(state.node_states[1].node_id, "node_b");
  EXPECT_EQ(state.edge_states.size(), 1u);
}

// Test 4: A duplicate update (same orderUpdateId) leaves the state untouched.
TEST(OrderAcceptanceTest, IgnoresDuplicateUpdate)
{
  OrderAcceptance strategy;
  auto context = make_context();

  {
    auto execution = context->get_resource<OrderExecutionResource>();
    types::State state = execution->get_state();
    state.order_id = "o1";
    state.order_update_id = 3;
    state.last_node_id = "node_a";
    state.last_node_sequence_id = 10;
    execution->set_state(std::move(state));
  }

  types::Order duplicate;
  duplicate.order_id = "o1";
  duplicate.order_update_id = 3;
  duplicate.nodes.push_back(make_node("node_a", 10));

  context->provider()->push<OrderUpdate>(duplicate);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_EQ(state.order_update_id, 3u);
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_TRUE(state.errors.empty());
}

// Test 5: Errors from a rejected order are cleared once a new order is accepted,
// rather than leaking into the freshly accepted order's reported state.
TEST(OrderAcceptanceTest, ClearsStaleErrorsOnAcceptedNewOrder)
{
  OrderAcceptance strategy;
  auto context = make_context();

  // First order is structurally invalid -> rejected, error recorded in state.
  types::Order bad;
  bad.order_id = "bad";
  bad.order_update_id = 0;
  bad.nodes.push_back(make_node("node_0", 0));
  bad.nodes.push_back(make_node("node_2", 2));  // 2 nodes, 0 edges -> invalid

  context->provider()->push<OrderUpdate>(bad);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  ASSERT_FALSE(execution->get_state().errors.empty());

  // A subsequent valid new order is accepted and must clear the stale error.
  types::Order good;
  good.order_id = "o1";
  good.order_update_id = 0;
  good.nodes.push_back(make_node("node_0", 0));

  context->provider()->push<OrderUpdate>(good);
  strategy.step(context);

  const auto state = execution->get_state();
  EXPECT_EQ(state.order_id, "o1");
  EXPECT_TRUE(state.errors.empty());
}

// Test 6: step() re-fires on every spin tick against the same cached order, so
// re-processing a rejected order must not accumulate duplicate errors.
TEST(OrderAcceptanceTest, ReprocessingRejectedOrderDoesNotDuplicateErrors)
{
  OrderAcceptance strategy;
  auto context = make_context();

  types::Order bad;
  bad.order_id = "bad";
  bad.order_update_id = 0;
  bad.nodes.push_back(make_node("node_0", 0));
  bad.nodes.push_back(make_node("node_2", 2));  // 2 nodes, 0 edges -> invalid

  context->provider()->push<OrderUpdate>(bad);
  auto execution = context->get_resource<OrderExecutionResource>();

  strategy.step(context);
  const size_t after_first = execution->get_state().errors.size();
  ASSERT_GT(after_first, 0u);

  // Same cached order re-processed on subsequent ticks; error count is stable.
  strategy.step(context);
  strategy.step(context);

  EXPECT_EQ(execution->get_state().errors.size(), after_first);
}

// Test 7: An order update that re-supplies a horizon node's action must not
// duplicate the corresponding action_state.
TEST(OrderAcceptanceTest, UpdateDoesNotDuplicateReSuppliedActionStates)
{
  OrderAcceptance strategy;
  auto context = make_context();

  // Seed a base sitting at node_a (seq 10) with an unreleased horizon node_b
  // whose action "act_b" is already tracked in action_states.
  {
    auto execution = context->get_resource<OrderExecutionResource>();
    execution->set_executing_order(true);
    types::State state = execution->get_state();
    state.order_id = "o1";
    state.order_update_id = 1;
    state.last_node_id = "node_a";
    state.last_node_sequence_id = 10;

    types::NodeState a;
    a.node_id = "node_a";
    a.sequence_id = 10;
    a.released = true;
    types::NodeState b;
    b.node_id = "node_b";
    b.sequence_id = 12;
    b.released = false;  // horizon
    state.node_states = {a, b};

    types::ActionState act;
    act.action_id = "act_b";
    state.action_states = {act};
    execution->set_state(std::move(state));
  }

  // Update re-supplies node_b (now released) carrying the same action act_b.
  types::Order update;
  update.order_id = "o1";
  update.order_update_id = 2;
  update.nodes.push_back(make_node("node_a", 10));  // re-sent stitch node
  auto node_b = make_node("node_b", 12);
  node_b.actions.push_back(make_action("act_b"));
  update.nodes.push_back(node_b);
  update.edges.push_back(make_edge("edge_ab", 11, "node_a", "node_b"));

  context->provider()->push<OrderUpdate>(update);
  strategy.step(context);

  const auto state =
    context->get_resource<OrderExecutionResource>()->get_state();
  size_t act_b_count = 0;
  for (const auto& a : state.action_states)
  {
    if (a.action_id == "act_b") ++act_b_count;
  }
  EXPECT_EQ(act_b_count, 1u);
}

// Test 8: A new order must not inherit traversal progress from a previous order.
// last_node_id / last_node_sequence_id / new_base_request are reset so a later
// update cannot stitch against a stale anchor.
TEST(OrderAcceptanceTest, NewOrderResetsCarriedOverTraversalProgress)
{
  OrderAcceptance strategy;
  auto context = make_context();

  // Seed leftover state from a completed previous order: idle (no nodes, no
  // pending actions) but still carrying a last-reached node and a base request.
  {
    auto execution = context->get_resource<OrderExecutionResource>();
    types::State state = execution->get_state();
    state.order_id = "old";
    state.order_update_id = 7;
    state.last_node_id = "old_node";
    state.last_node_sequence_id = 99;
    state.new_base_request = true;
    execution->set_state(std::move(state));
  }

  types::Order fresh;
  fresh.order_id = "new";
  fresh.order_update_id = 0;
  fresh.nodes.push_back(make_node("node_0", 0));

  context->provider()->push<OrderUpdate>(fresh);
  strategy.step(context);

  const auto state =
    context->get_resource<OrderExecutionResource>()->get_state();
  EXPECT_EQ(state.order_id, "new");
  EXPECT_TRUE(state.last_node_id.empty());
  EXPECT_EQ(state.last_node_sequence_id, 0u);
  EXPECT_FALSE(state.new_base_request.has_value());
}

// Test 9: Rejecting an update while an order is executing must preserve the
// running base (only errors are updated) and keep the vehicle executing.
TEST(OrderAcceptanceTest, RejectedUpdatePreservesRunningBase)
{
  OrderAcceptance strategy;
  auto context = make_context();

  // Running order o1 @ update 2, sitting at node_a (seq 10) still in the base.
  {
    auto execution = context->get_resource<OrderExecutionResource>();
    execution->set_executing_order(true);
    types::State state = execution->get_state();
    state.order_id = "o1";
    state.order_update_id = 2;
    state.last_node_id = "node_a";
    state.last_node_sequence_id = 10;
    types::NodeState a;
    a.node_id = "node_a";
    a.sequence_id = 10;
    a.released = true;
    state.node_states = {a};
    execution->set_state(std::move(state));
  }

  // A deprecated update (lower orderUpdateId) is rejected.
  types::Order stale;
  stale.order_id = "o1";
  stale.order_update_id = 1;
  stale.nodes.push_back(make_node("node_a", 10));

  context->provider()->push<OrderUpdate>(stale);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_TRUE(execution->is_executing_order());  // still executing
  EXPECT_EQ(state.order_update_id, 2u);          // base left untouched
  ASSERT_EQ(state.node_states.size(), 1u);
  EXPECT_EQ(state.node_states[0].node_id, "node_a");
  EXPECT_FALSE(state.errors.empty());  // rejection error recorded
}

// Test 10: A new order rejected because the AGV is busy must not be remembered
// as processed. If the master resends the same order after the AGV becomes idle,
// it should be accepted.
TEST(OrderAcceptanceTest, RejectedBusyOrderCanBeAcceptedWhenResentAfterIdle)
{
  OrderAcceptance strategy;
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();

  // Simulate that the AGV is currently busy executing another order.
  execution->set_executing_order(true);
  {
    types::State state = execution->get_state();
    state.order_id = "running";
    state.order_update_id = 0;
    state.node_states.push_back([] {
      types::NodeState n;
      n.node_id = "running_node";
      n.sequence_id = 0;
      n.released = true;
      return n;
    }());
    execution->set_state(std::move(state));
  }

  // New order arrives while busy, so it should be rejected.
  types::Order queued;
  queued.order_id = "queued";
  queued.order_update_id = 0;
  queued.nodes.push_back(make_node("node_0", 0));

  context->provider()->push<OrderUpdate>(queued);
  strategy.step(context);

  {
    const auto state = execution->get_state();
    EXPECT_EQ(state.order_id, "running");
    EXPECT_FALSE(state.errors.empty());
  }

  // AGV becomes idle after finishing the running order.
  execution->set_executing_order(false);
  {
    types::State state = execution->get_state();
    state.order_id.clear();
    state.order_update_id = 0;
    state.node_states.clear();
    state.edge_states.clear();
    state.action_states.clear();
    state.last_node_id.clear();
    state.last_node_sequence_id = 0;
    execution->set_state(std::move(state));
  }

  // Master resends the same order_id/order_update_id.
  // This should NOT be skipped by last_order_id_/last_order_update_id_.
  context->provider()->push<OrderUpdate>(queued);
  strategy.step(context);

  const auto state = execution->get_state();
  EXPECT_TRUE(execution->is_executing_order());
  EXPECT_EQ(state.order_id, "queued");
  EXPECT_EQ(state.order_update_id, 0u);
  EXPECT_TRUE(state.errors.empty());
  ASSERT_EQ(state.node_states.size(), 1u);
  EXPECT_EQ(state.node_states[0].node_id, "node_0");
}

// Test 11: An already accepted order should not be re-applied on later step()
// ticks while the same cached OrderUpdate remains in the context.
TEST(OrderAcceptanceTest, AcceptedOrderIsNotReprocessedOnLaterTicks)
{
  OrderAcceptance strategy;
  auto context = make_context();

  types::Order order;
  order.order_id = "o1";
  order.order_update_id = 0;
  order.nodes.push_back(make_node("node_0", 0));

  context->provider()->push<OrderUpdate>(order);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state_after_first = execution->get_state();
  ASSERT_EQ(state_after_first.order_id, "o1");
  ASSERT_EQ(state_after_first.node_states.size(), 1u);

  // Mutate state so we can detect if apply_new_order() runs again.
  {
    types::State state = execution->get_state();
    state.node_states.push_back([] {
      types::NodeState n;
      n.node_id = "should_not_be_cleared";
      n.sequence_id = 99;
      n.released = true;
      return n;
    }());
    execution->set_state(std::move(state));
  }

  // Same cached update should be skipped because it was already accepted.
  strategy.step(context);

  const auto state_after_second = execution->get_state();
  ASSERT_EQ(state_after_second.node_states.size(), 2u);
  EXPECT_EQ(state_after_second.node_states[1].node_id, "should_not_be_cleared");
}

}  // namespace
