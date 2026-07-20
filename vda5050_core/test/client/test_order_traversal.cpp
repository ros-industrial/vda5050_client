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
#include <vector>

#include "vda5050_core/client/contexts/agv_context.hpp"
#include "vda5050_core/client/events/edge_entered.hpp"
#include "vda5050_core/client/events/edge_left.hpp"
#include "vda5050_core/client/events/navigate_to_node.hpp"
#include "vda5050_core/client/events/node_traversed.hpp"
#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/strategies/order_traversal.hpp"
#include "vda5050_core/client/updates/node_reached.hpp"

namespace {

using EdgeEnteredEvent = vda5050_core::client::EdgeEnteredEvent;
using EdgeLeftEvent = vda5050_core::client::EdgeLeftEvent;
using NavigateToNodeEvent = vda5050_core::client::NavigateToNodeEvent;
using NodeReachedUpdate = vda5050_core::client::NodeReachedUpdate;
using AGVContext = vda5050_core::client::AGVContext;
using NodeTraversedEvent = vda5050_core::client::NodeTraversedEvent;
using OrderExecutionResource = vda5050_core::client::OrderExecutionResource;
using OrderTraversal = vda5050_core::client::OrderTraversal;
namespace types = vda5050_core::types;

std::shared_ptr<AGVContext> make_context()
{
  auto config = std::make_shared<vda5050_core::client::HeaderConfigResource>(
    "uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);
  context->init();
  return context;
}

types::NodeState node_state(
  const std::string& node_id, uint32_t sequence_id, bool released)
{
  types::NodeState n;
  n.node_id = node_id;
  n.sequence_id = sequence_id;
  n.released = released;
  return n;
}

types::EdgeState edge_state(const std::string& edge_id, uint32_t sequence_id)
{
  types::EdgeState e;
  e.edge_id = edge_id;
  e.sequence_id = sequence_id;
  e.released = true;
  return e;
}

types::Node node_from_state(const types::NodeState& state)
{
  types::Node n;
  n.node_id = state.node_id;
  n.sequence_id = state.sequence_id;
  n.released = state.released;
  n.node_position = state.node_position;
  n.node_description = state.node_description;
  return n;
}

types::Edge edge_from_state(const types::EdgeState& state)
{
  types::Edge e;
  e.edge_id = state.edge_id;
  e.sequence_id = state.sequence_id;
  e.released = state.released;
  e.trajectory = state.trajectory;
  return e;
}

// Seed the execution state with nodes and edges.
void seed(
  const std::shared_ptr<AGVContext>& context,
  std::vector<types::NodeState> nodes, std::vector<types::EdgeState> edges)
{
  types::Order order;
  order.order_id = "o1";
  for (const auto& node : nodes)
  {
    order.nodes.push_back(node_from_state(node));
  }
  for (const auto& edge : edges)
  {
    order.edges.push_back(edge_from_state(edge));
  }

  auto execution = context->get_resource<OrderExecutionResource>();
  types::State state = execution->get_state();
  state.order_id = "o1";
  state.node_states = std::move(nodes);
  state.edge_states = std::move(edges);
  execution->set_state(std::move(state));
  execution->set_order(std::move(order));
  execution->set_executing_order(true);
}

void seed(
  const std::shared_ptr<AGVContext>& context, types::Order order,
  std::vector<types::NodeState> nodes, std::vector<types::EdgeState> edges)
{
  auto execution = context->get_resource<OrderExecutionResource>();
  types::State state = execution->get_state();
  state.order_id = order.order_id;
  state.order_update_id = order.order_update_id;
  state.node_states = std::move(nodes);
  state.edge_states = std::move(edges);
  execution->set_state(std::move(state));
  execution->set_order(std::move(order));
  execution->set_executing_order(true);
}

// Test 1: Reaching a node drops its state + incoming edge and sets lastNode.
TEST(OrderTraversalTest, AdvancesStateOnNodeReached)
{
  OrderTraversal strategy;
  auto context = make_context();
  // node_0 already traversed; sitting on edge e1 toward node_2.
  // node_2, node_4 are in the base.
  // e1 is the incoming edge to node_2.
  // e3 is the outgoing edge from node_2.
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, true)},
    {edge_state("e1", 1), edge_state("e3", 3)});

  // node_2 is reached.
  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  // Step the strategy.
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();

  // Snapshot the execution state to read it.
  const auto state = execution->get_state();
  EXPECT_EQ(state.last_node_id, "node_2");
  EXPECT_EQ(state.last_node_sequence_id, 2u);

  // The reached node is removed from the pending traversal state,
  // leaving only the remaining base node ahead.
  ASSERT_EQ(state.node_states.size(), 1u);
  EXPECT_EQ(state.node_states.front().node_id, "node_4");

  // The incoming edge is removed from the pending traversal state,
  // leaving only the outgoing edge.
  ASSERT_EQ(state.edge_states.size(), 1u);
  EXPECT_EQ(state.edge_states.front().sequence_id, 3u);  // e1 dropped
}

// Test 2: An unknown node-reached signal leaves the state untouched.
TEST(OrderTraversalTest, IgnoresUnknownNode)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, true)},
    {edge_state("e1", 1), edge_state("e3", 3)});

  // Pushed an unknown node-reached signal.
  context->provider()->push<NodeReachedUpdate>("ghost", 99);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_EQ(state.node_states.size(), 2u);
  EXPECT_TRUE(state.last_node_id.empty());
}

// Test 3: Reaching the last released node before a horizon stops traversal and
// raises new_base_request.
TEST(OrderTraversalTest, StopsAtDecisionPoint)
{
  OrderTraversal strategy;
  auto context = make_context();
  // node_2 is the last released base node; node_4 is horizon.
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, false)},
    {edge_state("e3", 3)});

  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_EQ(state.last_node_id, "node_2");
  ASSERT_TRUE(state.new_base_request.has_value());
  EXPECT_TRUE(state.new_base_request.value());
  // node_4 (horizon) is kept, still unreleased.
  ASSERT_EQ(state.node_states.size(), 1u);
  EXPECT_FALSE(state.node_states.front().released);
}

// Test 4: Traversing the final node empties the base.
TEST(OrderTraversalTest, ClearsStateWhenOrderComplete)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(context, {node_state("node_4", 4, true)}, {edge_state("e3", 3)});

  context->provider()->push<NodeReachedUpdate>("node_4", 4);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_TRUE(state.node_states.empty());
  EXPECT_EQ(state.last_node_id, "node_4");
  EXPECT_EQ(state.last_node_sequence_id, 4u);
}

// Test 5: The next released node is dispatched via a NavigateToNodeEvent.
TEST(OrderTraversalTest, DispatchesNextNode)
{
  OrderTraversal strategy;
  auto context = make_context();

  auto node_2 = node_state("node_2", 2, true);
  auto node_4 = node_state("node_4", 4, true);
  node_4.node_description = "Target node";

  auto e1 = edge_state("e1", 1);
  auto e3 = edge_state("e3", 3);

  seed(context, {node_2, node_4}, {e1, e3});

  std::shared_ptr<NavigateToNodeEvent> dispatched;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent> event) { dispatched = event; });

  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);

  ASSERT_NE(dispatched, nullptr);

  // Check NodeRequest conversion.
  EXPECT_EQ(dispatched->target.node_id(), "node_4");
  EXPECT_EQ(dispatched->target.sequence_id(), 4u);
  ASSERT_TRUE(dispatched->target.node_description().has_value());
  EXPECT_EQ(dispatched->target.node_description().value(), "Target node");

  // Check EdgeRequest conversion.
  ASSERT_TRUE(dispatched->via_edge.has_value());
  EXPECT_EQ(dispatched->via_edge->edge_id(), "e3");
  EXPECT_EQ(dispatched->via_edge->sequence_id(), 3u);
}

// Test 6: A node reported out of order (ahead of a still-pending node) is
// ignored so the in-between nodes are not orphaned.
TEST(OrderTraversalTest, IgnoresOutOfOrderNode)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, true)},
    {edge_state("e1", 1), edge_state("e3", 3)});

  // Report reaching node_4 while node_2 is still pending.
  context->provider()->push<NodeReachedUpdate>("node_4", 4);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  EXPECT_EQ(state.node_states.size(), 2u);
  EXPECT_TRUE(state.last_node_id.empty());
}

// Test 7: new_base_request stays false while the released base is plentiful.
TEST(OrderTraversalTest, DoesNotRequestBaseWhenPlentiful)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(
    context,
    {node_state("node_2", 2, true), node_state("node_4", 4, true),
     node_state("node_6", 6, true)},
    {edge_state("e1", 1), edge_state("e3", 3), edge_state("e5", 5)});

  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto state = execution->get_state();
  ASSERT_TRUE(state.new_base_request.has_value());
  EXPECT_FALSE(state.new_base_request.value());
}

// Test 8: A base extension resumes traversal without a new node-reached update.
TEST(OrderTraversalTest, ResumesAfterBaseExtension)
{
  OrderTraversal strategy;
  auto context = make_context();
  // node_2 is the last released base node; node_4 is horizon.
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, false)},
    {edge_state("e3", 3)});

  int dispatches = 0;
  std::shared_ptr<NavigateToNodeEvent> last;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent> event) {
      ++dispatches;
      last = event;
    });

  // Reach the decision point: traversal stops, nothing dispatched.
  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);
  EXPECT_EQ(dispatches, 0);

  // Master extends the base: node_4 becomes released. No new node-reached.
  auto execution = context->get_resource<OrderExecutionResource>();
  types::State state = execution->get_state();
  ASSERT_EQ(state.node_states.size(), 1u);
  state.node_states.front().released = true;
  execution->set_state(std::move(state));

  // The next step reconciles against the now-released base and dispatches.
  strategy.step(context);
  ASSERT_EQ(dispatches, 1);
  ASSERT_NE(last, nullptr);
  EXPECT_EQ(last->target.node_id(), "node_4");
}

// Test 9: Repeated steps with the same cached update are no-ops.
TEST(OrderTraversalTest, IsIdempotentAcrossSteps)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, true)},
    {edge_state("e1", 1), edge_state("e3", 3)});

  int dispatches = 0;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent>) { ++dispatches; });

  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);
  ASSERT_EQ(dispatches, 1);

  auto execution = context->get_resource<OrderExecutionResource>();
  const auto after_first = execution->get_state();

  // The cached node-reached update is still present; extra steps must be no-ops.
  strategy.step(context);
  strategy.step(context);
  EXPECT_EQ(dispatches, 1);
  EXPECT_EQ(execution->get_state(), after_first);
}

// Test 10: A stale update from an old order must not advance a new order.
TEST(OrderTraversalTest, StaleUpdateDoesNotAdvanceNewOrder)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(context, {node_state("node_2", 2, true)}, {edge_state("e1", 1)});

  // Order o1 reaches node_2; the update stays cached afterwards.
  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);

  // A new order o2 reuses node_2 at the same sequence; it is not yet reached.
  auto execution = context->get_resource<OrderExecutionResource>();
  types::State fresh;
  fresh.order_id = "o2";
  fresh.node_states = {
    node_state("node_2", 2, true), node_state("node_4", 4, true)};
  fresh.edge_states = {edge_state("e1", 1), edge_state("e3", 3)};
  execution->set_state(std::move(fresh));
  types::Order fresh_order;
  fresh_order.order_id = "o2";
  fresh_order.nodes = {
    node_from_state(node_state("node_2", 2, true)),
    node_from_state(node_state("node_4", 4, true))};
  fresh_order.edges = {
    edge_from_state(edge_state("e1", 1)), edge_from_state(edge_state("e3", 3))};
  execution->set_order(std::move(fresh_order));

  // Step with the stale o1 update still cached (no new node-reached pushed).
  strategy.step(context);

  const auto state = execution->get_state();
  // node_2 must NOT be consumed as reached for o2.
  ASSERT_EQ(state.node_states.size(), 2u);
  EXPECT_EQ(state.node_states.front().node_id, "node_2");
  EXPECT_TRUE(state.last_node_id.empty());
}

// Test 11: The first pending released node is dispatched before any node-reached update.
TEST(OrderTraversalTest, BootstrapsFirstNode)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, true)},
    {edge_state("e1", 1), edge_state("e3", 3)});

  std::shared_ptr<NavigateToNodeEvent> dispatched;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent> event) { dispatched = event; });

  // No node-reached pushed.
  strategy.step(context);

  ASSERT_NE(dispatched, nullptr);
  EXPECT_EQ(dispatched->target.node_id(), "node_2");
  ASSERT_TRUE(dispatched->via_edge.has_value());
  EXPECT_EQ(dispatched->via_edge->sequence_id(), 1u);
}

// Test 12: Traversal is inert while no order is executing.
TEST(OrderTraversalTest, DoesNothingWhenNotExecuting)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(context, {node_state("node_2", 2, true)}, {edge_state("e1", 1)});
  context->get_resource<OrderExecutionResource>()->set_executing_order(false);

  bool dispatched = false;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent>) { dispatched = true; });

  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);

  EXPECT_FALSE(dispatched);
  const auto state =
    context->get_resource<OrderExecutionResource>()->get_state();
  EXPECT_EQ(state.node_states.size(), 1u);  // unchanged
  EXPECT_TRUE(state.last_node_id.empty());
}

// Test 13: Traversal resets its cached index when a new order update arrives.
TEST(OrderTraversalTest, ResetsCachedIndexOnOrderUpdate)
{
  OrderTraversal strategy;
  auto context = make_context();

  seed(
    context,
    {
      node_state("node_2", 2, true),
      node_state("node_4", 4, true),
    },
    {
      edge_state("e1", 1),
      edge_state("e3", 3),
    });

  auto execution = context->get_resource<OrderExecutionResource>();

  std::vector<uint32_t> dispatched_sequences;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent> event) {
      dispatched_sequences.push_back(event->target.sequence_id());
    });

  // Initial order dispatches node_2.
  strategy.step(context);

  ASSERT_EQ(dispatched_sequences.size(), 1u);
  EXPECT_EQ(dispatched_sequences.back(), 2u);

  // Simulate an order update with a changed node-state range.
  types::State state = execution->get_state();
  state.order_update_id = 1;
  state.node_states = {
    node_state("node_4", 4, true),
    node_state("node_6", 6, true),
  };
  state.edge_states = {
    edge_state("e3", 3),
    edge_state("e5", 5),
  };
  execution->set_state(std::move(state));
  types::Order order = execution->get_order();
  order.order_update_id = 1;
  order.nodes = {
    node_from_state(node_state("node_4", 4, true)),
    node_from_state(node_state("node_6", 6, true)),
  };
  order.edges = {
    edge_from_state(edge_state("e3", 3)),
    edge_from_state(edge_state("e5", 5)),
  };
  execution->set_order(std::move(order));

  strategy.step(context);

  ASSERT_EQ(dispatched_sequences.size(), 2u);
  EXPECT_EQ(dispatched_sequences.back(), 4u);
}

// Test 14: Traversal resets its cached index when a new order arrives.
TEST(OrderTraversalTest, ResetsCachedIndexForNewOrder)
{
  OrderTraversal strategy;
  auto context = make_context();

  seed(
    context,
    {
      node_state("a_node_2", 2, true),
      node_state("a_node_4", 4, true),
    },
    {
      edge_state("a_edge_1", 1),
      edge_state("a_edge_3", 3),
    });

  auto execution = context->get_resource<OrderExecutionResource>();

  std::vector<std::string> dispatched_nodes;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent> event) {
      dispatched_nodes.push_back(event->target.node_id());
    });

  strategy.step(context);

  ASSERT_EQ(dispatched_nodes.size(), 1u);
  EXPECT_EQ(dispatched_nodes.back(), "a_node_2");

  types::State new_order;
  new_order.order_id = "o2";
  new_order.order_update_id = 0;
  new_order.node_states = {
    node_state("b_node_2", 2, true),
    node_state("b_node_4", 4, true),
  };
  new_order.edge_states = {
    edge_state("b_edge_1", 1),
    edge_state("b_edge_3", 3),
  };
  execution->set_state(std::move(new_order));
  types::Order new_full_order;
  new_full_order.order_id = "o2";
  new_full_order.nodes = {
    node_from_state(node_state("b_node_2", 2, true)),
    node_from_state(node_state("b_node_4", 4, true)),
  };
  new_full_order.edges = {
    edge_from_state(edge_state("b_edge_1", 1)),
    edge_from_state(edge_state("b_edge_3", 3)),
  };
  execution->set_order(std::move(new_full_order));

  strategy.step(context);

  ASSERT_EQ(dispatched_nodes.size(), 2u);
  EXPECT_EQ(dispatched_nodes.back(), "b_node_2");
}

// Test 15: Dispatch uses the full accepted Order for execution fields that are
// intentionally absent from EdgeState.
TEST(OrderTraversalTest, DispatchesEdgeRequestFromFullOrder)
{
  OrderTraversal strategy;
  auto context = make_context();

  auto node_2 = node_state("node_2", 2, true);
  auto e1_state = edge_state("e1", 1);

  types::Order order;
  order.order_id = "o1";
  order.nodes = {node_from_state(node_2)};
  auto e1 = edge_from_state(e1_state);
  e1.max_speed = 1.25;
  e1.min_height = 0.1;
  e1.max_height = 2.0;
  e1.rotation_allowed = false;
  e1.max_rotation_speed = 0.5;
  e1.length = 3.5;
  order.edges = {e1};

  seed(context, std::move(order), {node_2}, {e1_state});

  std::shared_ptr<NavigateToNodeEvent> dispatched;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent> event) { dispatched = event; });

  strategy.step(context);

  ASSERT_NE(dispatched, nullptr);
  ASSERT_TRUE(dispatched->via_edge.has_value());
  EXPECT_EQ(dispatched->via_edge->max_speed(), 1.25);
  EXPECT_EQ(dispatched->via_edge->min_height(), 0.1);
  EXPECT_EQ(dispatched->via_edge->max_height(), 2.0);
  EXPECT_EQ(dispatched->via_edge->rotation_allowed(), false);
  EXPECT_EQ(dispatched->via_edge->max_rotation_speed(), 0.5);
  EXPECT_EQ(dispatched->via_edge->length(), 3.5);
}

// Test 16: Traversal does not dispatch the same node twice in a row.
TEST(OrderTraversalTest, DoesNotDispatchWhenIncomingEdgeIsMissing)
{
  OrderTraversal strategy;
  auto context = make_context();

  auto node_2 = node_state("node_2", 2, true);

  types::Order order;
  order.order_id = "o1";
  order.nodes = {node_from_state(node_2)};
  // Deliberately omit edge sequence 1.

  seed(context, std::move(order), {node_2}, {});

  bool dispatched = false;
  strategy.engine()->on<NavigateToNodeEvent>(
    [&](std::shared_ptr<NavigateToNodeEvent>) { dispatched = true; });

  strategy.step(context);

  EXPECT_FALSE(dispatched);
}

// Test 17: Traversing a mid-order node emits node-traversed, edge-left
// (incoming), and edge-entered (next) so action strategies can trigger/stop
// actions.
TEST(OrderTraversalTest, EmitsTraversalEvents)
{
  OrderTraversal strategy;
  auto context = make_context();
  // node_0 already traversed; sitting on edge e1 toward node_2.
  seed(
    context, {node_state("node_2", 2, true), node_state("node_4", 4, true)},
    {edge_state("e1", 1), edge_state("e3", 3)});

  std::shared_ptr<NodeTraversedEvent> traversed;
  std::shared_ptr<EdgeLeftEvent> left;
  std::shared_ptr<EdgeEnteredEvent> entered;
  strategy.engine()->on<NodeTraversedEvent>(
    [&](std::shared_ptr<NodeTraversedEvent> event) { traversed = event; });
  strategy.engine()->on<EdgeLeftEvent>(
    [&](std::shared_ptr<EdgeLeftEvent> event) { left = event; });
  strategy.engine()->on<EdgeEnteredEvent>(
    [&](std::shared_ptr<EdgeEnteredEvent> event) { entered = event; });

  context->provider()->push<NodeReachedUpdate>("node_2", 2);
  strategy.step(context);

  ASSERT_NE(traversed, nullptr);
  EXPECT_EQ(traversed->node_id, "node_2");
  EXPECT_EQ(traversed->sequence_id, 2u);
  ASSERT_NE(left, nullptr);
  EXPECT_EQ(left->edge_id, "e1");
  ASSERT_NE(entered, nullptr);
  EXPECT_EQ(entered->edge_id, "e3");
}

// Test 18: At the final node there is no next edge, so no edge-entered fires.
TEST(OrderTraversalTest, NoEdgeEnteredAtFinalNode)
{
  OrderTraversal strategy;
  auto context = make_context();
  seed(context, {node_state("node_4", 4, true)}, {edge_state("e3", 3)});

  std::shared_ptr<NodeTraversedEvent> traversed;
  std::shared_ptr<EdgeEnteredEvent> entered;
  strategy.engine()->on<NodeTraversedEvent>(
    [&](std::shared_ptr<NodeTraversedEvent> event) { traversed = event; });
  strategy.engine()->on<EdgeEnteredEvent>(
    [&](std::shared_ptr<EdgeEnteredEvent> event) { entered = event; });

  context->provider()->push<NodeReachedUpdate>("node_4", 4);
  strategy.step(context);

  ASSERT_NE(traversed, nullptr);
  EXPECT_EQ(traversed->node_id, "node_4");
  EXPECT_EQ(entered, nullptr);
}

}  // namespace
