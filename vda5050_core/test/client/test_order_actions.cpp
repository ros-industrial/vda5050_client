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

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/client/contexts/agv_context.hpp"
#include "vda5050_core/client/events/edge_entered.hpp"
#include "vda5050_core/client/events/edge_left.hpp"
#include "vda5050_core/client/events/execute_action.hpp"
#include "vda5050_core/client/events/node_traversed.hpp"
#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/strategies/order_actions.hpp"
#include "vda5050_core/client/strategies/order_traversal.hpp"
#include "vda5050_core/client/updates/node_reached.hpp"
#include "vda5050_core/execution/engine.hpp"
#include "vda5050_core/execution/event_queue.hpp"

namespace {

using AGVContext = vda5050_core::client::AGVContext;
using EdgeEnteredEvent = vda5050_core::client::EdgeEnteredEvent;
using EdgeLeftEvent = vda5050_core::client::EdgeLeftEvent;
using ExecuteActionEvent = vda5050_core::client::ExecuteActionEvent;
using Engine = vda5050_core::execution::Engine;
using NodeReachedUpdate = vda5050_core::client::NodeReachedUpdate;
using NodeTraversedEvent = vda5050_core::client::NodeTraversedEvent;
using OrderActions = vda5050_core::client::OrderActions;
using OrderExecutionResource = vda5050_core::client::OrderExecutionResource;
using OrderTraversal = vda5050_core::client::OrderTraversal;
using Priority = vda5050_core::execution::Priority;
namespace types = vda5050_core::types;

std::shared_ptr<AGVContext> make_context()
{
  auto config = std::make_shared<vda5050_core::client::HeaderConfigResource>(
    "uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);
  context->init();
  return context;
}

types::Action make_action(
  const std::string& action_id, const std::string& action_type,
  types::BlockingType blocking = types::BlockingType::NONE)
{
  types::Action action;
  action.action_id = action_id;
  action.action_type = action_type;
  action.blocking_type = blocking;
  return action;
}

types::Node make_node(
  const std::string& node_id, uint32_t sequence_id,
  std::vector<types::Action> actions)
{
  types::Node node;
  node.node_id = node_id;
  node.sequence_id = sequence_id;
  node.released = true;
  node.actions = std::move(actions);
  return node;
}

types::Edge make_edge(
  const std::string& edge_id, uint32_t sequence_id,
  std::vector<types::Action> actions)
{
  types::Edge edge;
  edge.edge_id = edge_id;
  edge.sequence_id = sequence_id;
  edge.released = true;
  edge.actions = std::move(actions);
  return edge;
}

// Persist the order and seed every node/edge action as WAITING, mirroring what
// OrderAcceptance does when an order is accepted.
void accept_order(
  const std::shared_ptr<AGVContext>& context, const types::Order& order)
{
  auto execution = context->get_resource<OrderExecutionResource>();
  types::State state = execution->get_state();
  auto seed = [&state](const std::vector<types::Action>& actions) {
    for (const auto& action : actions)
    {
      types::ActionState action_state;
      action_state.action_id = action.action_id;
      action_state.action_type = action.action_type;
      action_state.action_status = types::ActionStatus::WAITING;
      state.action_states.push_back(action_state);
    }
  };
  for (const auto& node : order.nodes) seed(node.actions);
  for (const auto& edge : order.edges) seed(edge.actions);
  execution->set_state(std::move(state));
  execution->set_order(order);
  execution->set_executing_order(true);
}

std::optional<types::ActionState> action_state_of(
  const std::shared_ptr<AGVContext>& context, const std::string& action_id)
{
  auto execution = context->get_resource<OrderExecutionResource>();
  for (const auto& action_state : execution->get_state().action_states)
  {
    if (action_state.action_id == action_id) return action_state;
  }
  return std::nullopt;
}

void finish_actions(const std::shared_ptr<Engine>& source)
{
  source->on<ExecuteActionEvent>([](std::shared_ptr<ExecuteActionEvent> event) {
    event->execution->finished();
  });
}

// Test 1: When a node is traversed/reached, all actions attached to that node are executed.
TEST(OrderActionsTest, RunsNodeActionsOnReached)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(
    make_node("n2", 2, {make_action("a1", "pick"), make_action("a2", "beep")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  finish_actions(source);

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::FINISHED);
  EXPECT_EQ(
    action_state_of(context, "a2")->action_status,
    types::ActionStatus::FINISHED);
}

// Test 2: The action executor request contains correct information.
TEST(OrderActionsTest, PassesActionRequestToExecutor)
{
  auto context = make_context();
  types::Order order;
  order.order_id = "order_1";
  order.order_update_id = 3;
  order.nodes.push_back(
    make_node("n2", 2, {make_action("a1", "pick", types::BlockingType::HARD)}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);

  std::string action_id;
  std::string action_type;
  std::string order_id;
  std::optional<uint32_t> order_update_id;
  source->on<ExecuteActionEvent>(
    [&](std::shared_ptr<ExecuteActionEvent> event) {
      action_id = event->request.action_id();
      action_type = event->request.action_type();
      order_id = event->request.order_id().value_or("");
      order_update_id = event->request.order_update_id();
      event->execution->finished();
    });

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(action_id, "a1");
  EXPECT_EQ(action_type, "pick");
  EXPECT_EQ(order_id, "order_1");
  ASSERT_TRUE(order_update_id.has_value());
  EXPECT_EQ(order_update_id.value(), 3u);
}

// Test 3: The executor's status and result description are recorded.
TEST(OrderActionsTest, RecordsExecutorResult)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node("n2", 2, {make_action("a1", "pick")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  source->on<ExecuteActionEvent>([](std::shared_ptr<ExecuteActionEvent> event) {
    event->execution->failed("gripper jammed");
  });

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  const auto state = action_state_of(context, "a1");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->action_status, types::ActionStatus::FAILED);
  ASSERT_TRUE(state->result_description.has_value());
  EXPECT_EQ(state->result_description.value(), "gripper jammed");
}

// Test 4: When an edge is entered, actions attached to that edge start running.
TEST(OrderActionsTest, StartsEdgeActionsOnEntered)
{
  auto context = make_context();
  types::Order order;
  order.edges.push_back(make_edge("e3", 3, {make_action("a1", "blink")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  source->on<ExecuteActionEvent>([](std::shared_ptr<ExecuteActionEvent> event) {
    event->execution->running();
  });

  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::RUNNING);
}

// Test 5: If an edge action is still running, leaving the edge should finish it.
TEST(OrderActionsTest, StopsRunningEdgeActionsOnLeft)
{
  auto context = make_context();
  types::Order order;
  order.edges.push_back(make_edge("e3", 3, {make_action("a1", "blink")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  source->on<ExecuteActionEvent>([](std::shared_ptr<ExecuteActionEvent> event) {
    event->execution->running();
  });

  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();
  source->emit<EdgeLeftEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();

  const auto state = action_state_of(context, "a1");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->action_status, types::ActionStatus::FINISHED);
  EXPECT_EQ(state->result_description.value(), "completed: edge left");
}

// Test 6: Re-delivering the same signal does not execute an action twice.
TEST(OrderActionsTest, IsIdempotentOnRedelivery)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node("n2", 2, {make_action("a1", "pick")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);

  int calls = 0;
  source->on<ExecuteActionEvent>(
    [&calls](std::shared_ptr<ExecuteActionEvent> event) {
      ++calls;
      event->execution->finished();
    });

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();
  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(calls, 1);
  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::FINISHED);
}

// Test 7: If nobody finishes or fails the action, the action remains RUNNING.
TEST(OrderActionsTest, LeavesActionsRunningWithoutHandler)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node("n2", 2, {make_action("a1", "pick")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::RUNNING);
}

// Test 8: If the event does not match the current order node, do nothing.
TEST(OrderActionsTest, IgnoresUnknownNode)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node("n2", 2, {make_action("a1", "pick")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  finish_actions(source);

  // Same node_id but wrong sequence_id, then an entirely unknown node.
  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 9u);
  source->step();
  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("nX"), 5u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::WAITING);
}

void run_actions_for_types(
  const std::shared_ptr<Engine>& source, std::vector<std::string> running_types)
{
  source->on<ExecuteActionEvent>(
    [running_types](std::shared_ptr<ExecuteActionEvent> event) {
      const bool stays_running =
        std::find(
          running_types.begin(), running_types.end(),
          event->request.action_type()) != running_types.end();
      if (stays_running)
      {
        event->execution->running();
      }
      else
      {
        event->execution->finished();
      }
    });
}

void set_driving(const std::shared_ptr<AGVContext>& context, bool driving)
{
  auto execution = context->get_resource<OrderExecutionResource>();
  types::State state = execution->get_state();
  state.driving = driving;
  execution->set_state(std::move(state));
}

// Test 9: A HARD action should wait if another action is already running.
TEST(OrderActionsTest, HardActionDefersWhileAnotherActionRuns)
{
  auto context = make_context();
  types::Order order;
  order.edges.push_back(make_edge("e3", 3, {make_action("edge_a", "blink")}));
  order.nodes.push_back(make_node(
    "n4", 4, {make_action("hard_a", "lift", types::BlockingType::HARD)}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  run_actions_for_types(source, {"blink"});

  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();
  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n4"), 4u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "edge_a")->action_status,
    types::ActionStatus::RUNNING);
  EXPECT_EQ(
    action_state_of(context, "hard_a")->action_status,
    types::ActionStatus::WAITING);

  source->emit<EdgeLeftEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "edge_a")->action_status,
    types::ActionStatus::FINISHED);
  EXPECT_EQ(
    action_state_of(context, "hard_a")->action_status,
    types::ActionStatus::FINISHED);
}

// Test 10: If a HARD action is already running, later actions should not start.
TEST(OrderActionsTest, HardActionBlocksLaterActions)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node(
    "n1", 1, {make_action("hard_a", "lift", types::BlockingType::HARD)}));
  order.nodes.push_back(make_node("n2", 2, {make_action("none_a", "beep")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  run_actions_for_types(source, {"lift"});

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n1"), 1u);
  source->step();
  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "hard_a")->action_status,
    types::ActionStatus::RUNNING);
  EXPECT_EQ(
    action_state_of(context, "none_a")->action_status,
    types::ActionStatus::WAITING);
}

// Test 11: A SOFT action must not start while the AGV is driving, but a NONE
// action may; the SOFT action runs once the AGV stops (retried in step()).
TEST(OrderActionsTest, BlockingActionWaitsWhileDrivingThenRuns)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node(
    "n2", 2,
    {make_action("soft_a", "scan", types::BlockingType::SOFT),
     make_action("none_a", "beep")}));
  accept_order(context, order);
  set_driving(context, true);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  finish_actions(source);

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "soft_a")->action_status,
    types::ActionStatus::WAITING);
  EXPECT_EQ(
    action_state_of(context, "none_a")->action_status,
    types::ActionStatus::FINISHED);

  // The AGV stops; the next spin retries the deferred SOFT action.
  set_driving(context, false);
  actions->step(context);

  EXPECT_EQ(
    action_state_of(context, "soft_a")->action_status,
    types::ActionStatus::FINISHED);
}

// Test 12: NONE actions do not block each other and run concurrently.
TEST(OrderActionsTest, NoneActionsRunConcurrently)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node(
    "n2", 2, {make_action("a1", "beep"), make_action("a2", "blink")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  run_actions_for_types(source, {"beep", "blink"});

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::RUNNING);
  EXPECT_EQ(
    action_state_of(context, "a2")->action_status,
    types::ActionStatus::RUNNING);
}

// Test 13: End-to-end. OrderActions wired to OrderTraversal's engine reacts to a
// real node-reached signal: traversing n2 runs its node action and entering the
// next edge e3 starts that edge's action.
TEST(OrderActionsTest, IntegratesWithOrderTraversal)
{
  auto context = make_context();

  // Reaching n2 leaves the incoming edge e1 and enters e3 toward n4.
  types::Order order;
  order.nodes.push_back(make_node("n2", 2, {make_action("node_a", "pick")}));
  order.nodes.push_back(make_node("n4", 4, {}));
  order.edges.push_back(make_edge("e1", 1, {}));
  order.edges.push_back(make_edge("e3", 3, {make_action("edge_a", "blink")}));
  accept_order(context, order);

  // Seed the traversal tracking state (node/edge states still to traverse).
  auto execution = context->get_resource<OrderExecutionResource>();
  types::State state = execution->get_state();
  types::NodeState n2;
  n2.node_id = "n2";
  n2.sequence_id = 2;
  n2.released = true;
  types::NodeState n4;
  n4.node_id = "n4";
  n4.sequence_id = 4;
  n4.released = true;
  types::EdgeState e1;
  e1.edge_id = "e1";
  e1.sequence_id = 1;
  e1.released = true;
  types::EdgeState e3;
  e3.edge_id = "e3";
  e3.sequence_id = 3;
  e3.released = true;
  state.node_states = {n2, n4};
  state.edge_states = {e1, e3};
  execution->set_state(std::move(state));

  auto traversal = std::make_shared<OrderTraversal>();
  auto actions = OrderActions::make(traversal->engine());
  traversal->init(context);
  actions->init(context);
  run_actions_for_types(traversal->engine(), {"blink"});

  // A real node-reached signal flows through traversal, whose cascade events
  // reach OrderActions over the shared engine.
  context->provider()->push<NodeReachedUpdate>("n2", 2);
  traversal->step(context);
  actions->step(context);

  EXPECT_EQ(
    action_state_of(context, "node_a")->action_status,
    types::ActionStatus::FINISHED);
  EXPECT_EQ(
    action_state_of(context, "edge_a")->action_status,
    types::ActionStatus::RUNNING);
}

// Test 14: A HARD action must not start while the AGV is driving, but it runs
// once the AGV stops.
TEST(OrderActionsTest, HardActionWaitsWhileDrivingThenRuns)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node(
    "n2", 2, {make_action("hard_a", "lift", types::BlockingType::HARD)}));
  accept_order(context, order);
  set_driving(context, true);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  finish_actions(source);

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "hard_a")->action_status,
    types::ActionStatus::WAITING);

  // The AGV stops; the next spin retries the deferred HARD action.
  set_driving(context, false);
  actions->step(context);

  EXPECT_EQ(
    action_state_of(context, "hard_a")->action_status,
    types::ActionStatus::FINISHED);
}

// Test 15: A terminal FAILED action is not executed again when the same signal
// is redelivered.
TEST(OrderActionsTest, FailedActionIsNotRetriedOnRedelivery)
{
  auto context = make_context();
  types::Order order;
  order.nodes.push_back(make_node("n2", 2, {make_action("a1", "pick")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);

  int calls = 0;
  source->on<ExecuteActionEvent>(
    [&calls](std::shared_ptr<ExecuteActionEvent> event) {
      ++calls;
      event->execution->failed("failed once");
    });

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  source->emit<NodeTraversedEvent>(Priority::NORMAL, std::string("n2"), 2u);
  source->step();

  EXPECT_EQ(calls, 1);

  const auto state = action_state_of(context, "a1");
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->action_status, types::ActionStatus::FAILED);
  ASSERT_TRUE(state->result_description.has_value());
  EXPECT_EQ(state->result_description.value(), "failed once");
}

// Test 16: Leaving one edge only finishes running actions attached to that edge.
TEST(OrderActionsTest, EdgeLeftOnlyStopsActionsForThatEdge)
{
  auto context = make_context();
  types::Order order;
  order.edges.push_back(make_edge("e3", 3, {make_action("a_e3", "blink")}));
  order.edges.push_back(make_edge("e5", 5, {make_action("a_e5", "scan")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);

  source->on<ExecuteActionEvent>([](std::shared_ptr<ExecuteActionEvent> event) {
    event->execution->running();
  });

  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();

  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("e5"), 5u);
  source->step();

  source->emit<EdgeLeftEvent>(Priority::NORMAL, std::string("e3"), 3u);
  source->step();

  const auto e3_state = action_state_of(context, "a_e3");
  const auto e5_state = action_state_of(context, "a_e5");

  ASSERT_TRUE(e3_state.has_value());
  ASSERT_TRUE(e5_state.has_value());

  EXPECT_EQ(e3_state->action_status, types::ActionStatus::FINISHED);
  ASSERT_TRUE(e3_state->result_description.has_value());
  EXPECT_EQ(e3_state->result_description.value(), "completed: edge left");

  EXPECT_EQ(e5_state->action_status, types::ActionStatus::RUNNING);
}

// Test 17: A signal for an edge/sequence not in the order is ignored.
TEST(OrderActionsTest, IgnoresUnknownEdge)
{
  auto context = make_context();
  types::Order order;
  order.edges.push_back(make_edge("e3", 3, {make_action("a1", "blink")}));
  accept_order(context, order);

  auto source = std::make_shared<Engine>();
  auto actions = OrderActions::make(source);
  actions->init(context);
  finish_actions(source);

  // Same edge_id but wrong sequence_id, then an entirely unknown edge.
  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("e3"), 9u);
  source->step();
  source->emit<EdgeEnteredEvent>(Priority::NORMAL, std::string("eX"), 5u);
  source->step();

  EXPECT_EQ(
    action_state_of(context, "a1")->action_status,
    types::ActionStatus::WAITING);
}

}  // namespace
