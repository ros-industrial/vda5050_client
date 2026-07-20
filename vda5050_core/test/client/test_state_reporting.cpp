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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "vda5050_core/client/contexts/agv_context.hpp"
#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/strategies/state_reporting.hpp"

namespace {

using AGVContext = vda5050_core::client::AGVContext;
using OrderExecutionResource = vda5050_core::client::OrderExecutionResource;
using StateReporting = vda5050_core::client::StateReporting;
namespace types = vda5050_core::types;

std::shared_ptr<AGVContext> make_context()
{
  auto config = std::make_shared<vda5050_core::client::HeaderConfigResource>(
    "uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);
  context->init();
  return context;
}

types::NodeState node_state(const std::string& node_id, uint32_t sequence_id)
{
  types::NodeState n;
  n.node_id = node_id;
  n.sequence_id = sequence_id;
  n.released = true;
  return n;
}

types::ActionState action_state(
  const std::string& action_id, types::ActionStatus status)
{
  types::ActionState a;
  a.action_id = action_id;
  a.action_status = status;
  return a;
}

void set_state(
  const std::shared_ptr<AGVContext>& context, const types::State& state)
{
  context->get_resource<OrderExecutionResource>()->set_state(state);
}

// Test 1: The assembled State is handed to the reporter hook.
TEST(StateReportingTest, ReportsAssembledState)
{
  auto context = make_context();
  types::State seeded;
  seeded.order_id = "o1";
  seeded.order_update_id = 3;
  seeded.last_node_id = "n2";
  seeded.node_states = {node_state("n4", 4)};
  seeded.battery_state.battery_charge = 88.0;
  seeded.battery_state.charging = false;
  seeded.driving = true;
  seeded.paused = false;
  set_state(context, seeded);

  StateReporting strategy;
  strategy.init(context);
  std::vector<types::State> reports;
  strategy.set_reporter(
    [&reports](const types::State& s) { reports.push_back(s); });

  strategy.step(context);

  ASSERT_EQ(reports.size(), 1u);
  EXPECT_EQ(reports.front().order_id, "o1");
  EXPECT_EQ(reports.front().order_update_id, 3u);
  EXPECT_EQ(reports.front().last_node_id, "n2");
  ASSERT_EQ(reports.front().node_states.size(), 1u);
  EXPECT_EQ(reports.front().node_states.front().node_id, "n4");
  EXPECT_DOUBLE_EQ(reports.front().battery_state.battery_charge, 88.0);
  EXPECT_FALSE(reports.front().battery_state.charging);
  EXPECT_TRUE(reports.front().driving);
  ASSERT_TRUE(reports.front().paused.has_value());
  EXPECT_FALSE(*reports.front().paused);
}

// Test 2: The State is reported only when it changes.
TEST(StateReportingTest, ReportsOnlyOnChange)
{
  auto context = make_context();
  types::State seeded;
  seeded.order_id = "o1";
  set_state(context, seeded);

  StateReporting strategy;
  strategy.init(context);
  std::vector<types::State> reports;
  strategy.set_reporter(
    [&reports](const types::State& s) { reports.push_back(s); });

  strategy.step(context);  // first report
  strategy.step(context);  // unchanged -> no report
  EXPECT_EQ(reports.size(), 1u);

  types::State changed = seeded;
  changed.order_update_id = 1;
  set_state(context, changed);
  strategy.step(context);  // changed -> report
  EXPECT_EQ(reports.size(), 2u);
}

// Test 3: A fully traversed order with terminal actions is marked not executing.
TEST(StateReportingTest, ClearsExecutingWhenOrderComplete)
{
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();
  execution->set_executing_order(true);

  types::State seeded;
  seeded.order_id = "o1";
  seeded.last_node_id = "n_last";
  // Route done (no node/edge states) and the only action is terminal.
  seeded.action_states = {action_state("a1", types::ActionStatus::FINISHED)};
  execution->set_state(seeded);

  StateReporting strategy;
  strategy.init(context);
  strategy.step(context);

  EXPECT_FALSE(execution->is_executing_order());
}

// Test 4: FAILED is also a terminal action status.
TEST(StateReportingTest, ClearsExecutingWhenActionFailedTerminal)
{
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();
  execution->set_executing_order(true);

  types::State seeded;
  seeded.action_states = {action_state("a1", types::ActionStatus::FAILED)};
  execution->set_state(seeded);

  StateReporting strategy;
  strategy.init(context);
  strategy.step(context);

  EXPECT_FALSE(execution->is_executing_order());
}

// Test 5: An order is not complete while nodes remain to traverse.
TEST(StateReportingTest, KeepsExecutingWhileNodeNotDone)
{
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();
  execution->set_executing_order(true);

  types::State seeded;
  seeded.node_states = {node_state("n4", 4)};  // still a node to visit
  execution->set_state(seeded);

  StateReporting strategy;
  strategy.init(context);
  strategy.step(context);

  EXPECT_TRUE(execution->is_executing_order());
}

// Test 6: An order is not complete while edges remain to traverse.
TEST(StateReportingTest, KeepsExecutingWhileEdgeNotDone)
{
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();
  execution->set_executing_order(true);

  types::State seeded;
  types::EdgeState edge;
  edge.edge_id = "e1";
  edge.sequence_id = 1;
  edge.released = true;
  seeded.edge_states = {edge};

  execution->set_state(seeded);

  StateReporting strategy;
  strategy.init(context);
  strategy.step(context);

  EXPECT_TRUE(execution->is_executing_order());
}

// Test 7: An order is not complete while an action is still running.
TEST(StateReportingTest, KeepsExecutingWhileActionPending)
{
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();
  execution->set_executing_order(true);

  types::State seeded;
  // Route done, but an action has not reached a terminal status.
  seeded.action_states = {action_state("a1", types::ActionStatus::RUNNING)};
  execution->set_state(seeded);

  StateReporting strategy;
  strategy.init(context);
  strategy.step(context);

  EXPECT_TRUE(execution->is_executing_order());
}

// Test 8: With no reporter registered, step() still finalizes completion safely.
TEST(StateReportingTest, FinalizesCompletionWithoutReporter)
{
  auto context = make_context();
  auto execution = context->get_resource<OrderExecutionResource>();
  execution->set_executing_order(true);
  execution->set_state(types::State{});  // empty: route done, no actions

  StateReporting strategy;
  strategy.init(context);
  strategy.step(context);  // no reporter set

  EXPECT_FALSE(execution->is_executing_order());
}

// Test 9: Test for other non-terminal action statuses.
TEST(StateReportingTest, KeepsExecutingWhileActionNonTerminal)
{
  const std::vector<types::ActionStatus> non_terminal_statuses = {
    types::ActionStatus::WAITING,
    types::ActionStatus::INITIALIZING,
    types::ActionStatus::RUNNING,
    types::ActionStatus::PAUSED,
  };

  for (const auto status : non_terminal_statuses)
  {
    auto context = make_context();
    auto execution = context->get_resource<OrderExecutionResource>();
    execution->set_executing_order(true);

    types::State seeded;
    seeded.action_states = {action_state("a1", status)};
    execution->set_state(seeded);

    StateReporting strategy;
    strategy.init(context);
    strategy.step(context);

    EXPECT_TRUE(execution->is_executing_order());
  }
}

// Test 10: An unchanged State is periodically reported as a heartbeat.
TEST(StateReportingTest, ReportsHeartbeatWhenStateIsUnchanged)
{
  auto context = make_context();
  types::State seeded;
  seeded.order_id = "o1";
  set_state(context, seeded);

  StateReporting strategy(std::chrono::milliseconds::zero());
  strategy.init(context);

  std::vector<types::State> reports;
  strategy.set_reporter(
    [&reports](const types::State& s) { reports.push_back(s); });

  strategy.step(context);
  strategy.step(context);

  ASSERT_EQ(reports.size(), 2u);
  EXPECT_EQ(reports[0], seeded);
  EXPECT_EQ(reports[1], seeded);
}

}  // namespace
