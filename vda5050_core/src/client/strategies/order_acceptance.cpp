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

#include "vda5050_core/client/strategies/order_acceptance.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>

#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/updates/order.hpp"
#include "vda5050_core/types/action_status.hpp"

namespace vda5050_core {

namespace client {

namespace {

types::NodeState to_node_state(const types::Node& node)
{
  types::NodeState state;
  state.node_id = node.node_id;
  state.sequence_id = node.sequence_id;
  state.node_description = node.node_description;
  state.node_position = node.node_position;
  state.released = node.released;
  return state;
}

types::EdgeState to_edge_state(const types::Edge& edge)
{
  types::EdgeState state;
  state.edge_id = edge.edge_id;
  state.sequence_id = edge.sequence_id;
  state.edge_description = edge.edge_description;
  state.released = edge.released;
  state.trajectory = edge.trajectory;
  return state;
}

types::ActionState to_action_state(const types::Action& action)
{
  types::ActionState state;
  state.action_id = action.action_id;
  state.action_type = action.action_type;
  state.action_description = action.action_description;
  state.action_status = types::ActionStatus::WAITING;
  return state;
}

void append_node(types::State& state, const types::Node& node)
{
  state.node_states.push_back(to_node_state(node));
  for (const auto& action : node.actions)
  {
    state.action_states.push_back(to_action_state(action));
  }
}

void append_edge(types::State& state, const types::Edge& edge)
{
  state.edge_states.push_back(to_edge_state(edge));
  for (const auto& action : edge.actions)
  {
    state.action_states.push_back(to_action_state(action));
  }
}

void apply_new_order(types::State& state, const types::Order& order)
{
  state.node_states.clear();
  state.edge_states.clear();
  state.action_states.clear();
  state.order_id = order.order_id;
  state.order_update_id = order.order_update_id;

  // A new order starts a fresh traversal, so clear any progress carried over
  // from a previous order. Otherwise the stale last-reached node could be
  // mistaken for this order's decision point when a later update is stitched.
  state.last_node_id.clear();
  state.last_node_sequence_id = 0;
  state.new_base_request = std::nullopt;

  for (const auto& node : order.nodes) append_node(state, node);
  for (const auto& edge : order.edges) append_edge(state, edge);
}

void apply_order_update(types::State& state, const types::Order& order)
{
  state.order_update_id = order.order_update_id;

  // Drop the previous horizon (unreleased) node/edge states; the update
  // re-supplies them. The first node of the update is the re-sent stitch node
  // (already present in the base), so it is skipped when appending.
  state.node_states.erase(
    std::remove_if(
      state.node_states.begin(), state.node_states.end(),
      [](const types::NodeState& n) { return !n.released; }),
    state.node_states.end());
  state.edge_states.erase(
    std::remove_if(
      state.edge_states.begin(), state.edge_states.end(),
      [](const types::EdgeState& e) { return !e.released; }),
    state.edge_states.end());

  for (size_t i = 1; i < order.nodes.size(); ++i)
  {
    append_node(state, order.nodes[i]);
  }

  for (size_t i = 0; i < order.edges.size(); ++i)
  {
    append_edge(state, order.edges[i]);
  }

  // Deduplicate action states after replacing the horizon. This prevents
  // re-sent horizon actions from accumulating while preserving first-seen
  // action status.
  // TODO(eileentyz): reconcile action_states fully (also drop actions for
  // horizon nodes/edges that an update removes) once node->action tracking
  // exists.
  std::unordered_set<std::string> seen_action_ids;
  state.action_states.erase(
    std::remove_if(
      state.action_states.begin(), state.action_states.end(),
      [&seen_action_ids](const types::ActionState& a) {
        return !seen_action_ids.insert(a.action_id).second;
      }),
    state.action_states.end());
}

}  // namespace

OrderAcceptance::OrderAcceptance() = default;

OrderAcceptance::OrderAcceptance(OrderValidator validator)
: validator_(std::move(validator))
{
}

void OrderAcceptance::init(
  std::shared_ptr<execution::ContextInterface> /*context*/)
{
  // Nothing to initialise; the validator is stateless and reads everything it
  // needs from the context on each step().
}

void OrderAcceptance::step(std::shared_ptr<execution::ContextInterface> context)
{
  if (!context) return;

  // The inbound order is whatever the protocol adapter (or a test) last pushed.
  auto update = context->get_update<OrderUpdate>();
  if (!update) return;  // nothing to process yet

  const types::Order& order = update->order;

  if (
    order.order_id == last_order_id_ &&
    order.order_update_id == last_order_update_id_)
  {
    return;
  }

  const AcceptanceResult result = validator_.validate_order(order, context);

  auto execution = context->get_resource<OrderExecutionResource>();
  if (!execution) return;

  switch (result.outcome)
  {
    case AcceptanceOutcome::ACCEPTED: {
      last_order_id_ = order.order_id;
      last_order_update_id_ = order.order_update_id;
      types::State state = execution->get_state();
      // A successful acceptance supersedes any prior rejection, so clear stale
      // errors before applying the new order or update.
      state.errors.clear();
      const bool is_update = (order.order_id == state.order_id);
      if (is_update)
      {
        apply_order_update(state, order);
      }
      else
      {
        apply_new_order(state, order);
      }
      execution->set_state(std::move(state));
      execution->set_executing_order(true);
      break;
    }
    case AcceptanceOutcome::REJECTED: {
      // Replace errors instead of appending, since step() may re-process the
      // same cached rejected order.
      types::State state = execution->get_state();
      state.errors = result.errors;
      execution->set_state(std::move(state));
      break;
    }
    case AcceptanceOutcome::IGNORED:
      break;  // duplicate resend; leave the execution state untouched
  }
}

}  // namespace client
}  // namespace vda5050_core
