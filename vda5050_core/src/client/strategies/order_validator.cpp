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

#include "vda5050_core/client/strategies/order_validator.hpp"

#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/validation/order_graph_validator.hpp"

namespace vda5050_core {

namespace client {

namespace {

void attach_order_refs(types::Error& error, const types::Order& order)
{
  if (!error.error_references) error.error_references.emplace();
  error.error_references->push_back({errors::RefOrderId, order.order_id});
  error.error_references->push_back(
    {errors::RefOrderUpdateId, std::to_string(order.order_update_id)});
}

types::Error make_error(
  const std::string& type, const std::string& description,
  const types::Order& order, std::vector<types::ErrorReference> refs = {})
{
  refs.push_back({errors::RefOrderId, order.order_id});
  refs.push_back(
    {errors::RefOrderUpdateId, std::to_string(order.order_update_id)});
  return errors::create_error(type, description, refs);
}

AcceptanceResult rejected(types::Error error)
{
  VDA5050_WARN_STREAM(
    "OrderValidator rejected order: " << error.error_type << " - "
                                      << error.error_description.value_or(""));
  AcceptanceResult res;
  res.outcome = AcceptanceOutcome::REJECTED;
  res.errors.push_back(std::move(error));
  return res;
}

AcceptanceResult rejected(std::vector<types::Error> errors)
{
  AcceptanceResult res;
  res.outcome = AcceptanceOutcome::REJECTED;
  res.errors = std::move(errors);
  return res;
}

AcceptanceResult ignored()
{
  VDA5050_INFO_STREAM("OrderValidator ignored duplicate order update");
  AcceptanceResult res;
  res.outcome = AcceptanceOutcome::IGNORED;
  return res;
}

AcceptanceResult accepted()
{
  AcceptanceResult res;
  res.outcome = AcceptanceOutcome::ACCEPTED;
  return res;
}

// The vehicle is "busy" if it still has nodes left
// to traverse (base or horizon) or any action that is not yet FINISHED/FAILED.
// Edges never outlive their nodes, so checking node_states is sufficient.
//
// LIMITATION: this PR does not yet remove nodes as the AGV reaches them, so
// node_states stays populated for the lifetime of an accepted order. Until
// traversal/state-update handling lands, an accepted order keeps the vehicle
// "busy" and new orders are rejected until its state is cleared.
bool is_busy(const types::State& state)
{
  if (!state.node_states.empty()) return true;

  for (const auto& action : state.action_states)
  {
    if (
      action.action_status != types::ActionStatus::FINISHED &&
      action.action_status != types::ActionStatus::FAILED)
    {
      return true;
    }
  }
  return false;
}

}  // namespace

AcceptanceResult OrderValidator::validate_order(
  const types::Order& incoming_order,
  const std::shared_ptr<execution::ContextInterface>& context) const
{
  if (!context)
  {
    return rejected(
      errors::create_error(errors::ValidationError, "context is null", {}));
  }

  auto execution = context->get_resource<OrderExecutionResource>();
  if (!execution)
  {
    return rejected(errors::create_error(
      errors::ValidationError, "OrderExecutionResource is null", {}));
  }

  if (auto graph = validation::is_valid_graph(incoming_order); !graph)
  {
    for (auto& error : graph.errors) attach_order_refs(error, incoming_order);
    return rejected(std::move(graph.errors));
  }

  // TODO(eileentyz): Add AGV capability/factsheet validation once capability
  // data is available.

  // Read one consistent snapshot of the execution state; every decision below
  // (busy check, duplicate/deprecated update, stitch match) reads from this
  // same snapshot so they cannot disagree about the moment they observed.
  const types::State current_state = execution->get_state();
  const std::string current_id = current_state.order_id;
  const uint32_t current_update_id = current_state.order_update_id;

  // Brand new order, or an update of the active one?
  if (incoming_order.order_id != current_id)
  {
    if (is_busy(current_state))
    {
      return rejected(make_error(
        errors::ValidationError,
        "Cannot accept a new order while the vehicle is still executing or "
        "holding a horizon",
        incoming_order));
    }

    // TODO(eileentyz): Add start-node reachability validation once AGV position
    // tracking is available.

    return accepted();
  }

  if (incoming_order.order_update_id == current_update_id)
  {
    return ignored();
  }

  if (incoming_order.order_update_id < current_update_id)
  {
    return rejected(make_error(
      errors::OrderUpdateError,
      "Order update is deprecated (older than the active orderUpdateId)",
      incoming_order));
  }

  if (incoming_order.nodes.empty())
  {
    return rejected(make_error(
      errors::ValidationError,
      "Order update must contain at least one node for stitching",
      incoming_order));
  }

  std::string base_end_id = current_state.last_node_id;
  uint32_t base_end_sequence_id = current_state.last_node_sequence_id;
  for (const auto& node : current_state.node_states)
  {
    if (node.released && node.sequence_id > base_end_sequence_id)
    {
      base_end_id = node.node_id;
      base_end_sequence_id = node.sequence_id;
    }
  }

  const auto& stitch_node = incoming_order.nodes.front();
  if (
    stitch_node.node_id != base_end_id ||
    stitch_node.sequence_id != base_end_sequence_id)
  {
    return rejected(make_error(
      errors::OrderUpdateError,
      "Order update stitch node does not match the last node of the current "
      "base (the decision point)",
      incoming_order,
      {{errors::RefNodeId, stitch_node.node_id},
       {errors::RefSequenceId, std::to_string(stitch_node.sequence_id)}}));
  }

  return accepted();
}

}  // namespace client
}  // namespace vda5050_core
