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

#include "vda5050_core/validation/content_validator.hpp"

#include <fmt/format.h>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/master/standard_names.hpp"

namespace vda5050_core::validation {

namespace {

using AddErrorFn =
  std::function<void(const std::string&, std::vector<types::ErrorReference>)>;

void validate_header_common(
  const types::Header& header, const AddErrorFn& add_error)
{
  const auto& supported = master::SupportedSchemaVersions;
  if (
    std::find(supported.begin(), supported.end(), header.version) ==
    supported.end())
  {
    add_error(
      fmt::format("Unsupported header version '{}'", header.version), {});
  }

  if (header.manufacturer.empty())
  {
    add_error("header.manufacturer must be non-empty", {});
  }

  if (header.serial_number.empty())
  {
    add_error("header.serial_number must be non-empty", {});
  }
}

void validate_action_content(
  const types::Action& action, const AddErrorFn& add_error)
{
  if (action.action_id.empty())
  {
    add_error("Action.action_id must be non-empty", {});
  }
  if (action.action_type.empty())
  {
    add_error(
      fmt::format(
        "Action.action_type must be non-empty (action_id='{}')",
        action.action_id),
      {});
  }
}

}  // namespace

errors::ValidationResult validate_order_content(const types::Order& order)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    refs.push_back({errors::RefOrderId, order.order_id});
    refs.push_back(
      {errors::RefOrderUpdateId, std::to_string(order.order_update_id)});
    res.add_error(
      errors::create_error(errors::ContentValidationError, description, refs));
  };

  validate_header_common(order.header, add_error);

  if (order.order_id.empty())
  {
    add_error("Order.order_id must be non-empty", {});
  }

  for (const auto& node : order.nodes)
  {
    if (node.node_id.empty())
    {
      add_error(
        "Node.node_id must be non-empty",
        {{errors::RefSequenceId, std::to_string(node.sequence_id)}});
    }
    for (const auto& action : node.actions)
    {
      validate_action_content(action, add_error);
    }
  }

  for (const auto& edge : order.edges)
  {
    if (edge.edge_id.empty())
    {
      add_error(
        "Edge.edge_id must be non-empty",
        {{errors::RefSequenceId, std::to_string(edge.sequence_id)}});
    }
    for (const auto& action : edge.actions)
    {
      validate_action_content(action, add_error);
    }
  }

  return res;
}

errors::ValidationResult validate_instant_actions_content(
  const types::InstantActions& actions)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(
      errors::create_error(errors::ContentValidationError, description, refs));
  };

  validate_header_common(actions.header, add_error);

  for (const auto& action : actions.actions)
  {
    validate_action_content(action, add_error);
  }

  return res;
}

errors::ValidationResult validate_state_content(const types::State& state)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(
      errors::create_error(errors::ContentValidationError, description, refs));
  };

  validate_header_common(state.header, add_error);

  // order_id may legitimately be empty (no current order).
  for (const auto& ns : state.node_states)
  {
    if (ns.node_id.empty())
    {
      add_error(
        "State.node_states[].node_id must be non-empty when populated",
        {{errors::RefSequenceId, std::to_string(ns.sequence_id)}});
    }
  }

  for (const auto& as : state.action_states)
  {
    if (as.action_id.empty())
    {
      add_error(
        "State.action_states[].action_id must be non-empty",
        {{errors::RefActionType, as.action_type.value_or("")}});
    }
  }

  for (const auto& err : state.errors)
  {
    if (err.error_type.empty())
    {
      add_error("State.errors[].error_type must be non-empty", {});
    }
  }

  return res;
}

errors::ValidationResult validate_connection_content(
  const types::Connection& connection)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(
      errors::create_error(errors::ContentValidationError, description, refs));
  };

  validate_header_common(connection.header, add_error);
  return res;
}

errors::ValidationResult validate_factsheet_content(
  const types::Factsheet& factsheet)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(
      errors::create_error(errors::ContentValidationError, description, refs));
  };

  validate_header_common(factsheet.header, add_error);
  return res;
}

errors::ValidationResult validate_visualization_content(
  const types::Visualization& visualization)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(
      errors::create_error(errors::ContentValidationError, description, refs));
  };

  validate_header_common(visualization.header, add_error);
  return res;
}

}  // namespace vda5050_core::validation
