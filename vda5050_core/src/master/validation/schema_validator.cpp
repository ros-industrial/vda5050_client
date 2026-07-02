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

#include "vda5050_core/master/validation/schema_validator.hpp"

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/master/standard_names.hpp"

namespace vda5050_core::master {

namespace {

using ::vda5050_core::errors::create_error;
using ::vda5050_core::errors::SchemaValidationError;
using ::vda5050_core::order_utils::ValidationResult;
using ::vda5050_core::types::ErrorReference;

using AddErrorFn =
  std::function<void(const std::string&, std::vector<ErrorReference>)>;

void validate_header_common(
  const ::vda5050_core::types::Header& header, const AddErrorFn& add_error)
{
  const auto& supported = SupportedSchemaVersions;
  if (
    std::find(supported.begin(), supported.end(), header.version) ==
    supported.end())
  {
    add_error(
      "header.version '" + header.version +
        "' is not in SupportedSchemaVersions",
      {});
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

void validate_action_schema(
  const ::vda5050_core::types::Action& action, const AddErrorFn& add_error)
{
  if (action.action_id.empty())
  {
    add_error("Action.action_id must be non-empty", {});
  }
  if (action.action_type.empty())
  {
    add_error(
      "Action.action_type must be non-empty (action_id='" + action.action_id +
        "')",
      {});
  }
}

}  // namespace

ValidationResult validate_order_schema(
  const ::vda5050_core::types::Order& order)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      refs.push_back({::vda5050_core::errors::RefOrderId, order.order_id});
      refs.push_back(
        {::vda5050_core::errors::RefOrderUpdateId,
         std::to_string(order.order_update_id)});
      res.errors.push_back(
        create_error(SchemaValidationError, description, refs));
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
        {{::vda5050_core::errors::RefSequenceId,
          std::to_string(node.sequence_id)}});
    }
    for (const auto& action : node.actions)
    {
      validate_action_schema(action, add_error);
    }
  }

  for (const auto& edge : order.edges)
  {
    if (edge.edge_id.empty())
    {
      add_error(
        "Edge.edge_id must be non-empty",
        {{::vda5050_core::errors::RefSequenceId,
          std::to_string(edge.sequence_id)}});
    }
    for (const auto& action : edge.actions)
    {
      validate_action_schema(action, add_error);
    }
  }

  return res;
}

ValidationResult validate_instant_actions_schema(
  const ::vda5050_core::types::InstantActions& actions)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      res.errors.push_back(
        create_error(SchemaValidationError, description, refs));
    };

  validate_header_common(actions.header, add_error);

  for (const auto& action : actions.actions)
  {
    validate_action_schema(action, add_error);
  }

  return res;
}

ValidationResult validate_state_schema(
  const ::vda5050_core::types::State& state)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      res.errors.push_back(
        create_error(SchemaValidationError, description, refs));
    };

  validate_header_common(state.header, add_error);

  // order_id may legitimately be empty (no current order).
  for (const auto& ns : state.node_states)
  {
    if (ns.node_id.empty())
    {
      add_error(
        "State.node_states[].node_id must be non-empty when populated",
        {{::vda5050_core::errors::RefSequenceId,
          std::to_string(ns.sequence_id)}});
    }
  }

  for (const auto& as : state.action_states)
  {
    if (as.action_id.empty())
    {
      add_error("State.action_states[].action_id must be non-empty", {});
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

ValidationResult validate_connection_schema(
  const ::vda5050_core::types::Connection& connection)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      res.errors.push_back(
        create_error(SchemaValidationError, description, refs));
    };

  validate_header_common(connection.header, add_error);
  return res;
}

ValidationResult validate_factsheet_schema(
  const ::vda5050_core::types::Factsheet& factsheet)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      res.errors.push_back(
        create_error(SchemaValidationError, description, refs));
    };

  validate_header_common(factsheet.header, add_error);
  return res;
}

ValidationResult validate_visualization_schema(
  const ::vda5050_core::types::Visualization& visualization)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      res.errors.push_back(
        create_error(SchemaValidationError, description, refs));
    };

  validate_header_common(visualization.header, add_error);
  return res;
}

}  // namespace vda5050_core::master
