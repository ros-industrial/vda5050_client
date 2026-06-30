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

#include "vda5050_core/master/validation/capability_validator.hpp"

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::master {

namespace {

using ::vda5050_core::errors::CapabilityValidationError;
using ::vda5050_core::errors::create_error;
using ::vda5050_core::order_utils::ValidationResult;
using ::vda5050_core::types::AGVAction;
using ::vda5050_core::types::ErrorReference;

using AddErrorFn =
  std::function<void(const std::string&, std::vector<ErrorReference>)>;

const AGVAction* find_agv_action(
  const vda5050_core::types::Factsheet& fs, const std::string& action_type)
{
  const auto& actions = fs.protocol_features.agv_actions;
  auto it = std::find_if(
    actions.begin(), actions.end(),
    [&](const AGVAction& a) { return a.action_type == action_type; });
  return (it == actions.end()) ? nullptr : &(*it);
}

void validate_action_against_factsheet(
  const vda5050_core::types::Action& action,
  vda5050_core::types::ActionScope expected_scope,
  const vda5050_core::types::Factsheet& factsheet, const AddErrorFn& add_error)
{
  const AGVAction* agv_action = find_agv_action(factsheet, action.action_type);
  if (agv_action == nullptr)
  {
    add_error(
      "Action.action_type '" + action.action_type +
        "' is not supported by AGV (not present in factsheet.agv_actions).",
      {});
    return;
  }

  const auto& scopes = agv_action->action_scopes;
  if (std::find(scopes.begin(), scopes.end(), expected_scope) == scopes.end())
  {
    add_error(
      "Action.action_type '" + action.action_type +
        "' does not declare the required scope for its placement.",
      {});
  }

  if (agv_action->blocking_types.has_value())
  {
    const auto& supported = agv_action->blocking_types.value();
    if (
      std::find(supported.begin(), supported.end(), action.blocking_type) ==
      supported.end())
    {
      add_error(
        "Action.action_type '" + action.action_type +
          "' does not support the requested blocking_type.",
        {});
    }
  }

  if (!agv_action->action_parameters.has_value()) return;
  const auto& declared = agv_action->action_parameters.value();

  if (action.action_parameters.has_value())
  {
    for (const auto& p : action.action_parameters.value())
    {
      auto it = std::find_if(
        declared.begin(), declared.end(),
        [&](const vda5050_core::types::ActionParameterFactsheet& d) {
          return d.key == p.key;
        });
      if (it == declared.end())
      {
        add_error(
          "Action parameter key '" + p.key +
            "' not declared by AGV for action_type '" + action.action_type +
            "'.",
          {});
      }
    }
  }

  for (const auto& d : declared)
  {
    if (d.is_optional.value_or(false)) continue;
    const bool present =
      action.action_parameters.has_value() &&
      std::any_of(
        action.action_parameters->begin(), action.action_parameters->end(),
        [&](const auto& p) { return p.key == d.key; });
    if (!present)
    {
      add_error(
        "Action '" + action.action_type + "' is missing required parameter '" +
          d.key + "'.",
        {});
    }
  }
}

}  // namespace

ValidationResult validate_capability(
  const PreSendContext& ctx, const vda5050_core::types::Order& order)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      refs.push_back({::vda5050_core::errors::RefOrderId, order.order_id});
      res.errors.push_back(
        create_error(CapabilityValidationError, description, refs));
    };

  if (!ctx.last_factsheet.has_value())
  {
    res.errors.push_back(create_error(
      ::vda5050_core::errors::CapabilityCheckSkipped,
      "No factsheet cached; capability checks skipped.",
      {{::vda5050_core::errors::RefOrderId, order.order_id}},
      vda5050_core::types::ErrorLevel::WARNING));
    return res;
  }

  const auto& fs = ctx.last_factsheet.value();

  for (const auto& node : order.nodes)
  {
    for (const auto& action : node.actions)
    {
      validate_action_against_factsheet(
        action, vda5050_core::types::ActionScope::NODE, fs, add_error);
    }
  }

  for (const auto& edge : order.edges)
  {
    for (const auto& action : edge.actions)
    {
      validate_action_against_factsheet(
        action, vda5050_core::types::ActionScope::EDGE, fs, add_error);
    }
  }

  return res;
}

ValidationResult validate_capability(
  const PreSendContext& ctx, const vda5050_core::types::InstantActions& actions)
{
  ValidationResult res;

  auto add_error =
    [&](const std::string& description, std::vector<ErrorReference> refs) {
      res.errors.push_back(
        create_error(CapabilityValidationError, description, refs));
    };

  if (!ctx.last_factsheet.has_value())
  {
    res.errors.push_back(create_error(
      ::vda5050_core::errors::CapabilityCheckSkipped,
      "No factsheet cached; instant-action capability checks skipped.", {},
      vda5050_core::types::ErrorLevel::WARNING));
    return res;
  }

  const auto& fs = ctx.last_factsheet.value();

  for (const auto& action : actions.actions)
  {
    validate_action_against_factsheet(
      action, vda5050_core::types::ActionScope::INSTANT, fs, add_error);
  }

  return res;
}

}  // namespace vda5050_core::master
