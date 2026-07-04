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

#include "vda5050_core/validation/capability_validator.hpp"

#include <fmt/format.h>
#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::validation {

namespace {

using AddErrorFn =
  std::function<void(const std::string&, std::vector<types::ErrorReference>)>;

const types::AGVAction* find_agv_action(
  const types::Factsheet& fs, const std::string& action_type)
{
  const auto& actions = fs.protocol_features.agv_actions;
  auto it = std::find_if(
    actions.begin(), actions.end(),
    [&](const types::AGVAction& a) { return a.action_type == action_type; });
  return (it == actions.end()) ? nullptr : &(*it);
}

void validate_action_against_factsheet(
  const types::Action& action, types::ActionScope expected_scope,
  const types::Factsheet& factsheet, const AddErrorFn& add_error)
{
  const types::AGVAction* agv_action =
    find_agv_action(factsheet, action.action_type);
  if (agv_action == nullptr)
  {
    add_error(
      fmt::format(
        "Action.action_type '{}' is not supported by AGV (not present in "
        "factsheet.agv_actions).",
        action.action_type),
      {});
    return;
  }

  const auto& scopes = agv_action->action_scopes;
  if (std::find(scopes.begin(), scopes.end(), expected_scope) == scopes.end())
  {
    add_error(
      fmt::format(
        "Action.action_type '{}' does not declare the required scope for its "
        "placement.",
        action.action_type),
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
        fmt::format(
          "Action.action_type '{}' does not support the requested "
          "blocking_type.",
          action.action_type),
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
        [&](const types::ActionParameterFactsheet& d) {
          return d.key == p.key;
        });
      if (it == declared.end())
      {
        add_error(
          fmt::format(
            "Action parameter key '{}' not declared by AGV for action_type "
            "'{}'.",
            p.key, action.action_type),
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
        fmt::format(
          "Action '{}' is missing required parameter '{}'.", action.action_type,
          d.key),
        {});
    }
  }
}

}  // namespace

errors::ValidationResult validate_capability(
  const PreSendContext& ctx, const types::Order& order)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    refs.push_back({errors::RefOrderId, order.order_id});
    res.add_error(errors::create_error(
      errors::CapabilityValidationError, description, refs));
  };

  if (!ctx.last_factsheet.has_value())
  {
    res.add_error(errors::create_error(
      errors::CapabilityCheckSkipped,
      "No factsheet cached; capability checks skipped.",
      {{errors::RefOrderId, order.order_id}}, types::ErrorLevel::WARNING));
    return res;
  }

  const auto& fs = ctx.last_factsheet.value();

  for (const auto& node : order.nodes)
  {
    for (const auto& action : node.actions)
    {
      validate_action_against_factsheet(
        action, types::ActionScope::NODE, fs, add_error);
    }
  }

  for (const auto& edge : order.edges)
  {
    for (const auto& action : edge.actions)
    {
      validate_action_against_factsheet(
        action, types::ActionScope::EDGE, fs, add_error);
    }
  }

  return res;
}

errors::ValidationResult validate_capability(
  const PreSendContext& ctx, const types::InstantActions& actions)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(errors::create_error(
      errors::CapabilityValidationError, description, refs));
  };

  if (!ctx.last_factsheet.has_value())
  {
    res.add_error(errors::create_error(
      errors::CapabilityCheckSkipped,
      "No factsheet cached; instant-action capability checks skipped.", {},
      types::ErrorLevel::WARNING));
    return res;
  }

  const auto& fs = ctx.last_factsheet.value();

  for (const auto& action : actions.actions)
  {
    validate_action_against_factsheet(
      action, types::ActionScope::INSTANT, fs, add_error);
  }

  return res;
}

}  // namespace vda5050_core::validation
