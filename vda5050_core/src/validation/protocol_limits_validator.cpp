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

#include "vda5050_core/validation/protocol_limits_validator.hpp"

#include <fmt/format.h>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::validation {

namespace {

using AddErrorFn =
  std::function<void(const std::string&, std::vector<types::ErrorReference>)>;

void check_limit(
  const std::optional<uint32_t>& limit, std::size_t actual,
  const std::string& field_name, const AddErrorFn& add_error)
{
  if (limit.has_value() && actual > limit.value())
  {
    add_error(
      fmt::format(
        "Message exceeds factsheet limit for {} (size={}, max={}).", field_name,
        actual, limit.value()),
      {});
  }
}

}  // namespace

errors::ValidationResult validate_protocol_limits(
  const PreSendContext& ctx, const types::Order& order)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    refs.push_back({errors::RefOrderId, order.order_id});
    res.add_error(
      errors::create_error(errors::ProtocolLimitError, description, refs));
  };

  if (!ctx.last_factsheet.has_value())
  {
    res.add_error(errors::create_error(
      errors::ProtocolLimitCheckSkipped,
      "No factsheet cached; array-limit checks skipped.",
      {{errors::RefOrderId, order.order_id}}, types::ErrorLevel::WARNING));
    return res;
  }

  const auto& limits = ctx.last_factsheet->protocol_limits.max_array_lens;

  check_limit(limits.order_nodes, order.nodes.size(), "order_nodes", add_error);
  check_limit(limits.order_edges, order.edges.size(), "order_edges", add_error);

  for (const auto& node : order.nodes)
  {
    check_limit(
      limits.node_actions, node.actions.size(), "node_actions", add_error);
    for (const auto& action : node.actions)
    {
      check_limit(
        limits.actions_actions_parameters,
        action.action_parameters.has_value() ? action.action_parameters->size()
                                             : 0,
        "actions_actions_parameters", add_error);
    }
  }

  for (const auto& edge : order.edges)
  {
    check_limit(
      limits.edge_actions, edge.actions.size(), "edge_actions", add_error);
    for (const auto& action : edge.actions)
    {
      check_limit(
        limits.actions_actions_parameters,
        action.action_parameters.has_value() ? action.action_parameters->size()
                                             : 0,
        "actions_actions_parameters", add_error);
    }
  }

  return res;
}

errors::ValidationResult validate_protocol_limits(
  const PreSendContext& ctx, const types::InstantActions& actions)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(
      errors::create_error(errors::ProtocolLimitError, description, refs));
  };

  if (!ctx.last_factsheet.has_value())
  {
    res.add_error(errors::create_error(
      errors::ProtocolLimitCheckSkipped,
      "No factsheet cached; instant-action array-limit checks skipped.", {},
      types::ErrorLevel::WARNING));
    return res;
  }

  const auto& limits = ctx.last_factsheet->protocol_limits.max_array_lens;

  check_limit(
    limits.instant_actions, actions.actions.size(), "instant_actions",
    add_error);

  for (const auto& action : actions.actions)
  {
    check_limit(
      limits.actions_actions_parameters,
      action.action_parameters.has_value() ? action.action_parameters->size()
                                           : 0,
      "actions_actions_parameters", add_error);
  }

  return res;
}

}  // namespace vda5050_core::validation
