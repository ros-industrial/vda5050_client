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

#include "vda5050_core/master/actions/instant_actions_publisher.hpp"

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/master/validation/action_conflict_validator.hpp"
#include "vda5050_core/master/validation/instant_action_mode_validator.hpp"
#include "vda5050_core/master/validation/pre_send_validator.hpp"
#include "vda5050_core/master/validation/schema_validator.hpp"
#include "vda5050_core/master/validation/traversability_validator.hpp"

namespace vda5050_core::master {

vda5050_core::order_utils::ValidationResult InstantActionsPublisher::publish(
  vda5050_core::execution::ProtocolAdapter& adapter, const PreSendContext& ctx,
  const vda5050_core::types::InstantActions& actions)
{
  // Validator chain: schema → online → operating-mode gate →
  // traversability (capability) → action conflict. Each link
  // short-circuits on failure.
  //
  // Full pre-send is intentionally NOT applied: its strict order-centric
  // checks (AUTOMATIC mode, position initialized, AVAILABLE state) would
  // drop instant actions that are meant to work in degraded states —
  // cancelOrder during an error, initPosition before localization,
  // factsheetRequest with no prior state. Instead two targeted defenses
  // run inline:
  //   1. online check — QoS-0 sends to an offline AGV drop silently.
  //   2. mode gate — in non-automatic modes only the predefined
  //      instant-scope actions (the allowlist) may be sent.
  auto schema_result = validate_instant_actions_schema(actions);
  if (!schema_result)
  {
    return schema_result;
  }

  if (ctx.connection_status != vda5050_core::types::ConnectionState::ONLINE)
  {
    vda5050_core::order_utils::ValidationResult res;
    res.errors.push_back(vda5050_core::errors::create_error(
      vda5050_core::errors::PreSendValidationError,
      "AGV connection_status is not ONLINE", {}));
    return res;
  }

  auto mode_result = validate_instant_action_mode(ctx, actions);
  if (!mode_result)
  {
    return mode_result;
  }

  auto traversability_result = validate_traversability(ctx, actions);
  if (!traversability_result)
  {
    return traversability_result;
  }

  auto conflict_result = validate_action_conflict(ctx, actions);
  if (!conflict_result)
  {
    return conflict_result;
  }

  adapter.publish<vda5050_core::types::InstantActions>(
    actions, static_cast<int>(InstantActionsQos));

  return vda5050_core::order_utils::ValidationResult{};  // success
}

}  // namespace vda5050_core::master
