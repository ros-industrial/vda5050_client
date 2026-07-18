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

#include <utility>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/validation/action_conflict_validator.hpp"
#include "vda5050_core/validation/capability_validator.hpp"
#include "vda5050_core/validation/content_validator.hpp"
#include "vda5050_core/validation/instant_action_mode_validator.hpp"
#include "vda5050_core/validation/pre_send_validator.hpp"
#include "vda5050_core/validation/protocol_limits_validator.hpp"

namespace vda5050_core::master {

ActionGateResult InstantActionsPublisher::validate_gate(
  const vda5050_core::validation::PreSendContext& ctx,
  const vda5050_core::types::InstantActions& actions)
{
  ActionGateResult gate;

  auto mode =
    vda5050_core::validation::validate_instant_action_mode(ctx, actions);
  if (!mode)
  {
    gate.result = std::move(mode);
    gate.failed = ActionGateStep::MODE;
    return gate;
  }

  auto capability = vda5050_core::validation::validate_capability(ctx, actions);
  if (!capability)
  {
    gate.result = std::move(capability);
    gate.failed = ActionGateStep::CAPABILITY;
    return gate;
  }

  auto conflict =
    vda5050_core::validation::validate_action_conflict(ctx, actions);
  if (!conflict)
  {
    gate.result = std::move(conflict);
    gate.failed = ActionGateStep::CONFLICT;
    return gate;
  }

  auto limits =
    vda5050_core::validation::validate_protocol_limits(ctx, actions);
  if (!limits)
  {
    gate.result = std::move(limits);
    gate.failed = ActionGateStep::LIMITS;
  }
  return gate;
}

vda5050_core::errors::ValidationResult InstantActionsPublisher::publish(
  vda5050_core::execution::ProtocolAdapter& adapter,
  const vda5050_core::validation::PreSendContext& ctx,
  const vda5050_core::types::InstantActions& actions)
{
  // Full pre-send is skipped so instant actions still work in degraded states
  // (cancelOrder in error, initPosition before localization).
  auto schema_result =
    vda5050_core::validation::validate_instant_actions_content(actions);
  if (!schema_result)
  {
    return schema_result;
  }

  if (ctx.connection_status != vda5050_core::types::ConnectionState::ONLINE)
  {
    vda5050_core::errors::ValidationResult res;
    res.add_error(vda5050_core::errors::create_error(
      vda5050_core::errors::PreSendValidationError,
      "AGV connection_status is not ONLINE", {}));
    return res;
  }

  auto gate = validate_gate(ctx, actions);
  if (gate.failed != ActionGateStep::NONE)
  {
    return gate.result;
  }

  // Fail rather than drop silently when the broker is down (QoS 0).
  if (!adapter.connected())
  {
    vda5050_core::errors::ValidationResult res;
    res.add_error(vda5050_core::errors::create_error(
      vda5050_core::errors::ValidationError,
      "Broker not connected; instant actions not published", {}));
    return res;
  }

  adapter.publish<vda5050_core::types::InstantActions>(
    actions, static_cast<int>(InstantActionsQos));

  return vda5050_core::errors::ValidationResult{};
}

}  // namespace vda5050_core::master
