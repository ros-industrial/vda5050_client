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

#include "vda5050_core/master/validation/instant_action_mode_validator.hpp"

#include <set>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::master {

namespace {

// Predefined instant-scope actions exempt from the mode gate.
const std::set<std::string>& exempt_action_types()
{
  static const std::set<std::string> kExempt = {
    "stateRequest", "factsheetRequest", "logReport",
    "cancelOrder",  "initPosition",     "startPause",
    "stopPause",    "startCharging",    "stopCharging"};
  return kExempt;
}

}  // namespace

bool is_mode_exempt_action_type(const std::string& action_type)
{
  return exempt_action_types().count(action_type) != 0;
}

vda5050_core::order_utils::ValidationResult validate_instant_action_mode(
  const PreSendContext& ctx, const vda5050_core::types::InstantActions& actions)
{
  vda5050_core::order_utils::ValidationResult res;

  // No reported state => unknown mode; pass through.
  if (!ctx.last_state.has_value()) return res;

  const auto mode = ctx.last_state->operating_mode;

  if (
    mode == vda5050_core::types::OperatingMode::AUTOMATIC ||
    mode == vda5050_core::types::OperatingMode::SEMIAUTOMATIC)
  {
    return res;
  }

  for (const auto& action : actions.actions)
  {
    if (is_mode_exempt_action_type(action.action_type)) continue;

    res.errors.push_back(vda5050_core::errors::create_error(
      vda5050_core::errors::ModeValidationError,
      "action_type '" + action.action_type +
        "' is not on the instant-scope allowlist and AGV is not in "
        "AUTOMATIC / SEMIAUTOMATIC operating_mode (master must "
        "not send driving orders or non-recovery actions in MANUAL / "
        "SERVICE / TEACHIN)",
      {{vda5050_core::errors::RefActionId, action.action_id}}));
    return res;  // short-circuit on first failure
  }

  return res;
}

}  // namespace vda5050_core::master
