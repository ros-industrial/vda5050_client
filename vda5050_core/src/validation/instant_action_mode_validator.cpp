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

#include "vda5050_core/validation/instant_action_mode_validator.hpp"

#include <fmt/format.h>
#include <set>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::validation {

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

errors::ValidationResult validate_instant_action_mode(
  const PreSendContext& ctx, const types::InstantActions& actions)
{
  errors::ValidationResult res;

  // Master control is confirmed only in AUTOMATIC / SEMIAUTOMATIC; any other or
  // unknown mode is treated conservatively — only exempt actions pass below.
  const bool master_in_control =
    ctx.last_state.has_value() &&
    (ctx.last_state->operating_mode == types::OperatingMode::AUTOMATIC ||
     ctx.last_state->operating_mode == types::OperatingMode::SEMIAUTOMATIC);
  if (master_in_control) return res;

  for (const auto& action : actions.actions)
  {
    if (is_mode_exempt_action_type(action.action_type)) continue;

    res.add_error(errors::create_error(
      errors::ModeValidationError,
      fmt::format(
        "action_type '{}' is not on the instant-scope allowlist and the AGV "
        "is not confirmed to be in AUTOMATIC / SEMIAUTOMATIC operating_mode "
        "(master must not send driving orders or non-recovery actions in "
        "MANUAL / SERVICE / TEACHIN, or when the AGV's mode is unknown)",
        action.action_type),
      {{errors::RefActionId, action.action_id}}));
  }

  return res;
}

}  // namespace vda5050_core::validation
