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

#include "vda5050_core/validation/action_conflict_validator.hpp"

#include <fmt/format.h>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::validation {

namespace {

// Active statuses: the AGV is committed to the action. PAUSED is included
// (mid-pause HARD could collide on resume); FINISHED / FAILED are not.
constexpr bool is_active_status(types::ActionStatus s)
{
  return s == types::ActionStatus::WAITING ||
         s == types::ActionStatus::INITIALIZING ||
         s == types::ActionStatus::RUNNING || s == types::ActionStatus::PAUSED;
}

}  // namespace

errors::ValidationResult validate_action_conflict(
  const PreSendContext& ctx, const types::InstantActions& actions)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& error_type,
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    res.add_error(errors::create_error(error_type, description, refs));
  };

  // No state to screen against — pass through. The AGV enforces non-conflict
  // itself; rejecting here would block recovery actions on a silent AGV.
  if (!ctx.last_state.has_value()) return res;

  const auto& state = ctx.last_state.value();
  const bool driving = state.driving;

  bool any_active = false;
  for (const auto& as : state.action_states)
  {
    if (is_active_status(as.action_status))
    {
      any_active = true;
      break;
    }
  }

  for (const auto& action : actions.actions)
  {
    switch (action.blocking_type)
    {
      case types::BlockingType::NONE:
        break;
      case types::BlockingType::SOFT:
        if (driving)
        {
          add_error(
            errors::ActionBlockedByDrivingError,
            fmt::format(
              "SOFT-blocking action '{}' rejected because AGV is driving "
              "(vehicle must not drive).",
              action.action_type),
            {{errors::RefActionId, action.action_id}});
        }
        break;
      case types::BlockingType::HARD:
        if (driving)
        {
          add_error(
            errors::ActionBlockedByDrivingError,
            fmt::format(
              "HARD-blocking action '{}' rejected because AGV is driving "
              "(vehicle must not drive).",
              action.action_type),
            {{errors::RefActionId, action.action_id}});
          continue;
        }
        if (any_active)
        {
          add_error(
            errors::HardActionBlockedError,
            fmt::format(
              "HARD-blocking action '{}' rejected because AGV has active "
              "actions in flight (must not be executed in parallel).",
              action.action_type),
            {{errors::RefActionId, action.action_id}});
        }
        break;
    }
  }

  return res;
}

}  // namespace vda5050_core::validation
