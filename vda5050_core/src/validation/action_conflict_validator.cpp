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

#include "vda5050_core/master/validation/action_conflict_validator.hpp"

#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::master {

namespace {

using ::vda5050_core::errors::ActionBlockedByDrivingError;
using ::vda5050_core::errors::create_error;
using ::vda5050_core::errors::HardActionBlockedError;
using ::vda5050_core::errors::RefActionId;
using ::vda5050_core::order_utils::ValidationResult;
using ::vda5050_core::types::ActionStatus;
using ::vda5050_core::types::BlockingType;
using ::vda5050_core::types::ErrorReference;

// Active statuses: the AGV is committed to the action. PAUSED is included
// (mid-pause HARD could collide on resume); FINISHED / FAILED are not.
constexpr bool is_active_status(ActionStatus s)
{
  return s == ActionStatus::WAITING || s == ActionStatus::INITIALIZING ||
         s == ActionStatus::RUNNING || s == ActionStatus::PAUSED;
}

}  // namespace

ValidationResult validate_action_conflict(
  const PreSendContext& ctx, const vda5050_core::types::InstantActions& actions)
{
  ValidationResult res;

  auto add_error = [&](
                     const std::string& error_type,
                     const std::string& description,
                     std::vector<ErrorReference> refs) {
    res.errors.push_back(create_error(error_type, description, refs));
  };

  // Best-effort pre-screen: this check is state-driven (it reads `driving` and
  // the running `action_states`). With no reported state there is no input to
  // screen against, so the conflict cannot be checked here. This is a known
  // limitation, not a guarantee — the AGV is the runtime authority and enforces
  // non-conflict itself (every received action gets an actionState; parallel
  // execution is governed by blockingType). Rejecting on no-state instead would
  // block recovery actions (cancelOrder, stateRequest) on a silent AGV, which
  // is worse. This complements the instant-action mode gate, which already
  // rejects non-exempt actions when the mode is unknown — so this pass-through
  // cannot admit a non-recovery action on its own.
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
      case BlockingType::NONE:
        break;
      case BlockingType::SOFT:
        if (driving)
        {
          add_error(
            ActionBlockedByDrivingError,
            "SOFT-blocking action '" + action.action_type +
              "' rejected because AGV is driving (vehicle must not drive).",
            {{RefActionId, action.action_id}});
          return res;
        }
        break;
      case BlockingType::HARD:
        if (driving)
        {
          add_error(
            ActionBlockedByDrivingError,
            "HARD-blocking action '" + action.action_type +
              "' rejected because AGV is driving (vehicle must not drive).",
            {{RefActionId, action.action_id}});
          return res;
        }
        if (any_active)
        {
          add_error(
            HardActionBlockedError,
            "HARD-blocking action '" + action.action_type +
              "' rejected because AGV has active actions in flight (must not "
              "be executed in parallel).",
            {{RefActionId, action.action_id}});
          return res;
        }
        break;
    }
  }

  return res;
}

}  // namespace vda5050_core::master
