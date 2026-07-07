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

#ifndef VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTION_ASSIGNMENT_RESULT_HPP_
#define VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTION_ASSIGNMENT_RESULT_HPP_

#include <vector>

#include "vda5050_core/types/error.hpp"

namespace vda5050_core {
namespace master {

// Synchronous, caller-visible outcome of dispatching instantActions to an AGV.
// Separate from AssignmentResult because each API grows distinct decisions.

/// \brief Outcome category returned by
///        `VDA5050Master::assign_instant_actions`.
enum class InstantActionDecision
{
  /// Pre-flight checks passed; actions queued for publish.
  ASSIGNED,
  /// Master has no AGV with this manufacturer/serial.
  AGV_NOT_ONBOARDED,
  /// AGV connection_state != ONLINE. Sending at QoS 0 to an offline AGV
  /// is a silent drop, so we reject pre-send.
  AGV_OFFLINE,
  /// A candidate action_id is empty, duplicated in the batch, or collides with
  /// an in-flight (state.action_states) or active-order action_id. Must be
  /// globally unique (UUID suggested).
  DUPLICATE_ACTION_ID,
  /// The AGV's outbound queue is full (connection is up, unlike AGV_OFFLINE).
  AGV_QUEUE_FULL,
  /// HARD-blocking candidate while the AGV has an active action
  /// (WAITING/INITIALIZING/RUNNING/PAUSED) — must not run in parallel.
  HARD_ACTION_BLOCKED,
  /// SOFT/HARD-blocking candidate while the AGV is driving — must not drive.
  ACTION_BLOCKED_BY_DRIVING,
  /// AGV not in AUTOMATIC/SEMIAUTOMATIC and the action_type isn't on the
  /// instant-scope allowlist (stateRequest, cancelOrder, initPosition, etc.).
  AGV_MODE_NOT_AUTO_FOR_ACTION
};

/// \brief Structured outcome of `VDA5050Master::assign_instant_actions`.
struct InstantActionAssignmentResult
{
  InstantActionDecision decision = InstantActionDecision::ASSIGNED;

  /// Diagnostic errors, populated for non-ASSIGNED outcomes. Empty on
  /// ASSIGNED.
  std::vector<vda5050_core::types::Error> errors;

  /// True iff `decision == ASSIGNED`.
  explicit operator bool() const
  {
    return decision == InstantActionDecision::ASSIGNED;
  }
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTION_ASSIGNMENT_RESULT_HPP_
