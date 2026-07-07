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

namespace vda5050_core::master {

// =============================================================================
// InstantActionAssignmentResult / InstantActionDecision.
// =============================================================================
//
// Returned by VDA5050Master::assign_instant_actions. Synchronous,
// caller-visible outcome of an FMS attempt to dispatch instantActions to a
// specific AGV. Mirrors the AssignmentResult / AssignmentDecision shape from
// #15 but is intentionally a separate type:
//   - The order side carries stitch-specific decisions (STITCH_REJECTED /
//     STITCH_QUEUED) that have no instant-actions analog.
//   - The instant side will gain conflict-specific decisions in #22
//     (CONFLICTS_WITH_ACTIVE_ORDER, etc.) that don't apply to orders.
// Keeping the enums separate avoids polluting either API as both grow.
//
// On ASSIGNED, the actions have been handed off to the AGV's outbound queue;
// the validator chain (schema, PreSend, traversability
// capability) still runs asynchronously on the queue-processor thread as
// defense-in-depth.

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
  /// One or more candidate action_ids collide:
  ///   - within the candidate batch, or
  ///   - with an in-flight action reported in state.action_states[], or
  ///   - with an action_id used in the active order's nodes/edges.
  /// Empty action_id is also reported here.
  /// action_id must be globally unique (UUID suggested).
  DUPLICATE_ACTION_ID,
  /// AGV's outbound queue is full and could not accept the actions.
  /// Distinct from AGV_OFFLINE — connection is up, but the master is
  /// dispatching faster than the AGV is processing.
  AGV_QUEUE_FULL,
  /// HARD-blocking candidate while AGV has at least one action with
  /// status WAITING / INITIALIZING / RUNNING / PAUSED in
  /// `state.action_states[]`. The spec: "must not be executed in parallel".
  HARD_ACTION_BLOCKED,
  /// SOFT- or HARD-blocking candidate while AGV is driving
  /// (state.driving == true). The spec: "Vehicle must not drive."
  ACTION_BLOCKED_BY_DRIVING,
  /// AGV is not in AUTOMATIC / SEMIAUTOMATIC mode AND the candidate
  /// action_type is not on the instant-scope allowlist
  /// (stateRequest, factsheetRequest, logReport, cancelOrder,
  /// initPosition, startPause, stopPause, startCharging, stopCharging).
  /// The spec: master must not send driving orders or non-recovery
  /// actions in MANUAL / SERVICE / TEACHIN.
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

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTION_ASSIGNMENT_RESULT_HPP_
