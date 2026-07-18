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
// Separate from OrderAssignmentResult because each API grows distinct
// decisions.

/// \brief Outcome category returned by
///        `VDA5050Master::assign_instant_actions`.
enum class InstantActionDecision
{
  /// Pre-flight checks passed; actions queued for publish.
  ASSIGNED,
  /// Master has no AGV with this manufacturer/serial.
  AGV_NOT_ONBOARDED,
  /// AGV offline; a QoS-0 send would be silently dropped, so reject pre-send.
  AGV_OFFLINE,
  /// Candidate action_id duplicated in the batch or colliding with an
  /// in-flight, active-order, or queued id. Empty id is INVALID_CONTENT.
  DUPLICATE_ACTION_ID,
  /// The AGV's outbound queue is full (connection is up, unlike AGV_OFFLINE).
  AGV_QUEUE_FULL,
  /// HARD-blocking candidate while the AGV already has an active action.
  HARD_ACTION_BLOCKED,
  /// SOFT/HARD-blocking candidate while the AGV is driving.
  ACTION_BLOCKED_BY_DRIVING,
  /// AGV not in an automatic mode and the action_type isn't instant-allowed.
  AGV_MODE_NOT_AUTO_FOR_ACTION,
  /// Batch failed schema validation, or is empty.
  INVALID_CONTENT,
  /// An action_type is not in the AGV's factsheet capabilities.
  AGV_CANNOT_PERFORM_ACTION,
  /// Batch exceeds an array size the AGV declared in its factsheet.
  EXCEEDS_PROTOCOL_LIMITS
};

/// \brief Structured outcome of `VDA5050Master::assign_instant_actions`.
struct InstantActionAssignmentResult
{
  InstantActionDecision decision = InstantActionDecision::ASSIGNED;

  /// Diagnostics for non-ASSIGNED outcomes; empty on ASSIGNED.
  std::vector<vda5050_core::types::Error> errors;

  explicit operator bool() const
  {
    return decision == InstantActionDecision::ASSIGNED;
  }
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTION_ASSIGNMENT_RESULT_HPP_
