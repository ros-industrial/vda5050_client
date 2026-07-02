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

#ifndef VDA5050_CORE__MASTER__ASSIGNMENT_RESULT_HPP_
#define VDA5050_CORE__MASTER__ASSIGNMENT_RESULT_HPP_

#include <vector>

#include "vda5050_core/types/error.hpp"

namespace vda5050_core::master {

// =============================================================================
// AssignmentResult / AssignmentDecision.
// =============================================================================
//
// Returned by VDA5050Master::assign_order. Synchronous, caller-visible
// outcome of an FMS attempt to assign an outbound Order to a specific AGV.
// Implements VM-VDA-6-6-3-1 #1 ("Master must check that the vehicle is
// in a state to receive a new order") at the public API level.
//
// On ASSIGNED / STITCH_QUEUED, the order has been handed off to the AGV's
// outbound queue; the validator chain (schema, PreSend, graph,
// #19 update path, traversability) still runs asynchronously on the
// queue-processor thread as defense-in-depth.

/// \brief Outcome category returned by `VDA5050Master::assign_order`.
enum class AssignmentDecision
{
  /// Pre-flight checks passed; order queued for publish.
  ASSIGNED,
  /// Master has no AGV with this manufacturer/serial.
  AGV_NOT_ONBOARDED,
  /// AGV connection_state != ONLINE.
  AGV_OFFLINE,
  /// Operational state ERROR / UNAVAILABLE / STATE_UNKNOWN.
  AGV_NOT_READY,
  /// Operating mode != AUTOMATIC (master control requires AUTOMATIC).
  AGV_MODE_NOT_AUTO,
  /// AGV reports position not initialized (VM-VDA-6-6-1-3 #7).
  AGV_POSITION_NOT_INITIALIZED,
  /// AGV has not yet reported any State message.
  AGV_NO_STATE_YET,
  /// Order is an update for an active order, but stitcher rejects
  /// (e.g., backward order_update_id, stitch node mismatch).
  STITCH_REJECTED,
  /// Order is an update for an active order; stitcher decided
  /// QUEUE_PENDING (AGV not yet at stitch point). Order has been
  /// enqueued in lifecycle's pending_updates; will be drained when
  /// AGV state reaches the stitch point.
  STITCH_QUEUED
};

/// \brief Structured outcome of `VDA5050Master::assign_order`.
struct AssignmentResult
{
  AssignmentDecision decision = AssignmentDecision::ASSIGNED;

  /// Diagnostic errors, populated for non-ASSIGNED outcomes. Empty on
  /// ASSIGNED. STITCH_QUEUED is treated as a success path (queue
  /// accepted) and carries no errors; FMS observes via
  /// `agv->pending_update_count()`.
  std::vector<vda5050_core::types::Error> errors;

  /// True iff `decision == ASSIGNED`. STITCH_QUEUED returns false here
  /// because the order has not yet been published — caller may want
  /// to differentiate; use `decision` for fine-grained logic.
  explicit operator bool() const
  {
    return decision == AssignmentDecision::ASSIGNED;
  }
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__ASSIGNMENT_RESULT_HPP_
