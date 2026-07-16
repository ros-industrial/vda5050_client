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

#ifndef VDA5050_CORE__MASTER__ORDER__ORDER_ASSIGNMENT_RESULT_HPP_
#define VDA5050_CORE__MASTER__ORDER__ORDER_ASSIGNMENT_RESULT_HPP_

#include <vector>

#include "vda5050_core/types/error.hpp"

namespace vda5050_core {
namespace master {

// Synchronous outcome of assign_order; the queue thread re-checks as defense.

/// \brief Outcome category returned by `VDA5050Master::assign_order`.
enum class OrderAssignmentDecision
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
  /// AGV reports position not initialized.
  AGV_POSITION_NOT_INITIALIZED,
  /// AGV has not yet reported any State message.
  AGV_NO_STATE_YET,
  /// The AGV's outbound queue is full (connection is up, unlike AGV_OFFLINE).
  AGV_QUEUE_FULL,
  /// Update rejected by the stitcher (backward id, stitch mismatch, etc.).
  STITCH_REJECTED,
  /// Update queued (AGV not yet on the order, or prior update unconfirmed);
  /// drained on a later State. Observe via agv->pending_update_count().
  STITCH_QUEUED,
  /// Duplicate order_update_id already applied; nothing published (no-op).
  DUPLICATE_IGNORED
};

/// \brief Structured outcome of `VDA5050Master::assign_order`.
struct OrderAssignmentResult
{
  OrderAssignmentDecision decision = OrderAssignmentDecision::ASSIGNED;

  /// Diagnostics for failure outcomes; empty on ASSIGNED / STITCH_QUEUED.
  std::vector<vda5050_core::types::Error> errors;

  /// True iff ASSIGNED (STITCH_QUEUED is not yet published).
  explicit operator bool() const
  {
    return decision == OrderAssignmentDecision::ASSIGNED;
  }
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_ASSIGNMENT_RESULT_HPP_
