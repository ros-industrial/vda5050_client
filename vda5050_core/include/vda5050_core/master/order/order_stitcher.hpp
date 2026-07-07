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

#ifndef VDA5050_CORE__MASTER__ORDER__ORDER_STITCHER_HPP_
#define VDA5050_CORE__MASTER__ORDER__ORDER_STITCHER_HPP_

#include <vector>

#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core::master {

// =============================================================================
// OrderStitcher — 4-condition stitch guard.
// =============================================================================
//
// Routes outbound Orders at AGV::publish_order BEFORE the publisher
// validator chain. Decides one of:
//
//   - SEND_NOW       — forward to OrderPublisher chain immediately
//   - QUEUE_PENDING  — hand to OrderLifecycleManager::enqueue_pending_update;
//                      will be retried on each State message
//   - REJECT         — log + drop (orderUpdateError)
//
// **What this enforces**: 4 FIWARE operational guards (queue-and-retry
// pattern) — these are NOT spec-mandated, but match the canonical FIWARE
// Mirrors the FIWARE reference master. The spec mandates only what
// `vda5050_core::is_valid_update()` already enforces (base
// immutability, monotonic update_id, stitch byte-equality).
//
// **What this does NOT enforce**: structural validation. The publisher
// chain handles schema, graph, and traversability.
// `is_valid_update()` runs separately inside the lifecycle manager's
// drain (when a queued update is finally combined).
//
// Stateless — does not own a mutex. Thread-safe by construction (inputs
// are stack-local: const-ref candidate, by-value snapshot).

/// \brief Routing decision for an outbound Order.
enum class StitchDecision
{
  SEND_NOW,
  QUEUE_PENDING,
  REJECT
};

/// \brief Identifies which of the 4 FIWARE guards rejected the candidate.
/// NONE for SEND_NOW or for REJECT (which is a structural / spec failure,
/// not a guard failure — see below).
enum class GuardFailure
{
  NONE,
  ORDER_ID_MISMATCH,         // cond 1: state.order_id != active.order_id
  STITCH_PASSED,             // cond 2: AGV is past the stitch point
  STITCH_NOT_REACHED,        // cond 3: AGV hasn't arrived at the stitch
  PREV_UPDATE_NOT_CONFIRMED  // cond 4: AGV still on prior order_update_id
};

/// \brief Result of OrderStitcher::decide.
///
/// On QUEUE_PENDING / REJECT, `errors` carries one or more
/// `OrderUpdateError`-typed Errors (mirrors ValidationResult).
/// `first_failed_guard` identifies the first FIWARE guard that triggered
/// the QUEUE_PENDING decision; NONE for SEND_NOW or REJECT.
struct StitchResult
{
  StitchDecision decision = StitchDecision::SEND_NOW;
  std::vector<vda5050_core::types::Error> errors;
  GuardFailure first_failed_guard = GuardFailure::NONE;

  explicit operator bool() const
  {
    return decision == StitchDecision::SEND_NOW;
  }
};

class OrderStitcher
{
public:
  OrderStitcher() = default;
  ~OrderStitcher() = default;
  OrderStitcher(const OrderStitcher&) = default;
  OrderStitcher& operator=(const OrderStitcher&) = default;

  /// \brief Decide what to do with `candidate` given the AGV's tracked
  ///        active-order context.
  ///
  /// Inputs are snapshot-only: the lifecycle manager's snapshot already
  /// carries `state_order_id`, `last_node_sequence_id`, and
  /// `state_order_update_id` — no separate `State` parameter needed.
  ///
  /// \param candidate  Outbound Order from the FMS / fleet logic.
  /// \param snapshot   Stable, by-value view from
  ///                   OrderLifecycleManager::snapshot().
  /// \return StitchResult.
  StitchResult decide(
    const vda5050_core::types::Order& candidate,
    const ActiveOrderSnapshot& snapshot) const;
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_STITCHER_HPP_
