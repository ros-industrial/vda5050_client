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

namespace vda5050_core {
namespace master {

// =============================================================================
// OrderStitcher — 4-condition stitch guard.
// =============================================================================
//
// Routes an outbound Order before the publisher chain: SEND_NOW, QUEUE_PENDING
// (retried on each State via OrderLifecycleManager), or REJECT. Enforces the
// queue-and-retry guards only, not structural/spec validation (the
// publisher chain and is_valid_update() handle those). Stateless, thread-safe.

/// \brief Routing decision for an outbound Order.
enum class StitchDecision
{
  SEND_NOW,
  QUEUE_PENDING,
  REJECT
};

/// \brief Identifies which of the 4 stitch guards rejected the candidate.
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

/// \brief Result of OrderStitcher::decide. On QUEUE_PENDING/REJECT, `errors`
///        carries OrderUpdateError entries; `first_failed_guard` names the
///        triggering stitch guard (NONE for SEND_NOW/REJECT).
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
  ///        active-order context (snapshot carries the needed State fields).
  /// \param candidate  Outbound Order from the FMS / fleet logic.
  /// \param snapshot   Stable, by-value view from
  ///                   OrderLifecycleManager::snapshot().
  /// \return StitchResult.
  StitchResult decide(
    const vda5050_core::types::Order& candidate,
    const ActiveOrderSnapshot& snapshot) const;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_STITCHER_HPP_
