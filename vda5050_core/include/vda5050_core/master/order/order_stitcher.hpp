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

#include "vda5050_core/master/order/active_order_snapshot.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core {
namespace master {

// Routes an outbound Order via the stitch guards only. Stateless.

/// \brief Routing decision for an outbound Order (IGNORE = duplicate no-op).
enum class StitchDecision
{
  SEND_NOW,
  QUEUE_PENDING,
  IGNORE,
  REJECT
};

/// \brief Which stitch guard queued the candidate (NONE for SEND_NOW/REJECT).
enum class GuardFailure
{
  NONE,
  ORDER_ID_MISMATCH,         // state.order_id != active.order_id
  NO_STATE_YET,              // no State reported; timing not evaluable
  PREV_UPDATE_NOT_CONFIRMED  // AGV still on prior order_update_id
};

/// \brief OrderStitcher::decide result; errors set on QUEUE_PENDING/REJECT.
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

  /// \brief Decide what to do with `candidate` vs the tracked active order.
  /// \param candidate  Outbound Order from the fleet logic.
  /// \param snapshot   By-value view from OrderLifecycleManager::snapshot().
  StitchResult decide(
    const vda5050_core::types::Order& candidate,
    const ActiveOrderSnapshot& snapshot) const;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_STITCHER_HPP_
