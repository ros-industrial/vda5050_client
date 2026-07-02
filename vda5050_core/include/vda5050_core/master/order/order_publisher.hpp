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

#ifndef VDA5050_CORE__MASTER__ORDER__ORDER_PUBLISHER_HPP_
#define VDA5050_CORE__MASTER__ORDER__ORDER_PUBLISHER_HPP_

#include <optional>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/order_utils/validation_result.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core::master {

// Forward-declared to avoid pulling agv.hpp (transitive cycle) into this
// header; defined in vda5050_core/master/validation/pre_send_validator.hpp.
struct PreSendContext;

/// \brief Runs the outgoing-order validator chain, then publishes.
///
/// Stateless: encapsulates the order QoS and routes the typed payload through
/// the per-AGV ProtocolAdapter. Safe to call concurrently.
class OrderPublisher
{
public:
  OrderPublisher() = default;

  /// \brief Validate and publish an order through the supplied adapter.
  ///
  /// Runs schema → pre-send → structural → traversability, short-circuiting on
  /// the first failure, and publishes only if all pass. When `active_order` is
  /// set and shares the order's id, the candidate is validated as a stitch
  /// update against it; otherwise it is validated as a fresh graph.
  ///
  /// \param adapter       per-AGV typed adapter (caller-owned)
  /// \param ctx           AGV readiness snapshot, built by the caller
  /// \param order         the order to publish
  /// \param active_order  the AGV's current active order, if any
  /// \param merged_out    if non-null and the order stitched onto an active
  ///                      order, receives the full merged order so the caller
  ///                      can adopt it without re-combining
  /// \return validation result; empty errors means published
  vda5050_core::order_utils::ValidationResult publish(
    vda5050_core::execution::ProtocolAdapter& adapter,
    const PreSendContext& ctx, const vda5050_core::types::Order& order,
    const std::optional<vda5050_core::types::Order>& active_order,
    std::optional<vda5050_core::types::Order>* merged_out = nullptr);
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_PUBLISHER_HPP_
