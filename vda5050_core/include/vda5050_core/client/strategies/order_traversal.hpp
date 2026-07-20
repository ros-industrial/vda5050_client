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

#ifndef VDA5050_CORE__CLIENT__STRATEGIES__ORDER_TRAVERSAL_HPP_
#define VDA5050_CORE__CLIENT__STRATEGIES__ORDER_TRAVERSAL_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"

namespace vda5050_core {

namespace client {

struct NodeReachedUpdate;

/// \brief Moves through the released part of an accepted order.
///
/// Applies new `NodeReachedUpdate`s to the order state, then sends a
/// `NavigateToNodeEvent` for the next released node. It stops at decision
/// points and does not move into the horizon.
///
/// This runs every step, so the AGV can continue after a base extension.
/// Cached updates and repeated steps are handled safely.
class OrderTraversal : public execution::StrategyInterface
{
public:
  OrderTraversal();

  void init(std::shared_ptr<execution::ContextInterface> context) override;

  void step(std::shared_ptr<execution::ContextInterface> context) override;

private:
  /// \brief Last node-reached update that was already used.
  std::shared_ptr<NodeReachedUpdate> last_processed_;

  /// \brief Last node sequence ID that was already dispatched.
  std::optional<uint32_t> last_dispatched_seq_;

  /// \brief Order ID for the last dispatched node.
  std::string last_dispatched_order_id_;

  /// \brief Cached search position for the next node in node_states.
  std::size_t next_node_index_ = 0;

  /// \brief Order ID associated with next_node_index_.
  std::string next_node_order_id_;

  /// \brief Order update ID associated with next_node_index_.
  uint32_t next_node_order_update_id_ = 0;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__STRATEGIES__ORDER_TRAVERSAL_HPP_
