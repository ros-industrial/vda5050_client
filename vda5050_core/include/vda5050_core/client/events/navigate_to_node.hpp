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

#ifndef VDA5050_CORE__CLIENT__EVENTS__NAVIGATE_TO_NODE_HPP_
#define VDA5050_CORE__CLIENT__EVENTS__NAVIGATE_TO_NODE_HPP_

#include <optional>
#include <utility>

#include "vda5050_core/client/adapter/edge_request.hpp"
#include "vda5050_core/client/adapter/node_request.hpp"
#include "vda5050_core/execution/base.hpp"

namespace vda5050_core {

namespace client {

/// \brief Event requesting navigation to a node in the current order.
///
/// Emitted by the order traversal strategy when AGV should navigate to the
/// next node. The payload carries action-free navigation requests,
/// node and edge actions are dispatched separately.
struct NavigateToNodeEvent
: public execution::Initialize<NavigateToNodeEvent, execution::EventBase>
{
  /// \brief Target node that the AGV should navigate to.
  adapter::NodeRequest target;

  /// \brief Optional edge leading to the target node.
  ///
  /// Empty when target is the first node of order.
  std::optional<adapter::EdgeRequest> via_edge;

  explicit NavigateToNodeEvent(
    adapter::NodeRequest target,
    std::optional<adapter::EdgeRequest> via_edge = std::nullopt)
  : target(std::move(target)), via_edge(std::move(via_edge))
  {
  }
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__EVENTS__NAVIGATE_TO_NODE_HPP_
