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

#ifndef VDA5050_CORE__CLIENT__RESOURCES__ORDER_EXECUTION_HPP_
#define VDA5050_CORE__CLIENT__RESOURCES__ORDER_EXECUTION_HPP_

#include <mutex>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {

namespace client {

/// \brief Persistent, mutable AGV execution state shared across strategies.
class OrderExecutionResource
: public execution::Initialize<OrderExecutionResource, execution::ResourceBase>
{
public:
  /// \brief Whether the AGV is actively executing an order.
  bool is_executing_order() const;

  /// \brief Set whether the AGV is actively executing an order.
  void set_executing_order(bool executing);

  /// \brief Whether the AGV is waiting for a base extension from the master.
  bool is_awaiting_order_update() const;

  /// \brief Set whether the AGV is waiting for a base extension.
  void set_awaiting_order_update(bool awaiting);

  /// \brief Return a copy of the full working state snapshot.
  types::State get_state() const;

  /// \brief Replace the full working state snapshot.
  void set_state(types::State state);

private:
  /// \brief Serialises concurrent reads and writes to the members below.
  mutable std::mutex mutex_;

  /// \brief True while the AGV is actively executing an order.
  bool executing_order_ = false;

  /// \brief True when AGV is waiting for base extension from master.
  bool awaiting_order_update_ = false;

  /// \brief Working VDA5050 state snapshot.
  ///
  /// Includes:
  /// - order_id, order_update_id
  /// - last_node_id, last_node_sequence_id
  /// - node_states, edge_states, action_states
  types::State state_;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__RESOURCES__ORDER_EXECUTION_HPP_
