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

#ifndef VDA5050_MASTER_ROS2__ORDER_STATUS_BUILDER_HPP_
#define VDA5050_MASTER_ROS2__ORDER_STATUS_BUILDER_HPP_

#include <string>

#include "vda5050_core/master/agv.hpp"
#include "vda5050_master_ros2/msg/order_status.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// build_order_status_msg — pure transformation from AGV bundle → ROS msg.
// =============================================================================
//
// Free function used by OrderStatusPublisher (called from on_state with a
// freshly-cached State). No rclcpp dependency, no I/O, no locking — caller
// has already taken the atomic bundle via AGV::get_order_status_bundle().
//
// **Phase derivation** (master's view, sequential precedence):
//
//   !bundle.active_order_snapshot.has_active   → PHASE_NO_ORDER
//   has_active && order_complete               → PHASE_COMPLETED
//   has_active && state has any FATAL error    → PHASE_FAILED
//   has_active && state.last_node_sequence_id>0→ PHASE_RUNNING
//   else                                       → PHASE_ACCEPTED
//
// **Empty-state behaviour**: if `bundle.state` is `std::nullopt`, fields
// echoed from State are zero-initialised (empty strings, false bools, 0
// ints). Phase is whatever the lifecycle snapshot dictates — typically
// PHASE_NO_ORDER but PHASE_ACCEPTED is reachable if a publish recorded
// an active order before any State arrived.
//
// **Stamp**: set to `std::chrono::system_clock::now()` at the call site.

/// \brief Build an OrderStatus message from an AGV's order-status bundle.
///
/// \param assignment_id   Correlation UUID echoed into the message's
///                        assignment_id field. Empty when the master
///                        has no active assignment for this AGV — e.g.
///                        PHASE_NO_ORDER, or the order arrived via the
///                        synchronous AssignOrder service (not
///                        correlation-aware).
vda5050_master_ros2::msg::OrderStatus build_order_status_msg(
  const vda5050_core::master::AGV::OrderStatusBundle& bundle,
  const std::string& manufacturer, const std::string& serial_number,
  const std::string& assignment_id = "");

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ORDER_STATUS_BUILDER_HPP_
