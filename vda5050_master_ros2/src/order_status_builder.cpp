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

#include "vda5050_master_ros2/order_status_builder.hpp"

#include <chrono>
#include <string>

#include "vda5050_core/types/error_level.hpp"
#include "vda5050_interfaces/msg/action_state.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
namespace {

builtin_interfaces::msg::Time now_as_ros_time()
{
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  const auto secs =
    std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(secs.count());
  t.nanosec = static_cast<uint32_t>(nsecs.count());
  return t;
}

bool any_fatal_error(const vda5050_core::types::State& state)
{
  for (const auto& e : state.errors)
  {
    if (e.error_level == vda5050_core::types::ErrorLevel::FATAL) return true;
  }
  return false;
}

uint8_t derive_phase(
  const vda5050_core::master::AGV::OrderStatusBundle& bundle,
  const vda5050_core::types::State* state_or_null)
{
  using OrderStatus = vda5050_master_ros2::msg::OrderStatus;
  if (!bundle.active_order_snapshot.has_active)
  {
    return OrderStatus::PHASE_NO_ORDER;
  }
  if (bundle.active_order_snapshot.order_complete)
  {
    return OrderStatus::PHASE_COMPLETED;
  }
  if (state_or_null != nullptr && any_fatal_error(*state_or_null))
  {
    return OrderStatus::PHASE_FAILED;
  }
  if (state_or_null != nullptr && state_or_null->last_node_sequence_id > 0)
  {
    return OrderStatus::PHASE_RUNNING;
  }
  return OrderStatus::PHASE_ACCEPTED;
}

}  // namespace

vda5050_master_ros2::msg::OrderStatus build_order_status_msg(
  const vda5050_core::master::AGV::OrderStatusBundle& bundle,
  const std::string& manufacturer, const std::string& serial_number,
  const std::string& assignment_id)
{
  using OrderStatus = vda5050_master_ros2::msg::OrderStatus;

  OrderStatus msg;
  msg.manufacturer = manufacturer;
  msg.serial_number = serial_number;
  msg.assignment_id = assignment_id;
  msg.stamp = now_as_ros_time();

  const vda5050_core::types::State* state_ptr =
    bundle.state.has_value() ? &bundle.state.value() : nullptr;

  msg.phase = derive_phase(bundle, state_ptr);

  // Order identification — prefer lifecycle snapshot's order id (master's
  // tracked order); fall back to State's order id when no active order.
  const auto& snap = bundle.active_order_snapshot;
  if (snap.has_active)
  {
    msg.order_id = snap.order_id;
    msg.order_update_id = snap.order_update_id;
  }
  else if (state_ptr != nullptr)
  {
    msg.order_id = state_ptr->order_id;
    msg.order_update_id = state_ptr->order_update_id;
  }

  // Stable base / horizon counts from the merged active order.
  for (const auto& n : snap.nodes)
  {
    if (n.released)
    {
      ++msg.nodes_in_base;
    }
    else
    {
      ++msg.nodes_in_horizon;
    }
  }

  msg.pending_stitch_count = static_cast<uint32_t>(bundle.pending_stitch_count);

  if (state_ptr != nullptr)
  {
    msg.last_node_id = state_ptr->last_node_id;
    msg.last_node_sequence_id = state_ptr->last_node_sequence_id;
    msg.driving = state_ptr->driving;
    msg.paused = state_ptr->paused.value_or(false);
    msg.new_base_request = state_ptr->new_base_request.value_or(false);

    msg.action_states.reserve(state_ptr->action_states.size());
    for (const auto& a : state_ptr->action_states)
    {
      msg.action_states.push_back(internal::to_msg<
                                  vda5050_core::types::ActionState,
                                  vda5050_interfaces::msg::ActionState>(a));
    }

    msg.errors.reserve(state_ptr->errors.size());
    for (const auto& e : state_ptr->errors)
    {
      msg.errors.push_back(
        internal::to_msg<
          vda5050_core::types::Error, vda5050_interfaces::msg::Error>(e));
    }
  }

  return msg;
}

}  // namespace vda5050_master_ros2
