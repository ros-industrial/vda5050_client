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

#ifndef VDA5050_CORE__MASTER__ORDER__ACTIVE_ORDER_SNAPSHOT_HPP_
#define VDA5050_CORE__MASTER__ORDER__ACTIVE_ORDER_SNAPSHOT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/node.hpp"

namespace vda5050_core {
namespace master {

/// \brief Snapshot of tracked state; state_* fields mirror the last State.
/// Shared DTO between OrderLifecycleManager and OrderStitcher.
struct ActiveOrderSnapshot
{
  bool has_active = false;
  std::string order_id;
  uint32_t order_update_id = 0;
  std::vector<vda5050_core::types::Node> nodes;  // merged base + horizon
  std::vector<vda5050_core::types::Edge> edges;
  uint32_t last_node_sequence_id = 0;
  uint32_t state_order_update_id = 0;
  std::string state_order_id;
  bool order_complete = false;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ORDER__ACTIVE_ORDER_SNAPSHOT_HPP_
