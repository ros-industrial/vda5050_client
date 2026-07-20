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

#ifndef VDA5050_CORE__CLIENT__EVENTS__EDGE_LEFT_HPP_
#define VDA5050_CORE__CLIENT__EVENTS__EDGE_LEFT_HPP_

#include <cstdint>
#include <string>
#include <utility>

#include "vda5050_core/execution/base.hpp"

namespace vda5050_core {

namespace client {

/// \brief Emitted when the AGV finishes traversing an edge.
///
/// The action strategy observes this event to detect that traversal of the
/// matching edge has ended. The edge is identified by its ID and sequence ID.
/// This event is typically emitted when the AGV reaches the edges's end node.
struct EdgeLeftEvent
: public execution::Initialize<EdgeLeftEvent, execution::EventBase>
{
  std::string edge_id;
  uint32_t sequence_id = 0;

  EdgeLeftEvent(std::string edge_id, uint32_t sequence_id)
  : edge_id(std::move(edge_id)), sequence_id(sequence_id)
  {
  }
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__EVENTS__EDGE_LEFT_HPP_
