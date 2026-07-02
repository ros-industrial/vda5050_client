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

#ifndef VDA5050_CORE__MASTER__STATE__STATE_EVENT_DETECTOR_HPP_
#define VDA5050_CORE__MASTER__STATE__STATE_EVENT_DETECTOR_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {
namespace master {
namespace event {

/// \brief A node the AGV reports as reached (lastNodeId + lastNodeSequenceId).
struct ReachedNode
{
  std::string node_id;
  uint32_t sequence_id;
};

/// \brief Node just reached: lastNodeId/sequence advanced vs prev; else nullopt.
std::optional<ReachedNode> newly_reached_node(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief Errors in curr but not prev.
std::vector<vda5050_core::types::Error> errors_appeared(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief Errors in prev but no longer in curr.
std::vector<vda5050_core::types::Error> errors_resolved(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief Rising edge: new_base_request false → true (absent = false).
bool new_base_requested(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief curr.operating_mode != prev.operating_mode.
bool mode_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief curr.paused != prev.paused (absent = false).
bool paused_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief curr.driving != prev.driving.
bool driving_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/// \brief curr.loads != prev.loads.
bool loads_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

}  // namespace event
}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__STATE__STATE_EVENT_DETECTOR_HPP_
