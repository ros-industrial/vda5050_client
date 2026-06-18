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

#include "vda5050_core/master/state/state_event_detector.hpp"

#include <algorithm>

namespace vda5050_core {
namespace master {
namespace event {

namespace {

bool same_error(
  const vda5050_core::types::Error& a, const vda5050_core::types::Error& b)
{
  return a.error_type == b.error_type &&
         a.error_references == b.error_references &&
         a.error_description == b.error_description;
}

// Errors in `from` absent from `against` (identity: type + references +
// description).
std::vector<vda5050_core::types::Error> errors_diff(
  const std::vector<vda5050_core::types::Error>& from,
  const std::vector<vda5050_core::types::Error>& against)
{
  std::vector<vda5050_core::types::Error> diff;
  for (const auto& e : from)
  {
    auto it = std::find_if(
      against.begin(), against.end(),
      [&e](const vda5050_core::types::Error& a) { return same_error(e, a); });
    if (it == against.end()) diff.push_back(e);
  }
  return diff;
}

}  // namespace

std::optional<ReachedNode> newly_reached_node(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  if (curr.last_node_id.empty()) return std::nullopt;
  if (
    curr.last_node_id == prev.last_node_id &&
    curr.last_node_sequence_id == prev.last_node_sequence_id)
  {
    return std::nullopt;
  }
  return ReachedNode{curr.last_node_id, curr.last_node_sequence_id};
}

std::vector<vda5050_core::types::Error> errors_appeared(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return errors_diff(curr.errors, prev.errors);
}

std::vector<vda5050_core::types::Error> errors_resolved(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return errors_diff(prev.errors, curr.errors);
}

bool new_base_requested(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  bool prev_set = prev.new_base_request.value_or(false);
  bool curr_set = curr.new_base_request.value_or(false);
  return curr_set && !prev_set;
}

bool mode_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.operating_mode != curr.operating_mode;
}

bool paused_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.paused.value_or(false) != curr.paused.value_or(false);
}

bool driving_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.driving != curr.driving;
}

bool loads_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.loads != curr.loads;
}

}  // namespace event
}  // namespace master
}  // namespace vda5050_core
