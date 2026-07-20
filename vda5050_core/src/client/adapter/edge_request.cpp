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

#include <utility>

#include "vda5050_core/client/adapter/edge_request.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
const std::string& EdgeRequest::edge_id() const
{
  return edge_id_;
}

//=============================================================================
uint32_t EdgeRequest::sequence_id() const
{
  return sequence_id_;
}

//=============================================================================
const std::optional<types::Trajectory>& EdgeRequest::trajectory()
{
  return trajectory_;
}

//=============================================================================
std::optional<double> EdgeRequest::max_speed() const
{
  return max_speed_;
}

//=============================================================================
std::optional<double> EdgeRequest::min_height() const
{
  return min_height_;
}

//=============================================================================
std::optional<double> EdgeRequest::max_height() const
{
  return max_height_;
}

//=============================================================================
std::optional<bool> EdgeRequest::rotation_allowed() const
{
  return rotation_allowed_;
}

//=============================================================================
std::optional<double> EdgeRequest::max_rotation_speed() const
{
  return max_rotation_speed_;
}

//=============================================================================
std::optional<double> EdgeRequest::length() const
{
  return length_;
}

//=============================================================================
EdgeRequest EdgeRequest::from_edge(const types::Edge& edge)
{
  return EdgeRequest(
    edge.edge_id, edge.sequence_id, edge.trajectory, edge.max_speed,
    edge.min_height, edge.max_height, edge.rotation_allowed,
    edge.max_rotation_speed, edge.length);
}

//=============================================================================
EdgeRequest::EdgeRequest(
  const std::string& edge_id, uint32_t sequence_id,
  std::optional<types::Trajectory> trajectory, std::optional<double> max_speed,
  std::optional<double> min_height, std::optional<double> max_height,
  std::optional<bool> rotation_allowed,
  std::optional<double> max_rotation_speed, std::optional<double> length)
: edge_id_(edge_id),
  sequence_id_(sequence_id),
  trajectory_(std::move(trajectory)),
  max_speed_(max_speed),
  min_height_(min_height),
  max_height_(max_height),
  rotation_allowed_(rotation_allowed),
  max_rotation_speed_(max_rotation_speed),
  length_(length)
{
  // Nothing to do here ...
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
