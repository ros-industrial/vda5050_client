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

#include "vda5050_core/client/adapter/node_request.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
const std::string& NodeRequest::node_id() const
{
  return node_id_;
}

//=============================================================================
uint32_t NodeRequest::sequence_id() const
{
  return sequence_id_;
}

//=============================================================================
const std::optional<types::NodePosition>& NodeRequest::node_position() const
{
  return node_position_;
}

//=============================================================================
const std::optional<std::string>& NodeRequest::node_description() const
{
  return node_description_;
}

//=============================================================================
NodeRequest NodeRequest::from_node(const types::Node& node)
{
  return NodeRequest(
    node.node_id, node.sequence_id, node.node_position, node.node_description);
}

//=============================================================================
NodeRequest::NodeRequest(
  const std::string& node_id, uint32_t sequence_id,
  std::optional<types::NodePosition> node_position,
  std::optional<std::string> node_description)
: node_id_(node_id),
  sequence_id_(sequence_id),
  node_position_(std::move(node_position)),
  node_description_(std::move(node_description))
{
  // Nothing to do here ...
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
