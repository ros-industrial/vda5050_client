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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__NODE_REQUEST_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__NODE_REQUEST_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/node_position.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

class NodeRequest
{
public:
  const std::string& node_id() const;

  uint32_t sequence_id() const;

  const std::optional<types::NodePosition>& node_position() const;

  const std::optional<std::string>& node_description() const;

private:
  friend class Adapter;

  static NodeRequest from_node(const types::Node& node);

  NodeRequest(
    const std::string& node_id, uint32_t sequence_id,
    std::optional<types::NodePosition> node_position = std::nullopt,
    std::optional<std::string> node_description = std::nullopt);

  std::string node_id_;
  uint32_t sequence_id_;

  std::optional<types::NodePosition> node_position_;

  std::optional<std::string> node_description_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__NODE_REQUEST_HPP_
