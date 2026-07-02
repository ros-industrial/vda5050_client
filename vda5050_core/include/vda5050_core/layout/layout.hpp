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

#ifndef VDA5050_CORE__LAYOUT__LAYOUT_HPP_
#define VDA5050_CORE__LAYOUT__LAYOUT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/layout/edge.hpp"
#include "vda5050_core/layout/node.hpp"
#include "vda5050_core/layout/station.hpp"

namespace vda5050_core::layout {

struct Layout
{
  std::string layout_id;
  std::string layout_version;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  std::vector<Station> stations;
  std::optional<std::string> layout_name;
  std::optional<std::string> layout_level_id;
  std::optional<std::string> layout_description;

  inline bool operator==(const Layout& other) const
  {
    return this->layout_id == other.layout_id &&
           this->layout_version == other.layout_version &&
           this->nodes == other.nodes && this->edges == other.edges &&
           this->stations == other.stations &&
           this->layout_name == other.layout_name &&
           this->layout_level_id == other.layout_level_id &&
           this->layout_description == other.layout_description;
  }
  inline bool operator!=(const Layout& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__LAYOUT_HPP_
