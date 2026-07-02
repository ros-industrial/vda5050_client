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

#ifndef VDA5050_CORE__LAYOUT__EDGE_HPP_
#define VDA5050_CORE__LAYOUT__EDGE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/layout/vehicle_type_edge_property.hpp"

namespace vda5050_core::layout {

struct Edge
{
  std::string edge_id;
  std::string start_node_id;  ///< must be in the current layout
  std::string end_node_id;    ///< may be in another layout (layout transition)
  std::vector<VehicleTypeEdgeProperty>
    vehicle_type_edge_properties;  ///< non-empty
  std::optional<std::string> edge_name;
  std::optional<std::string> edge_description;

  inline bool operator==(const Edge& other) const
  {
    return this->edge_id == other.edge_id &&
           this->start_node_id == other.start_node_id &&
           this->end_node_id == other.end_node_id &&
           this->vehicle_type_edge_properties ==
             other.vehicle_type_edge_properties &&
           this->edge_name == other.edge_name &&
           this->edge_description == other.edge_description;
  }
  inline bool operator!=(const Edge& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__EDGE_HPP_
