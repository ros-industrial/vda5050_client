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

#ifndef VDA5050_CORE__LAYOUT__STATION_HPP_
#define VDA5050_CORE__LAYOUT__STATION_HPP_

#include <optional>
#include <string>
#include <vector>

namespace vda5050_core::layout {

struct StationPosition
{
  double x = 0.0;               ///< [m]
  double y = 0.0;               ///< [m]
  std::optional<double> theta;  ///< [rad], range [-Pi..Pi]

  inline bool operator==(const StationPosition& other) const
  {
    return this->x == other.x && this->y == other.y &&
           this->theta == other.theta;
  }
  inline bool operator!=(const StationPosition& other) const
  {
    return !(this->operator==(other));
  }
};

struct Station
{
  std::string station_id;
  std::vector<std::string> interaction_node_ids;  ///< non-empty
  std::optional<std::string> station_name;
  std::optional<std::string> station_description;
  std::optional<double> station_height;  ///< [m], default 0
  std::optional<StationPosition> station_position;

  inline bool operator==(const Station& other) const
  {
    return this->station_id == other.station_id &&
           this->interaction_node_ids == other.interaction_node_ids &&
           this->station_name == other.station_name &&
           this->station_description == other.station_description &&
           this->station_height == other.station_height &&
           this->station_position == other.station_position;
  }
  inline bool operator!=(const Station& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__STATION_HPP_
