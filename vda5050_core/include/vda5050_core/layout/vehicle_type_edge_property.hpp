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

#ifndef VDA5050_CORE__LAYOUT__VEHICLE_TYPE_EDGE_PROPERTY_HPP_
#define VDA5050_CORE__LAYOUT__VEHICLE_TYPE_EDGE_PROPERTY_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/layout/layout_action.hpp"
#include "vda5050_core/layout/load_restriction.hpp"
#include "vda5050_core/layout/trajectory.hpp"

namespace vda5050_core::layout {

enum class OrientationType
{
  GLOBAL,
  TANGENTIAL,
};

enum class RotationAtNode
{
  NONE,
  CCW,
  CW,
  BOTH,
};

struct VehicleTypeEdgeProperty
{
  std::string vehicle_type_id;
  bool rotation_allowed = false;

  std::optional<double> vehicle_orientation;  ///< [rad]
  std::optional<OrientationType> orientation_type;
  std::optional<RotationAtNode> rotation_at_start_node_allowed;
  std::optional<RotationAtNode> rotation_at_end_node_allowed;
  std::optional<double> max_speed;           ///< [m/s]
  std::optional<double> max_rotation_speed;  ///< [rad/s]
  std::optional<double> min_height;          ///< [m]
  std::optional<double> max_height;          ///< [m]
  std::optional<LoadRestriction> load_restriction;
  std::optional<std::vector<Action>> actions;
  std::optional<Trajectory> trajectory;
  std::optional<bool> reentry_allowed;

  inline bool operator==(const VehicleTypeEdgeProperty& other) const
  {
    return this->vehicle_type_id == other.vehicle_type_id &&
           this->rotation_allowed == other.rotation_allowed &&
           this->vehicle_orientation == other.vehicle_orientation &&
           this->orientation_type == other.orientation_type &&
           this->rotation_at_start_node_allowed ==
             other.rotation_at_start_node_allowed &&
           this->rotation_at_end_node_allowed ==
             other.rotation_at_end_node_allowed &&
           this->max_speed == other.max_speed &&
           this->max_rotation_speed == other.max_rotation_speed &&
           this->min_height == other.min_height &&
           this->max_height == other.max_height &&
           this->load_restriction == other.load_restriction &&
           this->actions == other.actions &&
           this->trajectory == other.trajectory &&
           this->reentry_allowed == other.reentry_allowed;
  }
  inline bool operator!=(const VehicleTypeEdgeProperty& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__VEHICLE_TYPE_EDGE_PROPERTY_HPP_
