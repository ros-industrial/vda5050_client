/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
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

#ifndef VDA5050_CORE__TYPES__TYPE_SPECIFICATION_HPP_
#define VDA5050_CORE__TYPES__TYPE_SPECIFICATION_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/types/agv_class.hpp"
#include "vda5050_core/types/agv_kinematic.hpp"

namespace vda5050_core {

namespace types {

/// \brief This message describes general properties of the AGV type.
struct TypeSpecification
{
  /// \brief Free text generalized series name as specified by manufacturer
  std::string series_name;

  /// \brief Simplified description of AGV kinematics type.
  /// Allowed values: [DIFF, OMNI, THREEWHEEL]
  /// Default-member-initialized so a default-constructed TypeSpecification
  /// has a defined value (otherwise the to_string() trait would observe
  /// indeterminate memory and throw "Invalid AGVKinematic enum value").
  AGVKinematic agv_kinematic{AGVKinematic::DIFF};

  /// \brief Simplified description of AGV class.
  /// Allowed values: [FORKLIFT, CONVEYOR, TUGGER, CARRIER]
  /// Default-member-initialized for the same reason as agv_kinematic.
  AGVClass agv_class{AGVClass::FORKLIFT};

  /// \brief Maximum loadable mass [kg]
  double max_load_mass = 0.0;

  /// \brief Simplified description of localization types.
  /// Example values: NATURAL, REFLECTOR, RFID, DMC, SPOT, GRID
  std::vector<std::string> localization_types;

  /// \brief Path planning types supported by the AGV, sorted by priority.
  /// Example values: PHYSICAL_LINE_GUIDED, VIRTUAL_LINE_GUIDED, AUTONOMOUS
  std::vector<std::string> navigation_types;

  /// \brief Free text human-readable description of the AGV type series
  std::optional<std::string> series_description;

  /// \brief Equality operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is equal?
  inline bool operator==(const TypeSpecification& other) const
  {
    if (this->series_name != other.series_name) return false;
    if (this->agv_kinematic != other.agv_kinematic) return false;
    if (this->agv_class != other.agv_class) return false;
    if (this->max_load_mass != other.max_load_mass) return false;
    if (this->localization_types != other.localization_types) return false;
    if (this->navigation_types != other.navigation_types) return false;
    if (this->series_description != other.series_description) return false;
    return true;
  }

  /// \brief Inequality operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is not equal?
  inline bool operator!=(const TypeSpecification& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace types
}  // namespace vda5050_core

#endif  // VDA5050_CORE__TYPES__TYPE_SPECIFICATION_HPP_
