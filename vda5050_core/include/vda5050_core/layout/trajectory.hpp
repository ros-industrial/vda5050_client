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

#ifndef VDA5050_CORE__LAYOUT__TRAJECTORY_HPP_
#define VDA5050_CORE__LAYOUT__TRAJECTORY_HPP_

#include <optional>
#include <vector>

namespace vda5050_core::layout {

struct ControlPoint
{
  double x = 0.0;  ///< [m]
  double y = 0.0;  ///< [m]
  std::optional<double> weight;

  inline bool operator==(const ControlPoint& other) const
  {
    return this->x == other.x && this->y == other.y &&
           this->weight == other.weight;
  }
  inline bool operator!=(const ControlPoint& other) const
  {
    return !(this->operator==(other));
  }
};

struct Trajectory
{
  std::vector<double> knot_vector;
  std::vector<ControlPoint> control_points;
  std::optional<int> degree;

  inline bool operator==(const Trajectory& other) const
  {
    return this->knot_vector == other.knot_vector &&
           this->control_points == other.control_points &&
           this->degree == other.degree;
  }
  inline bool operator!=(const Trajectory& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__TRAJECTORY_HPP_
