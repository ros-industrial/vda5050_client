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

#include <cmath>

#include "vda5050_core/client/adapter/transformation.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
Transformation::Transformation(Pose2D calibration)
: calibration_point_(calibration)
{
  // Nothing to do here ...
}

//=============================================================================
Transformation::Pose2D Transformation::transform_to_world(Pose2D local)
{
  return Pose2D{
    local.x * std::cos(local.theta) - local.y * std::sin(local.theta) +
      calibration_point_.x,

    local.x * std::sin(local.theta) + local.y * std::cos(local.theta) +
      calibration_point_.y,

    local.theta + calibration_point_.theta};
}

//=============================================================================
Transformation::Pose2D Transformation::transform_to_local(Pose2D world) {}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
