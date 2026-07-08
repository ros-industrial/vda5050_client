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
Transformation Transformation::calibrate(
  const Pose2D& world_pose, const Pose2D& agv_pose)
{
  return Transformation(transform_pose(world_pose, inverse(agv_pose)));
}

//=============================================================================
Pose2D Transformation::to_world_pose(const Pose2D& agv_pose) const
{
  return transform_pose(world_to_agv_, agv_pose);
}

//=============================================================================
Pose2D Transformation::to_agv_pose(const Pose2D& world_pose) const
{
  return transform_pose(agv_to_world_, world_pose);
}

//=============================================================================
Pose2D Transformation::transform_pose(const Pose2D& tf, const Pose2D& pose)
{
  const auto c = std::cos(tf.theta);
  const auto s = std::sin(tf.theta);

  return Pose2D{
    c * pose.x - s * pose.y + tf.x, s * pose.x + c * pose.y + tf.y,
    normalize_angle(tf.theta + pose.theta)};
}

//=============================================================================
Pose2D Transformation::inverse(const Pose2D& pose)
{
  const auto c = std::cos(pose.theta);
  const auto s = std::sin(pose.theta);

  return Pose2D{
    -(c * pose.x + s * pose.y), (s * pose.x - c * pose.y),
    normalize_angle(-pose.theta)};
}

//=============================================================================
double Transformation::normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

//=============================================================================
Transformation::Transformation(const Pose2D& calibration)
: world_to_agv_(calibration), agv_to_world_(inverse(calibration))
{
  // Nothing to do here ...
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
