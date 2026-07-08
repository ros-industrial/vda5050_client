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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__TRANSFORMATION_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__TRANSFORMATION_HPP_

namespace vda5050_core {

namespace client {

namespace adapter {

struct Pose2D
{
  double x;
  double y;
  double theta;
};

class Transformation
{
public:
  static Transformation calibrate(
    const Pose2D& world_pose, const Pose2D& agv_pose);

  Pose2D to_world_pose(const Pose2D& agv_pose) const;

  Pose2D to_agv_pose(const Pose2D& world_pose) const;

private:
  explicit Transformation(const Pose2D& world_to_agv);

  static Pose2D transform_pose(const Pose2D& tf, const Pose2D& pose);

  static Pose2D inverse(const Pose2D& pose);

  static double normalize_angle(double angle);

  Pose2D world_to_agv_;
  Pose2D agv_to_world_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__TRANSFORMATION_HPP_
