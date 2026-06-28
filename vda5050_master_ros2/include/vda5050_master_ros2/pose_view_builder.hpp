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

#ifndef VDA5050_MASTER_ROS2__POSE_VIEW_BUILDER_HPP_
#define VDA5050_MASTER_ROS2__POSE_VIEW_BUILDER_HPP_

#include <string>

#include "vda5050_core/master/pose_view.hpp"
#include "vda5050_master_ros2/msg/pose_view.hpp"

namespace vda5050_master_ros2 {

/// \brief Build a PoseView message from the core PoseView snapshot.
///
/// Pure transformation, no I/O or locking. Used by PoseViewPublisher's
/// fixed-rate timer.
vda5050_master_ros2::msg::PoseView build_pose_view_msg(
  const vda5050_core::master::PoseView& view, const std::string& manufacturer,
  const std::string& serial_number);

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__POSE_VIEW_BUILDER_HPP_
