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

#ifndef VDA5050_CORE__MASTER__POSE_VIEW_HPP_
#define VDA5050_CORE__MASTER__POSE_VIEW_HPP_

#include <chrono>
#include <optional>

#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/velocity.hpp"

namespace vda5050_core::master {

/// \brief Which cached message supplied the position / velocity in a PoseView.
enum class PoseSource
{
  None,  // No position received yet; agv_position / velocity empty.
  Visualization,
  State,
  Extrapolated  // Reserved for the extrapolation mode; never set in relay mode.
};

/// \brief Fused per-AGV pose for the pose_view stream.
///
/// Carries the freshest of the AGV's cached State / Visualization position and
/// velocity, with driving relayed from State. data_age is the age of the
/// underlying sample measured against the master receive time.
struct PoseView
{
  bool driving = false;
  std::optional<vda5050_core::types::AGVPosition> agv_position;
  std::optional<vda5050_core::types::Velocity> velocity;
  PoseSource source = PoseSource::None;
  std::chrono::nanoseconds data_age{0};
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__POSE_VIEW_HPP_
