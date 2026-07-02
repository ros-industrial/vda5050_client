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

#include "vda5050_master_ros2/pose_view_builder.hpp"

#include <chrono>
#include <string>

#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/duration.hpp"
#include "vda5050_interfaces/msg/agv_position.hpp"
#include "vda5050_interfaces/msg/velocity.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
namespace {

builtin_interfaces::msg::Time now_as_ros_time()
{
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  const auto secs =
    std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(secs.count());
  t.nanosec = static_cast<uint32_t>(nsecs.count());
  return t;
}

uint8_t to_source_enum(vda5050_core::master::PoseSource source)
{
  using PoseView = vda5050_master_ros2::msg::PoseView;
  switch (source)
  {
    case vda5050_core::master::PoseSource::None:
      return PoseView::SOURCE_NONE;
    case vda5050_core::master::PoseSource::Visualization:
      return PoseView::SOURCE_VISUALIZATION;
    case vda5050_core::master::PoseSource::State:
      return PoseView::SOURCE_STATE;
    case vda5050_core::master::PoseSource::Extrapolated:
      return PoseView::SOURCE_EXTRAPOLATED;
  }
  return PoseView::SOURCE_NONE;
}

}  // namespace

vda5050_master_ros2::msg::PoseView build_pose_view_msg(
  const vda5050_core::master::PoseView& view, const std::string& manufacturer,
  const std::string& serial_number)
{
  vda5050_master_ros2::msg::PoseView msg;
  msg.manufacturer = manufacturer;
  msg.serial_number = serial_number;
  msg.stamp = now_as_ros_time();
  msg.source = to_source_enum(view.source);
  msg.data_age = static_cast<builtin_interfaces::msg::Duration>(
    rclcpp::Duration(view.data_age));
  msg.driving = view.driving;

  if (view.agv_position)
  {
    msg.agv_position.push_back(
      internal::to_msg<
        vda5050_core::types::AGVPosition, vda5050_interfaces::msg::AGVPosition>(
        *view.agv_position));
  }
  if (view.velocity)
  {
    msg.velocity.push_back(
      internal::to_msg<
        vda5050_core::types::Velocity, vda5050_interfaces::msg::Velocity>(
        *view.velocity));
  }

  return msg;
}

}  // namespace vda5050_master_ros2
