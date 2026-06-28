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

#include "vda5050_master_ros2/pose_view_publisher.hpp"

#include <string>
#include <utility>

namespace vda5050_master_ros2 {

PoseViewPublisher::PoseViewPublisher(
  rclcpp::Node::SharedPtr node, const std::string& topic_namespace)
: impl_(
    std::move(node), topic_namespace, "pose_view", "PoseViewPublisher",
    rclcpp::QoS(kQosDepth))
{
}

void PoseViewPublisher::publish_pose_view(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_master_ros2::msg::PoseView& msg)
{
  impl_.publish(manufacturer, serial_number, msg);
}

void PoseViewPublisher::remove_agv(
  const std::string& manufacturer, const std::string& serial_number)
{
  impl_.remove_agv(manufacturer, serial_number);
}

std::string PoseViewPublisher::pose_view_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return impl_.topic_for(manufacturer, serial_number);
}

bool PoseViewPublisher::has_publisher_for(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return impl_.has_publisher_for(manufacturer, serial_number);
}

}  // namespace vda5050_master_ros2
