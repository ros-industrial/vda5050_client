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

#include "vda5050_master_ros2/assignment_result_publisher.hpp"

#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {

std::string AssignmentResultPublisher::make_topic_name(
  const std::string& topic_namespace)
{
  std::string name = "/";
  if (!topic_namespace.empty())
  {
    name += topic_namespace;
    name += "/";
  }
  name += kTopicLeaf;
  return name;
}

AssignmentResultPublisher::AssignmentResultPublisher(
  rclcpp::Node::SharedPtr node, const std::string& topic_namespace)
: node_(std::move(node)), topic_name_(make_topic_name(topic_namespace))
{
  rclcpp::QoS qos(kQosDepth);
  qos.reliable();  // VOLATILE durability by default
  pub_ = node_->create_publisher<vda5050_master_ros2::msg::AssignmentResult>(
    topic_name_, qos);

  VDA5050_INFO("[AssignmentResultPublisher] publishing on {}", topic_name_);
}

void AssignmentResultPublisher::publish_result(
  const std::string& assignment_id, const std::string& order_id,
  std::uint32_t order_update_id, std::uint8_t decision,
  const std::vector<vda5050_interfaces::msg::Error>& errors)
{
  vda5050_master_ros2::msg::AssignmentResult msg;
  msg.assignment_id = assignment_id;
  msg.order_id = order_id;
  msg.order_update_id = order_update_id;
  msg.decision = decision;
  msg.errors = errors;
  msg.stamp = node_->now();
  pub_->publish(std::move(msg));
}

}  // namespace vda5050_master_ros2
