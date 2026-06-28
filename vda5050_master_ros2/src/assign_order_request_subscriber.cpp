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

#include "vda5050_master_ros2/assign_order_request_subscriber.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"
#include "vda5050_master_ros2/msg/assignment_result.hpp"

namespace vda5050_master_ros2 {
namespace {

// Map the C++ AssignmentDecision to the topic's AssignmentResult enum.
// All non-success outcomes fold into REJECTED_PREFLIGHT because the
// sync dispatcher runs entirely before the order enters the lifecycle.
// REJECTED_POSTFLIGHT is reserved for later events (factsheet change,
// stitch timeout, etc.) and is not emitted by this subscriber today.
std::uint8_t map_decision(vda5050_core::master::AssignmentDecision d)
{
  using AR = vda5050_master_ros2::msg::AssignmentResult;
  switch (d)
  {
    case vda5050_core::master::AssignmentDecision::ASSIGNED:
      return AR::ACCEPTED;
    case vda5050_core::master::AssignmentDecision::STITCH_QUEUED:
      return AR::QUEUED;
    case vda5050_core::master::AssignmentDecision::AGV_NOT_ONBOARDED:
    case vda5050_core::master::AssignmentDecision::AGV_OFFLINE:
    case vda5050_core::master::AssignmentDecision::AGV_NOT_READY:
    case vda5050_core::master::AssignmentDecision::AGV_MODE_NOT_AUTO:
    case vda5050_core::master::AssignmentDecision::AGV_POSITION_NOT_INITIALIZED:
    case vda5050_core::master::AssignmentDecision::AGV_NO_STATE_YET:
    case vda5050_core::master::AssignmentDecision::STITCH_REJECTED:
      return AR::REJECTED_PREFLIGHT;
  }
  return AR::REJECTED_PREFLIGHT;  // unreachable
}

}  // namespace

std::string AssignOrderRequestSubscriber::make_topic_name(
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

AssignOrderRequestSubscriber::AssignOrderRequestSubscriber(
  rclcpp::Node::SharedPtr node, AssignmentSender sender,
  AssignmentRecorder recorder,
  std::shared_ptr<AssignmentResultPublisher> result_publisher,
  const std::string& topic_namespace)
: node_(std::move(node)),
  sender_(std::move(sender)),
  recorder_(std::move(recorder)),
  result_publisher_(std::move(result_publisher)),
  topic_name_(make_topic_name(topic_namespace))
{
  rclcpp::QoS qos(kQosDepth);
  qos.reliable();  // VOLATILE durability — requests do not replay.

  sub_ =
    node_->create_subscription<vda5050_master_ros2::msg::AssignOrderRequest>(
      topic_name_, qos,
      [this](vda5050_master_ros2::msg::AssignOrderRequest::SharedPtr msg) {
        this->on_request(msg);
      });

  VDA5050_INFO("[AssignOrderRequestSubscriber] subscribed to {}", topic_name_);
}

void AssignOrderRequestSubscriber::on_request(
  vda5050_master_ros2::msg::AssignOrderRequest::SharedPtr msg)
{
  using AR = vda5050_master_ros2::msg::AssignmentResult;

  if (
    msg->assignment_id.empty() || msg->manufacturer.empty() ||
    msg->serial_number.empty())
  {
    vda5050_interfaces::msg::Error err;
    err.error_type = "InvalidRequest";
    err.error_level = vda5050_interfaces::msg::Error::ERROR_LEVEL_FATAL;
    err.error_description.push_back(
      "AssignOrderRequest: assignment_id, manufacturer, and serial_number "
      "must all be non-empty");
    result_publisher_->publish_result(
      msg->assignment_id, msg->order.order_id, msg->order.order_update_id,
      AR::REJECTED_PREFLIGHT, {err});
    return;
  }

  vda5050_core::types::Order order_typed = internal::from_msg<
    vda5050_interfaces::msg::Order, vda5050_core::types::Order>(msg->order);

  const vda5050_core::master::AssignmentResult result =
    sender_(msg->manufacturer, msg->serial_number, order_typed);

  const std::uint8_t decision = map_decision(result.decision);
  std::vector<vda5050_interfaces::msg::Error> error_msgs;
  error_msgs.reserve(result.errors.size());
  for (const auto& err : result.errors)
  {
    error_msgs.push_back(
      internal::to_msg<
        vda5050_core::types::Error, vda5050_interfaces::msg::Error>(err));
  }

  if (recorder_ && (decision == AR::ACCEPTED || decision == AR::QUEUED))
  {
    recorder_(
      msg->manufacturer, msg->serial_number, msg->assignment_id,
      msg->order.order_id, msg->order.order_update_id);
  }

  result_publisher_->publish_result(
    msg->assignment_id, msg->order.order_id, msg->order.order_update_id,
    decision, error_msgs);
}

}  // namespace vda5050_master_ros2
