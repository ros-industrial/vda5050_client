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

#ifndef VDA5050_MASTER_ROS2__ASSIGN_ORDER_REQUEST_SUBSCRIBER_HPP_
#define VDA5050_MASTER_ROS2__ASSIGN_ORDER_REQUEST_SUBSCRIBER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/assignment_result.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_master_ros2/assignment_result_publisher.hpp"
#include "vda5050_master_ros2/msg/assign_order_request.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// AssignOrderRequestSubscriber — wire-async dispatch path for RES /
// MAPF / FMS.
// =============================================================================
//
// Topic: /<topic_namespace>/assign_order_request  [AssignOrderRequest]
//
// On each incoming request the subscriber:
//   1. Validates request shape (assignment_id, mfg, serial all non-empty).
//   2. Converts the embedded msg::Order to types::Order.
//   3. Dispatches via the injected AssignmentSender (typically
//      VDA5050Master::assign_order).
//   4. Maps the resulting AssignmentDecision to the topic's
//      AssignmentResult enum.
//   5. On ACCEPTED / QUEUED, records the assignment_id via the injected
//      AssignmentRecorder so the OrderStatus stream can echo it back.
//   6. Publishes the AssignmentResult via the shared publisher.

class AssignOrderRequestSubscriber
{
public:
  /// \brief Topic-name leaf appended after the namespace prefix.
  static constexpr const char* kTopicLeaf = "assign_order_request";

  /// \brief QoS depth.
  static constexpr int kQosDepth = 10;

  /// \brief Default ROS 2 namespace prefix.
  static constexpr const char* kDefaultNamespace = "vda5050_master";

  /// \brief Synchronous dispatcher into the master. Typically wraps
  ///        VDA5050Master::assign_order(...).
  using AssignmentSender = std::function<vda5050_core::master::AssignmentResult(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order)>;

  /// \brief Records the assignment_id correlation. Typically wraps
  ///        VDA5050Master::record_assignment(...).
  using AssignmentRecorder = std::function<void(
    const std::string& manufacturer, const std::string& serial_number,
    const std::string& assignment_id, const std::string& order_id,
    std::uint32_t order_update_id)>;

  /// \brief Construct.
  /// \param node                ROS 2 node hosting the subscription.
  ///                            Must outlive this object.
  /// \param sender              Sync dispatcher into the master.
  /// \param recorder            Records assignment_id when the
  ///                            dispatcher returns ACCEPTED / QUEUED.
  ///                            May be null — in that case the
  ///                            assignment_id correlation will not flow
  ///                            into the OrderStatus stream, but the
  ///                            result topic still publishes correctly.
  /// \param result_publisher    Sink for outcome messages.
  /// \param topic_namespace     Prefix for the topic name.
  AssignOrderRequestSubscriber(
    rclcpp::Node::SharedPtr node, AssignmentSender sender,
    AssignmentRecorder recorder,
    std::shared_ptr<AssignmentResultPublisher> result_publisher,
    const std::string& topic_namespace = kDefaultNamespace);

  ~AssignOrderRequestSubscriber() = default;
  AssignOrderRequestSubscriber(const AssignOrderRequestSubscriber&) = delete;
  AssignOrderRequestSubscriber& operator=(const AssignOrderRequestSubscriber&) =
    delete;
  AssignOrderRequestSubscriber(AssignOrderRequestSubscriber&&) = delete;
  AssignOrderRequestSubscriber& operator=(AssignOrderRequestSubscriber&&) =
    delete;

  /// \brief The fully-qualified ROS 2 topic name.
  const std::string& topic_name() const
  {
    return topic_name_;
  }

private:
  void on_request(vda5050_master_ros2::msg::AssignOrderRequest::SharedPtr msg);

  static std::string make_topic_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  AssignmentSender sender_;
  AssignmentRecorder recorder_;
  std::shared_ptr<AssignmentResultPublisher> result_publisher_;
  std::string topic_name_;
  rclcpp::Subscription<vda5050_master_ros2::msg::AssignOrderRequest>::SharedPtr
    sub_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ASSIGN_ORDER_REQUEST_SUBSCRIBER_HPP_
