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

#ifndef VDA5050_MASTER_ROS2__ORDER_STATUS_PUBLISHER_HPP_
#define VDA5050_MASTER_ROS2__ORDER_STATUS_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/internal/per_agv_topic_publisher.hpp"
#include "vda5050_master_ros2/msg/order_status.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OrderStatusPublisher.
// =============================================================================
//
// Per-AGV ROS 2 publisher for OrderStatus messages. Topic:
//
//   /<namespace>/<manufacturer>/<serial_number>/order_status
//     [vda5050_master_ros2::msg::OrderStatus]
//
// Default <namespace> is "vda5050_master"; configurable via constructor
// (matches DeviceStatusPublisher).
//
// **Trigger**: VDA5050MasterROS2's on_state override builds an
// OrderStatus message via build_order_status_msg() and calls
// publish_order_status() — fired on every State arrival, same cadence
// as MQTT State (consistent with the tracker mandate "FMS subscribes
// and observes order progress consistent with MQTT State messages").
//
// **Lazy publisher creation**: the first publish for a given AGV
// creates that AGV's publisher; subsequent calls reuse. Per-AGV state
// is held in a map under `mutex_`. Mirrors DeviceStatusPublisher.

class OrderStatusPublisher
{
public:
  /// \brief QoS depth — same as DeviceStatusPublisher.
  static constexpr int kQosDepth = 10;

  /// \brief Default ROS 2 namespace prefix for per-AGV topics.
  static constexpr const char* kDefaultNamespace = "vda5050_master";

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the publishers. Must
  ///                        outlive this object.
  /// \param topic_namespace Prefix for all per-AGV topics. Empty string
  ///                        means topics are published at the node's
  ///                        own namespace.
  explicit OrderStatusPublisher(
    rclcpp::Node::SharedPtr node,
    const std::string& topic_namespace = kDefaultNamespace);

  ~OrderStatusPublisher() = default;
  OrderStatusPublisher(const OrderStatusPublisher&) = delete;
  OrderStatusPublisher& operator=(const OrderStatusPublisher&) = delete;
  OrderStatusPublisher(OrderStatusPublisher&&) = delete;
  OrderStatusPublisher& operator=(OrderStatusPublisher&&) = delete;

  /// \brief Publish an OrderStatus for the given AGV. First call for an
  ///        AGV lazy-creates the publisher.
  void publish_order_status(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_master_ros2::msg::OrderStatus& msg);

  /// \brief Drop the publisher for an AGV (called from the on_offboard
  ///        override).
  void remove_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Topic name for the OrderStatus publisher of the given AGV.
  std::string order_status_topic(
    const std::string& manufacturer, const std::string& serial_number) const;

  /// \brief True iff a publisher for this AGV has been lazy-created.
  ///        Test-only helper.
  bool has_publisher_for(
    const std::string& manufacturer, const std::string& serial_number) const;

private:
  internal::PerAgvTopicPublisher<vda5050_master_ros2::msg::OrderStatus> impl_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ORDER_STATUS_PUBLISHER_HPP_
