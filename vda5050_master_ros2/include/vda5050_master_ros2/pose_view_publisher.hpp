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

#ifndef VDA5050_MASTER_ROS2__POSE_VIEW_PUBLISHER_HPP_
#define VDA5050_MASTER_ROS2__POSE_VIEW_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/internal/per_agv_topic_publisher.hpp"
#include "vda5050_master_ros2/msg/pose_view.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// PoseViewPublisher.
// =============================================================================
//
// Per-AGV ROS 2 publisher for PoseView messages. Topic:
//
//   /<namespace>/<manufacturer>/<serial_number>/pose_view
//     [vda5050_master_ros2::msg::PoseView]
//
// **Trigger**: unlike the reactive status publishers, pose_view is driven by
// a fixed-rate timer in VDA5050MasterROS2 (default 1 Hz) so the cadence is
// independent of the AGV's State heartbeat and Visualization frequency.
//
// Lazy publisher creation + per-AGV map under `mutex_`, mirroring
// OrderStatusPublisher / DeviceStatusPublisher.

class PoseViewPublisher
{
public:
  /// \brief QoS depth — same as the other per-AGV publishers.
  static constexpr int kQosDepth = 10;

  /// \brief Default ROS 2 namespace prefix for per-AGV topics.
  static constexpr const char* kDefaultNamespace = "vda5050_master";

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the publishers. Must
  ///                        outlive this object.
  /// \param topic_namespace Prefix for all per-AGV topics. Empty string
  ///                        means topics are published at the node's
  ///                        own namespace.
  explicit PoseViewPublisher(
    rclcpp::Node::SharedPtr node,
    const std::string& topic_namespace = kDefaultNamespace);

  ~PoseViewPublisher() = default;
  PoseViewPublisher(const PoseViewPublisher&) = delete;
  PoseViewPublisher& operator=(const PoseViewPublisher&) = delete;
  PoseViewPublisher(PoseViewPublisher&&) = delete;
  PoseViewPublisher& operator=(PoseViewPublisher&&) = delete;

  /// \brief Publish a PoseView for the given AGV. First call for an AGV
  ///        lazy-creates the publisher.
  void publish_pose_view(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_master_ros2::msg::PoseView& msg);

  /// \brief Drop the publisher for an AGV (called from the on_offboard
  ///        override).
  void remove_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Topic name for the PoseView publisher of the given AGV.
  std::string pose_view_topic(
    const std::string& manufacturer, const std::string& serial_number) const;

  /// \brief True iff a publisher for this AGV has been lazy-created.
  ///        Test-only helper.
  bool has_publisher_for(
    const std::string& manufacturer, const std::string& serial_number) const;

private:
  internal::PerAgvTopicPublisher<vda5050_master_ros2::msg::PoseView> impl_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__POSE_VIEW_PUBLISHER_HPP_
