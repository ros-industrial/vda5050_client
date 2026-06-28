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

#ifndef VDA5050_MASTER_ROS2__ASSIGNMENT_RESULT_PUBLISHER_HPP_
#define VDA5050_MASTER_ROS2__ASSIGNMENT_RESULT_PUBLISHER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "vda5050_master_ros2/msg/assignment_result.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// AssignmentResultPublisher — dispatch-outcome publisher for the
// async order dispatch path. Companion to AssignOrderRequestSubscriber.
// =============================================================================
//
// Single shared topic across all AGVs (master → caller):
//
//   /<topic_namespace>/assignment_results  [AssignmentResult]
//
// QoS: RELIABLE + VOLATILE, KEEP_LAST(10). One-shot per assignment;
// late subscribers do not need history beyond the depth window.

class AssignmentResultPublisher
{
public:
  /// \brief Topic-name leaf appended after the namespace prefix.
  static constexpr const char* kTopicLeaf = "assignment_results";

  /// \brief QoS depth.
  static constexpr int kQosDepth = 10;

  /// \brief Default ROS 2 namespace prefix.
  static constexpr const char* kDefaultNamespace = "vda5050_master";

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the publisher. Must
  ///                        outlive this object.
  /// \param topic_namespace Prefix for the topic name.
  explicit AssignmentResultPublisher(
    rclcpp::Node::SharedPtr node,
    const std::string& topic_namespace = kDefaultNamespace);

  ~AssignmentResultPublisher() = default;
  AssignmentResultPublisher(const AssignmentResultPublisher&) = delete;
  AssignmentResultPublisher& operator=(const AssignmentResultPublisher&) =
    delete;
  AssignmentResultPublisher(AssignmentResultPublisher&&) = delete;
  AssignmentResultPublisher& operator=(AssignmentResultPublisher&&) = delete;

  /// \brief Publish one AssignmentResult.
  /// \param assignment_id    Correlation UUID echoed from the request.
  /// \param order_id         Order id from the request.
  /// \param order_update_id  Order update id from the request.
  /// \param decision         AssignmentResult::ACCEPTED / QUEUED /
  ///                         REJECTED_PREFLIGHT / REJECTED_POSTFLIGHT.
  /// \param errors           Diagnostic errors. Empty on
  ///                         ACCEPTED / QUEUED.
  void publish_result(
    const std::string& assignment_id, const std::string& order_id,
    std::uint32_t order_update_id, std::uint8_t decision,
    const std::vector<vda5050_interfaces::msg::Error>& errors);

  /// \brief The fully-qualified ROS 2 topic name.
  const std::string& topic_name() const
  {
    return topic_name_;
  }

private:
  static std::string make_topic_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  std::string topic_name_;
  rclcpp::Publisher<vda5050_master_ros2::msg::AssignmentResult>::SharedPtr pub_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ASSIGNMENT_RESULT_PUBLISHER_HPP_
