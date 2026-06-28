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

#ifndef VDA5050_MASTER_ROS2__INTERNAL__PER_AGV_TOPIC_PUBLISHER_HPP_
#define VDA5050_MASTER_ROS2__INTERNAL__PER_AGV_TOPIC_PUBLISHER_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/internal/ros2_topic_naming.hpp"

namespace vda5050_master_ros2 {
namespace internal {

// Per-AGV ROS 2 publisher for a single message type on a single topic leaf.
// Holds one publisher per AGV in a map, lazy-created on first publish. The
// mutex guards only the map lookup/insert; the publish runs after the lock is
// released so concurrent per-AGV threads do not serialize on it (rclcpp
// publish is itself thread-safe, and the copied shared_ptr keeps the
// publisher alive even if remove_agv races the publish).
template <typename MsgT>
class PerAgvTopicPublisher
{
public:
  PerAgvTopicPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_namespace, std::string leaf,
    std::string log_tag, rclcpp::QoS qos)
  : node_(std::move(node)),
    namespace_(std::move(topic_namespace)),
    leaf_(std::move(leaf)),
    log_tag_(std::move(log_tag)),
    qos_(std::move(qos))
  {
  }

  void publish(
    const std::string& manufacturer, const std::string& serial_number,
    const MsgT& msg)
  {
    typename rclcpp::Publisher<MsgT>::SharedPtr pub;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pub = ensure_locked(manufacturer, serial_number);
    }
    pub->publish(msg);
  }

  void remove_agv(
    const std::string& manufacturer, const std::string& serial_number)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishers_.erase(key(manufacturer, serial_number));
  }

  std::string topic_for(
    const std::string& manufacturer, const std::string& serial_number) const
  {
    return build_per_agv_topic(namespace_, manufacturer, serial_number, leaf_);
  }

  bool has_publisher_for(
    const std::string& manufacturer, const std::string& serial_number) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return publishers_.count(key(manufacturer, serial_number)) != 0;
  }

private:
  static std::string key(
    const std::string& manufacturer, const std::string& serial_number)
  {
    return manufacturer + "/" + serial_number;
  }

  // Lookup or lazy-create the publisher for an AGV. Caller holds mutex_.
  typename rclcpp::Publisher<MsgT>::SharedPtr ensure_locked(
    const std::string& manufacturer, const std::string& serial_number)
  {
    const std::string id = key(manufacturer, serial_number);
    auto it = publishers_.find(id);
    if (it != publishers_.end()) return it->second;

    const std::string topic = topic_for(manufacturer, serial_number);
    auto pub = node_->create_publisher<MsgT>(topic, qos_);

    if (
      needs_topic_sanitization(manufacturer) ||
      needs_topic_sanitization(serial_number))
    {
      VDA5050_INFO(
        "[{}] sanitized ROS 2 segment for AGV {} -> topic {} (ROS 2 names "
        "cannot start with a digit; '_' was prepended)",
        log_tag_, id, topic);
    }
    VDA5050_INFO(
      "[{}] created publisher for {} on topic {}", log_tag_, id, topic);

    return publishers_.emplace(id, std::move(pub)).first->second;
  }

  rclcpp::Node::SharedPtr node_;
  const std::string namespace_;
  const std::string leaf_;
  const std::string log_tag_;
  const rclcpp::QoS qos_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, typename rclcpp::Publisher<MsgT>::SharedPtr>
    publishers_;
};

}  // namespace internal
}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__INTERNAL__PER_AGV_TOPIC_PUBLISHER_HPP_
