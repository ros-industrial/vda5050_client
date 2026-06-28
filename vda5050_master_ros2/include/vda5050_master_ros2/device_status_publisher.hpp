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

#ifndef VDA5050_MASTER_ROS2__DEVICE_STATUS_PUBLISHER_HPP_
#define VDA5050_MASTER_ROS2__DEVICE_STATUS_PUBLISHER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_interfaces/msg/connection.hpp"
#include "vda5050_interfaces/msg/factsheet.hpp"
#include "vda5050_interfaces/msg/state.hpp"
#include "vda5050_master_ros2/msg/device_status.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// DeviceStatusPublisher.
// =============================================================================
//
// Publishes per-AGV cached state on ROS 2 topics. One MASTER per fleet;
// one publisher *triple* (state / connection / factsheet) per AGV.
// Topics:
//
//   /<namespace>/<manufacturer>/<serial_number>/state       [vda5050_interfaces::msg::State]
//   /<namespace>/<manufacturer>/<serial_number>/connection  [vda5050_interfaces::msg::Connection]
//   /<namespace>/<manufacturer>/<serial_number>/factsheet   [vda5050_interfaces::msg::Factsheet]
//
// Default <namespace> is "vda5050_master"; configurable via constructor.
//
// **Trigger model**: event-driven. `VDA5050MasterROS2`'s overrides of
// `on_state` / `on_connection` / `on_factsheet` call the corresponding
// `publish_*` method here. Each cached-message arrival publishes once;
// no periodic timer.
//
// **Lazy publisher creation**: the first publish for a given AGV
// creates that AGV's publishers; subsequent calls reuse. Avoids
// pre-creating publishers for AGVs that never connect. Per-AGV state
// is held in a map under `mutex_` for thread-safety (rclcpp callbacks
// can land on the MQTT thread).
//
// **Conversion**: vda5050_types ↔ vda5050_interfaces::msg via
// nlohmann::json intermediate (vda5050_json_utils with ENABLE_ROS2).

class DeviceStatusPublisher
{
public:
  /// \brief QoS depth for the per-component topics. State is high-cadence
  ///        (every 30s+), Connection rarely changes, Factsheet very rarely.
  ///        Depth 10 covers late subscribers without unbounded memory.
  static constexpr int kQosDepth = 10;

  /// \brief QoS depth for the combined `device_status` topic. The topic is
  ///        latched (RELIABLE + TRANSIENT_LOCAL); a late subscriber receives
  ///        the most recent snapshot and we do not need history beyond that.
  static constexpr int kQosDepthCombined = 1;

  /// \brief Default ROS 2 namespace prefix for all per-AGV topics.
  static constexpr const char* kDefaultNamespace = "vda5050_master";

  /// \brief Construct.
  /// \param node            ROS 2 node that will host the publishers.
  ///                        Must outlive this object.
  /// \param topic_namespace Prefix for all per-AGV topics. Empty string
  ///                        means topics are published at the node's
  ///                        own namespace.
  explicit DeviceStatusPublisher(
    rclcpp::Node::SharedPtr node,
    const std::string& topic_namespace = kDefaultNamespace);

  ~DeviceStatusPublisher() = default;
  DeviceStatusPublisher(const DeviceStatusPublisher&) = delete;
  DeviceStatusPublisher& operator=(const DeviceStatusPublisher&) = delete;
  DeviceStatusPublisher(DeviceStatusPublisher&&) = delete;
  DeviceStatusPublisher& operator=(DeviceStatusPublisher&&) = delete;

  /// \brief Publish a State message for the given AGV. First call for
  ///        an AGV lazy-creates the publisher.
  void publish_state(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::State& state);

  /// \brief Publish a Connection message for the given AGV.
  void publish_connection(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Connection& connection);

  /// \brief Publish a Factsheet message for the given AGV.
  void publish_factsheet(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Factsheet& factsheet);

  /// \brief Publish a combined DeviceStatus message — one snapshot
  ///        carrying the latest cached State + Connection + Factsheet
  ///        for the AGV. Fields that have never been received from the
  ///        AGV are conveyed as empty bounded arrays. Latched topic
  ///        (TRANSIENT_LOCAL) so a late subscriber gets the current
  ///        snapshot immediately. Typically called from the State /
  ///        Connection / Factsheet receive callbacks after the
  ///        per-component publish, so every cache update produces a
  ///        fresh combined snapshot.
  void publish_device_status(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::master::AGV::StatusSnapshot& snapshot);

  /// \brief Drop publishers for an AGV (called from the on_offboard
  ///        override).
  void remove_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Topic name for the State publisher of the given AGV.
  ///        Useful for tests + diagnostics.
  std::string state_topic(
    const std::string& manufacturer, const std::string& serial_number) const;
  std::string connection_topic(
    const std::string& manufacturer, const std::string& serial_number) const;
  std::string factsheet_topic(
    const std::string& manufacturer, const std::string& serial_number) const;
  std::string device_status_topic(
    const std::string& manufacturer, const std::string& serial_number) const;

  /// \brief True iff publishers for this AGV have been lazy-created.
  ///        Test-only helper.
  bool has_publishers_for(
    const std::string& manufacturer, const std::string& serial_number) const;

private:
  struct PerAgvPublishers
  {
    rclcpp::Publisher<vda5050_interfaces::msg::State>::SharedPtr state;
    rclcpp::Publisher<vda5050_interfaces::msg::Connection>::SharedPtr
      connection;
    rclcpp::Publisher<vda5050_interfaces::msg::Factsheet>::SharedPtr factsheet;
    rclcpp::Publisher<vda5050_master_ros2::msg::DeviceStatus>::SharedPtr
      device_status;
  };

  // Build the AGV identity used as map key.
  static std::string agv_id(
    const std::string& manufacturer, const std::string& serial_number);

  // Build a topic name. Format: /{namespace}/{mfg}/{serial}/{leaf}.
  std::string make_topic(
    const std::string& manufacturer, const std::string& serial_number,
    const std::string& leaf) const;

  // Lookup or lazy-create publisher triple for an AGV. Caller holds mutex_.
  PerAgvPublishers& ensure_publishers_locked(
    const std::string& manufacturer, const std::string& serial_number);

  rclcpp::Node::SharedPtr node_;
  const std::string namespace_;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, PerAgvPublishers> publishers_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__DEVICE_STATUS_PUBLISHER_HPP_
