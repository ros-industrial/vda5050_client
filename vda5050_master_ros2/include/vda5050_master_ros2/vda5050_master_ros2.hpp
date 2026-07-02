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

#ifndef VDA5050_MASTER_ROS2__VDA5050_MASTER_ROS2_HPP_
#define VDA5050_MASTER_ROS2__VDA5050_MASTER_ROS2_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_master_ros2/assign_order_request_subscriber.hpp"
#include "vda5050_master_ros2/assignment_result_publisher.hpp"
#include "vda5050_master_ros2/device_status_publisher.hpp"
#include "vda5050_master_ros2/order_status_publisher.hpp"
#include "vda5050_master_ros2/pose_view_publisher.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// VDA5050MasterROS2 — topic-based ROS 2 API.
// =============================================================================
//
// Opt-in subclass of VDA5050Master that publishes the master's per-AGV data
// to ROS 2 and accepts order dispatch over topics. FMS deployments that want
// ROS 2 integration construct `VDA5050MasterROS2` instead of `VDA5050Master`;
// pure C++ users keep `VDA5050Master` and pay nothing for ROS 2.
//
// This is the topic/message layer:
//   * Per-AGV publishers — State / Connection / Factsheet / combined
//     DeviceStatus / OrderStatus, fired from the `on_state` / `on_connection`
//     / `on_factsheet` hooks; plus a fixed-rate PoseView stream (timer).
//   * Async order dispatch — AssignOrderRequest subscriber → AssignmentResult
//     publisher (caller-keyed UUIDs).
//
// Synchronous request/response services and Device Manager integration
// (FleetRoster / MasterConnection) are intentionally NOT part of this layer —
// they land in follow-up PRs.
//
// **Override chaining**: each `on_*` override calls `VDA5050Master::on_*(...)`
// after publishing so further FMS subclass semantics are preserved.

class VDA5050MasterROS2 : public vda5050_core::master::VDA5050Master
{
public:
  /// \brief Default pose_view publish rate (Hz) when not otherwise configured.
  static constexpr double kDefaultPoseViewRateHz = 1.0;
  /// \brief Upper bound on the configurable pose_view rate (Hz); rejected
  ///        above.
  static constexpr double kMaxPoseViewRateHz = 1000.0;

  /// \brief Construct.
  ///
  /// Like the base VDA5050Master, this MUST be created via
  /// std::make_shared<VDA5050MasterROS2>(...). The base dispatches AGV
  /// callbacks through weak_from_this(); a stack- or unique_ptr-allocated
  /// instance silently publishes nothing per-AGV.
  ///
  /// \param mqtt_client      Same MQTT client passed to base; required.
  /// \param ros2_node        ROS 2 node hosting publishers.
  ///                         Caller spins. Must outlive this object.
  /// \param topic_namespace  Prefix for per-AGV ROS 2 topics. Default
  ///                         "vda5050_master".
  /// \param pose_view_rate_hz Fixed publish rate (Hz) for the per-AGV
  ///                          pose_view stream. Must be in
  ///                          (0, kMaxPoseViewRateHz]; out-of-range throws.
  VDA5050MasterROS2(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client,
    rclcpp::Node::SharedPtr ros2_node,
    const std::string& topic_namespace =
      DeviceStatusPublisher::kDefaultNamespace,
    double pose_view_rate_hz = kDefaultPoseViewRateHz);

  ~VDA5050MasterROS2() override;

  VDA5050MasterROS2(const VDA5050MasterROS2&) = delete;
  VDA5050MasterROS2& operator=(const VDA5050MasterROS2&) = delete;
  VDA5050MasterROS2(VDA5050MasterROS2&&) = delete;
  VDA5050MasterROS2& operator=(VDA5050MasterROS2&&) = delete;

  /// \brief Read access to the DeviceStatus publisher (test + diagnostics).
  DeviceStatusPublisher& device_status_publisher()
  {
    return *device_status_;
  }

  /// \brief Read access to the OrderStatus publisher (test + diagnostics).
  OrderStatusPublisher& order_status_publisher()
  {
    return *order_status_publisher_;
  }

  /// \brief Read access to the PoseView publisher (test + diagnostics).
  PoseViewPublisher& pose_view_publisher()
  {
    return *pose_view_publisher_;
  }

  // ============================================================================
  // VDA5050Master extension callback overrides
  // ============================================================================
  //
  // These intercept the raw cached-message arrivals and publish to ROS 2
  // before chaining to the base implementation (default empty). Further
  // subclasses must call VDA5050MasterROS2::on_*(...) to keep publishing.

  void on_state(
    const std::string& agv_id,
    const vda5050_core::types::State& state) override;
  void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection) override;
  void on_factsheet(
    const std::string& agv_id,
    const vda5050_core::types::Factsheet& factsheet) override;
  void on_offboard(const std::string& agv_id) override;

private:
  // agv_id is "{manufacturer}/{serial_number}". Split it back for topic
  // naming.
  static std::pair<std::string, std::string> split_agv_id(
    const std::string& agv_id);

  // Timer callback: publish a fused PoseView for every onboarded AGV.
  void publish_pose_views();

  // False when this AGV's sanitized ROS 2 topic identity collides with one
  // already claimed by a different AGV; its ROS 2 topics are then refused (a
  // one-shot ERROR is logged). Decided once per AGV.
  bool ros2_topics_allowed(
    const std::string& agv_id, const std::string& manufacturer,
    const std::string& serial_number);

  rclcpp::Node::SharedPtr node_;
  // Shared by every per-AGV publisher so concurrent create_publisher() calls
  // from the MQTT and timer threads serialize on one node-wide mutex.
  std::shared_ptr<std::mutex> node_create_mutex_;
  std::unique_ptr<DeviceStatusPublisher> device_status_;
  std::unique_ptr<OrderStatusPublisher> order_status_publisher_;
  std::unique_ptr<PoseViewPublisher> pose_view_publisher_;
  // Async-dispatch — created publisher-first so the subscriber can
  // capture a shared reference at construction time.
  std::shared_ptr<AssignmentResultPublisher> assignment_result_publisher_;
  std::unique_ptr<AssignOrderRequestSubscriber>
    assign_order_request_subscriber_;

  // ROS 2 topic-name collision guard: distinct raw identities can sanitize to
  // the same topic segment pair. The first claims it; later colliders land in
  // refused_agvs_ and get no ROS 2 topics.
  std::mutex topic_identity_mutex_;
  std::unordered_map<std::string, std::string> claimed_topic_identities_;
  std::unordered_set<std::string> refused_agvs_;

  // Guards pose-timer teardown: the destructor sets shutting_down_ under this
  // mutex so it waits out any in-flight publish_pose_views() before the members
  // it touches are destroyed (safe even under a multi-threaded executor).
  std::mutex pose_timer_mutex_;
  bool shutting_down_ = false;

  // Declared last so it is destroyed first — the timer callback touches the
  // pose_view publisher and the base AGV registry, which must outlive it.
  rclcpp::TimerBase::SharedPtr pose_view_timer_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__VDA5050_MASTER_ROS2_HPP_
