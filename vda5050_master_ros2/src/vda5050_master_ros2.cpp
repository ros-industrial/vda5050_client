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

#include "vda5050_master_ros2/vda5050_master_ros2.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "fmt/format.h"
#include "vda5050_master_ros2/order_status_builder.hpp"
#include "vda5050_master_ros2/pose_view_builder.hpp"

namespace vda5050_master_ros2 {

VDA5050MasterROS2::VDA5050MasterROS2(
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client,
  rclcpp::Node::SharedPtr ros2_node, const std::string& topic_namespace,
  double pose_view_rate_hz)
: vda5050_core::master::VDA5050Master(std::move(mqtt_client)),
  node_(ros2_node),
  device_status_(
    std::make_unique<DeviceStatusPublisher>(ros2_node, topic_namespace)),
  order_status_publisher_(
    std::make_unique<OrderStatusPublisher>(ros2_node, topic_namespace)),
  pose_view_publisher_(
    std::make_unique<PoseViewPublisher>(ros2_node, topic_namespace)),
  assignment_result_publisher_(
    std::make_shared<AssignmentResultPublisher>(ros2_node, topic_namespace)),
  assign_order_request_subscriber_(
    std::make_unique<AssignOrderRequestSubscriber>(
      std::move(ros2_node),
      [this](
        const std::string& mfg, const std::string& serial,
        const vda5050_core::types::Order& order) {
        return this->assign_order(mfg, serial, order);
      },
      [this](
        const std::string& mfg, const std::string& serial,
        const std::string& assignment_id, const std::string& order_id,
        std::uint32_t order_update_id) {
        this->record_assignment(
          mfg, serial, assignment_id, order_id, order_update_id);
      },
      assignment_result_publisher_, topic_namespace))
{
  if (pose_view_rate_hz <= 0.0 || pose_view_rate_hz > kMaxPoseViewRateHz)
  {
    throw std::invalid_argument(fmt::format(
      "pose_view_rate_hz must be in (0, {}] Hz", kMaxPoseViewRateHz));
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / pose_view_rate_hz));
  pose_view_timer_ =
    node_->create_wall_timer(period, [this]() { this->publish_pose_views(); });
}

VDA5050MasterROS2::~VDA5050MasterROS2()
{
  // Stop the periodic callback before the members it touches are torn down.
  // The caller is expected to have stopped spinning the executor first.
  if (pose_view_timer_) pose_view_timer_->cancel();
}

void VDA5050MasterROS2::publish_pose_views()
{
  for (const auto& [mfg, serial] : get_onboarded_agvs())
  {
    auto agv = get_agv(mfg, serial);
    if (!agv) continue;
    const vda5050_core::master::PoseView view = agv->get_pose_view();
    if (view.source == vda5050_core::master::PoseSource::None) continue;
    pose_view_publisher_->publish_pose_view(
      mfg, serial, build_pose_view_msg(view, mfg, serial));
  }
}

std::pair<std::string, std::string> VDA5050MasterROS2::split_agv_id(
  const std::string& agv_id)
{
  // master keeps agvs_ keyed as "{mfg}/{serial}".
  const auto slash = agv_id.find('/');
  if (slash == std::string::npos) return {agv_id, std::string{}};
  return {agv_id.substr(0, slash), agv_id.substr(slash + 1)};
}

void VDA5050MasterROS2::on_state(
  const std::string& agv_id, const vda5050_core::types::State& state)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_state(mfg, serial, state);

  // OrderStatus + combined DeviceStatus alongside the per-component
  // State stream. handle_state has already cached the new State, so
  // both the order bundle and the status snapshot reflect it.
  if (auto agv = get_agv(mfg, serial))
  {
    auto bundle = agv->get_order_status_bundle();
    auto msg = build_order_status_msg(
      bundle, mfg, serial, get_active_assignment_id(mfg, serial));
    order_status_publisher_->publish_order_status(mfg, serial, msg);
    device_status_->publish_device_status(
      mfg, serial, agv->get_status_snapshot());
  }

  vda5050_core::master::VDA5050Master::on_state(agv_id, state);
}

void VDA5050MasterROS2::on_connection(
  const std::string& agv_id, const vda5050_core::types::Connection& connection)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_connection(mfg, serial, connection);
  if (auto agv = get_agv(mfg, serial))
  {
    device_status_->publish_device_status(
      mfg, serial, agv->get_status_snapshot());
  }
  vda5050_core::master::VDA5050Master::on_connection(agv_id, connection);
}

void VDA5050MasterROS2::on_factsheet(
  const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_factsheet(mfg, serial, factsheet);
  if (auto agv = get_agv(mfg, serial))
  {
    device_status_->publish_device_status(
      mfg, serial, agv->get_status_snapshot());
  }
  vda5050_core::master::VDA5050Master::on_factsheet(agv_id, factsheet);
}

void VDA5050MasterROS2::on_offboard(const std::string& agv_id)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->remove_agv(mfg, serial);
  order_status_publisher_->remove_agv(mfg, serial);
  pose_view_publisher_->remove_agv(mfg, serial);
  vda5050_core::master::VDA5050Master::on_offboard(agv_id);
}

}  // namespace vda5050_master_ros2
