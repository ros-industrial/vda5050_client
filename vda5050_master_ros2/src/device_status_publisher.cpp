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

#include "vda5050_master_ros2/device_status_publisher.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/internal/ros2_topic_naming.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
using internal::to_msg;

namespace {

builtin_interfaces::msg::Time to_ros_time(
  const vda5050_core::master::AGV::TimePoint& tp)
{
  const auto since_epoch = tp.time_since_epoch();
  const auto secs =
    std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto nsecs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - secs);
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(secs.count());
  out.nanosec = static_cast<std::uint32_t>(nsecs.count());
  return out;
}

}  // namespace

DeviceStatusPublisher::DeviceStatusPublisher(
  rclcpp::Node::SharedPtr node, const std::string& topic_namespace)
: node_(std::move(node)), namespace_(topic_namespace)
{
}

std::string DeviceStatusPublisher::agv_id(
  const std::string& manufacturer, const std::string& serial_number)
{
  return manufacturer + "/" + serial_number;
}

std::string DeviceStatusPublisher::make_topic(
  const std::string& manufacturer, const std::string& serial_number,
  const std::string& leaf) const
{
  return internal::build_per_agv_topic(
    namespace_, manufacturer, serial_number, leaf);
}

std::string DeviceStatusPublisher::state_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "state");
}

std::string DeviceStatusPublisher::connection_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "connection");
}

std::string DeviceStatusPublisher::factsheet_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "factsheet");
}

std::string DeviceStatusPublisher::device_status_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "device_status");
}

bool DeviceStatusPublisher::has_publishers_for(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return publishers_.count(agv_id(manufacturer, serial_number)) != 0;
}

DeviceStatusPublisher::PerAgvPublishers&
DeviceStatusPublisher::ensure_publishers_locked(
  const std::string& manufacturer, const std::string& serial_number)
{
  const std::string id = agv_id(manufacturer, serial_number);
  auto it = publishers_.find(id);
  if (it != publishers_.end()) return it->second;

  PerAgvPublishers pubs;
  pubs.state = node_->create_publisher<vda5050_interfaces::msg::State>(
    state_topic(manufacturer, serial_number), kQosDepth);
  pubs.connection =
    node_->create_publisher<vda5050_interfaces::msg::Connection>(
      connection_topic(manufacturer, serial_number), kQosDepth);
  pubs.factsheet = node_->create_publisher<vda5050_interfaces::msg::Factsheet>(
    factsheet_topic(manufacturer, serial_number), kQosDepth);

  // Combined snapshot — latched so a late subscriber gets the current
  // view immediately, without waiting for the next State arrival.
  rclcpp::QoS combined_qos(kQosDepthCombined);
  combined_qos.reliable().transient_local();
  pubs.device_status =
    node_->create_publisher<vda5050_master_ros2::msg::DeviceStatus>(
      device_status_topic(manufacturer, serial_number), combined_qos);

  if (
    internal::needs_topic_sanitization(manufacturer) ||
    internal::needs_topic_sanitization(serial_number))
  {
    VDA5050_INFO(
      "[DeviceStatusPublisher] sanitized ROS 2 segment for AGV {} "
      "(ROS 2 names cannot start with a digit; '_' was prepended)",
      id);
  }
  VDA5050_INFO(
    "[DeviceStatusPublisher] created publishers for {} on topics {} / {} / "
    "{} / {}",
    id, state_topic(manufacturer, serial_number),
    connection_topic(manufacturer, serial_number),
    factsheet_topic(manufacturer, serial_number),
    device_status_topic(manufacturer, serial_number));

  return publishers_.emplace(id, std::move(pubs)).first->second;
}

void DeviceStatusPublisher::publish_state(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::State& state)
{
  rclcpp::Publisher<vda5050_interfaces::msg::State>::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pub = ensure_publishers_locked(manufacturer, serial_number).state;
  }
  pub->publish(
    to_msg<vda5050_core::types::State, vda5050_interfaces::msg::State>(state));
}

void DeviceStatusPublisher::publish_connection(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::Connection& connection)
{
  rclcpp::Publisher<vda5050_interfaces::msg::Connection>::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pub = ensure_publishers_locked(manufacturer, serial_number).connection;
  }
  pub->publish(
    to_msg<
      vda5050_core::types::Connection, vda5050_interfaces::msg::Connection>(
      connection));
}

void DeviceStatusPublisher::publish_factsheet(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::Factsheet& factsheet)
{
  rclcpp::Publisher<vda5050_interfaces::msg::Factsheet>::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pub = ensure_publishers_locked(manufacturer, serial_number).factsheet;
  }
  pub->publish(
    to_msg<vda5050_core::types::Factsheet, vda5050_interfaces::msg::Factsheet>(
      factsheet));
}

void DeviceStatusPublisher::publish_device_status(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::master::AGV::StatusSnapshot& snapshot)
{
  vda5050_master_ros2::msg::DeviceStatus msg;
  msg.manufacturer = manufacturer;
  msg.serial_number = serial_number;
  msg.stamp = node_->now();

  if (snapshot.state.has_value())
  {
    msg.state.push_back(
      to_msg<vda5050_core::types::State, vda5050_interfaces::msg::State>(
        *snapshot.state));
    if (snapshot.state_received_at.has_value())
    {
      msg.state_received_at.push_back(to_ros_time(*snapshot.state_received_at));
    }
  }
  if (snapshot.connection.has_value())
  {
    msg.connection.push_back(
      to_msg<
        vda5050_core::types::Connection, vda5050_interfaces::msg::Connection>(
        *snapshot.connection));
    if (snapshot.connection_received_at.has_value())
    {
      msg.connection_received_at.push_back(
        to_ros_time(*snapshot.connection_received_at));
    }
  }
  if (snapshot.factsheet.has_value())
  {
    msg.factsheet.push_back(
      to_msg<
        vda5050_core::types::Factsheet, vda5050_interfaces::msg::Factsheet>(
        *snapshot.factsheet));
    if (snapshot.factsheet_received_at.has_value())
    {
      msg.factsheet_received_at.push_back(
        to_ros_time(*snapshot.factsheet_received_at));
    }
  }

  rclcpp::Publisher<vda5050_master_ros2::msg::DeviceStatus>::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pub = ensure_publishers_locked(manufacturer, serial_number).device_status;
  }
  pub->publish(std::move(msg));
}

void DeviceStatusPublisher::remove_agv(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishers_.erase(agv_id(manufacturer, serial_number));
}

}  // namespace vda5050_master_ros2
