/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
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

#include <utility>

#include "vda5050_core/logger/logger.hpp"

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/master/standard_names.hpp"

namespace vda5050_core {

namespace master {

//=============================================================================
VDA5050Master::VDA5050Master(std::shared_ptr<MqttClientInterface> mqtt_client)
: mqtt_client_(std::move(mqtt_client))
{
  VDA5050_INFO("[VDA5050Master] Created VDA5050Master instance");
}

//=============================================================================
VDA5050Master::~VDA5050Master()
{
  VDA5050_INFO("[VDA5050Master] Destroying VDA5050Master instance");
  disconnect();
  VDA5050_INFO("[VDA5050Master] VDA5050Master instance destroyed");
}

//=============================================================================
void VDA5050Master::connect()
{
  if (!mqtt_client_)
  {
    VDA5050_WARN("[VDA5050Master] Cannot connect: no MQTT client");
    return;
  }

  if (mqtt_client_->connected())
  {
    VDA5050_WARN("[VDA5050Master] Already connected");
    return;
  }

  VDA5050_INFO("[VDA5050Master] Connecting MQTT client");
  mqtt_client_->connect();
}

//=============================================================================
void VDA5050Master::disconnect()
{
  if (!mqtt_client_)
  {
    return;
  }

  if (!mqtt_client_->connected())
  {
    return;
  }

  VDA5050_INFO("[VDA5050Master] Disconnecting MQTT client");
  mqtt_client_->disconnect();
  VDA5050_INFO("[VDA5050Master] Disconnected");
}

//=============================================================================
bool VDA5050Master::is_connected() const
{
  return mqtt_client_ && mqtt_client_->connected();
}

//=============================================================================
void VDA5050Master::onboard_agv(
  const std::string& interface_name, const std::string& manufacturer,
  const std::string& serial_number, size_t max_queue_size, bool drop_oldest)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::lock_guard<std::mutex> lock(agv_mutex_);

  if (get_agv_by_id(agv_id))
  {
    VDA5050_WARN("[VDA5050Master] AGV already onboarded: {}", agv_id);
    return;
  }

  // Create AGV instance with a new protocol adapter. Pass weak_from_this()
  // as the back-pointer so the AGV can dispatch incoming messages to the
  // master's virtual callbacks (on_state, on_connection, etc.) while
  // detecting master destruction cleanly via lock().
  auto agv = std::make_shared<AGV>(
    ProtocolAdapter::make(
      mqtt_client_, interface_name, Version, manufacturer, serial_number),
    interface_name, manufacturer, serial_number, max_queue_size, drop_oldest,
    StateHeartbeatInterval, weak_from_this());

  // Subscriptions are wired here (not in AGV's ctor) so that
  // AGV::weak_from_this() is valid — make_shared above has
  // associated the shared_ptr by this point.
  agv->setup_subscriptions();

  agvs_[agv_id] = std::move(agv);

  VDA5050_INFO("[VDA5050Master] Onboarded AGV: {}", agv_id);
}

void VDA5050Master::onboard_agv(
  const std::string& manufacturer, const std::string& serial_number,
  size_t max_queue_size, bool drop_oldest)
{
  onboard_agv(
    DefaultInterfaceName, manufacturer, serial_number, max_queue_size,
    drop_oldest);
}

//=============================================================================
void VDA5050Master::offboard_agv(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    auto it = agvs_.find(agv_id);
    if (it == agvs_.end())
    {
      VDA5050_WARN(
        "[VDA5050Master] Cannot offboard: AGV not found: {}", agv_id);
      return;
    }
    agv = std::move(it->second);
    agvs_.erase(it);
  }

  // Stop AGV after removing from map. The AGV destructor calls
  // protocol_adapter_->unsubscribe<T>() for each subscribed topic, so
  // the broker stops routing to lambdas captured at subscribe time
  // before the AGV instance is gone.
  agv->stop();

  VDA5050_INFO("[VDA5050Master] Offboarded AGV: {}", agv_id);
}

//=============================================================================
bool VDA5050Master::is_agv_onboarded(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(agv_mutex_);
  std::string agv_id = manufacturer + "/" + serial_number;
  return get_agv_by_id(agv_id) != nullptr;
}

//=============================================================================
std::shared_ptr<AGV> VDA5050Master::get_agv(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(agv_mutex_);
  std::string agv_id = manufacturer + "/" + serial_number;
  return get_agv_by_id(agv_id);
}

//=============================================================================
std::shared_ptr<AGV> VDA5050Master::get_agv_by_id(
  const std::string& agv_id) const
{
  // Note: Caller must hold agv_mutex_
  auto it = agvs_.find(agv_id);
  return (it != agvs_.end()) ? it->second : nullptr;
}

//=============================================================================
bool VDA5050Master::publish_order(
  const std::string& manufacturer, const std::string& serial_number,
  const Order& order)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(agv_id);
  }

  if (!agv)
  {
    throw std::runtime_error(
      "Cannot publish order: AGV not onboarded: " + agv_id);
  }

  return agv->send_order(order);
}

//=============================================================================
bool VDA5050Master::publish_instant_actions(
  const std::string& manufacturer, const std::string& serial_number,
  const InstantActions& actions)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(agv_id);
  }

  if (!agv)
  {
    throw std::runtime_error(
      "Cannot publish instant actions: AGV not onboarded: " + agv_id);
  }

  return agv->send_instant_actions(actions);
}

//=============================================================================
void VDA5050Master::on_state(
  const std::string& /*agv_id*/, const State& /*state*/)
{
}

//=============================================================================
void VDA5050Master::on_connection(
  const std::string& /*agv_id*/, const Connection& /*connection*/)
{
}

//=============================================================================
void VDA5050Master::on_factsheet(
  const std::string& /*agv_id*/, const Factsheet& /*factsheet*/)
{
}

//=============================================================================
void VDA5050Master::on_visualization(
  const std::string& /*agv_id*/, const Visualization& /*visualization*/)
{
}

}  // namespace master
}  // namespace vda5050_core
