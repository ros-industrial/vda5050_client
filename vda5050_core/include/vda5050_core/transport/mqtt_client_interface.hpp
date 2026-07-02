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

#ifndef VDA5050_CORE__TRANSPORT__MQTT_CLIENT_INTERFACE_HPP_
#define VDA5050_CORE__TRANSPORT__MQTT_CLIENT_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <string>

namespace vda5050_core {

namespace transport {

class MqttClientInterface
{
public:
  /// \brief Default destructor
  virtual ~MqttClientInterface() = default;

  /// \brief Connect to the MQTT broker
  virtual void connect() = 0;

  /// \brief Disconnect from the the MQTT broker
  virtual void disconnect() = 0;

  /// \brief Check MQTT connection
  virtual bool connected() = 0;

  /// \brief Publish a message to the MQTT broker
  ///
  /// \param topic Topic for publish
  /// \param message Raw message string
  /// \param qos Quality of service setting for the publish
  /// \param retain Flag to retain the mesasge in the broker
  virtual void publish(
    const std::string& topic, const std::string& message, int qos,
    bool retain = false) = 0;

  using MessageHandler =
    std::function<void(const std::string&, const std::string&)>;

  /// \brief Subscribe to a topic to receive a message from the broker
  ///
  /// \param topic Topic for subscription
  /// \param handler Callback to handle the incoming message
  /// \param qos Quality of service setting for the subscription
  virtual void subscribe(
    const std::string& topic, MessageHandler handler, int qos) = 0;

  /// \brief Unsubscribe from a topic
  ///
  /// \param topic Topic to unsubscribe from
  virtual void unsubscribe(const std::string& topic) = 0;

  /// \brief Set a will message for when the client disconnects abruptly
  ///
  /// \param topic Topic to publish will message
  /// \param message Raw message string
  /// \param qos Quality of service setting for the publish
  /// \param retain Flag to retain the mesasge in the broker
  virtual void set_will(
    const std::string& topic, const std::string& message, int qos,
    bool retain = true) = 0;

  using ConnectionStateHandler = std::function<void(const std::string&)>;

  /// \brief Register a handler to be invoked when the underlying transport
  /// reports the broker connection has been lost (Task #70).
  ///
  /// The handler runs on the transport's I/O thread and must be
  /// thread-safe with respect to any state it touches. Setting a new
  /// handler replaces any previously-registered one. Default impl is
  /// a no-op so existing MqttClientInterface implementations continue
  /// to compile without changes.
  virtual void set_connection_lost_callback(ConnectionStateHandler /*handler*/)
  {
  }

  /// \brief Register a handler to be invoked when the underlying transport
  /// reports the broker connection has been (re)established (Task #70).
  ///
  /// Fires on initial connect and on every Paho-driven auto-reconnect.
  /// Same threading + replace-on-set semantics as
  /// set_connection_lost_callback. Default impl is a no-op.
  virtual void set_connected_callback(ConnectionStateHandler /*handler*/) {}
};

/// \brief Create a default shared MQTT client interface
///
/// \param broker_address Address of the MQTT broker
/// \param client_id ID of the MQTT client
///
/// \return Shared pointer to MQTT client
std::shared_ptr<MqttClientInterface> create_default_client_shared(
  const std::string& broker_address, const std::string& client_id);

/// \brief Create a default unique MQTT client interface
///
/// \param broker_address Address of the MQTT broker
/// \param client_id ID of the MQTT client
///
/// \return Unique pointer to MQTT client
std::unique_ptr<MqttClientInterface> create_default_client_unique(
  const std::string& broker_address, const std::string& client_id);

}  // namespace transport
}  // namespace vda5050_core

#endif  // VDA5050_CORE__TRANSPORT__MQTT_CLIENT_INTERFACE_HPP_
