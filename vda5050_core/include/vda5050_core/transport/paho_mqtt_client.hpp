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

#ifndef VDA5050_CORE__TRANSPORT__PAHO_MQTT_CLIENT_HPP_
#define VDA5050_CORE__TRANSPORT__PAHO_MQTT_CLIENT_HPP_

#include <mqtt/async_client.h>
#include <mqtt/callback.h>
#include <mqtt/connect_options.h>
#include <mqtt/iaction_listener.h>
#include <mqtt/will_options.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "vda5050_core/transport/mqtt_client_interface.hpp"

namespace vda5050_core {

namespace transport {

class PahoMqttClient;

//=============================================================================
class MqttActionListener : public mqtt::iaction_listener
{
public:
  /// \brief Constructor for MqttActionListener
  MqttActionListener();

protected:
  // Documentation inherited from mqtt::iaction_listener
  void on_failure(const mqtt::token& tok) override;

  // Documentation inherited from mqtt::iaction_listener
  void on_success(const mqtt::token& tok) override;
};

//=============================================================================
class MqttCallback : public mqtt::callback
{
public:
  /// \brief Constructor for MqttCallback
  explicit MqttCallback(PahoMqttClient& parent);

protected:
  // Documentation inherited from mqtt::callback
  void connected(const std::string& cause) override;

  // Documentation inherited from mqtt::callback
  void connection_lost(const std::string& cause) override;

  // Documentation inherited from mqtt::callback
  void message_arrived(mqtt::const_message_ptr msg) override;

  // Documentation inherited from mqtt::callback
  void delivery_complete(mqtt::delivery_token_ptr tok) override;

private:
  /// \brief Reference to the MQTT client interface
  PahoMqttClient& parent_;
};

//=============================================================================
class PahoMqttClient : public MqttClientInterface
{
public:
  /// \brief Create a shared pointer to PahoMqttClient
  ///
  /// \param broker_address Address of the MQTT broker
  /// \param client_id ID of the MQTT client
  ///
  /// \return Shared pointer to Paho MQTT client
  static std::shared_ptr<PahoMqttClient> make_shared(
    const std::string& broker_address, const std::string& client_id);

  /// \brief Create a unique pointer to PahoMqttClient
  ///
  /// \param broker_address Address of the MQTT broker
  /// \param client_id ID of the MQTT client
  ///
  /// \return Unique pointer to Paho MQTT client
  static std::unique_ptr<PahoMqttClient> make_unique(
    const std::string& broker_address, const std::string& client_id);

  /// \brief Destructor for PahoMqttClient
  ~PahoMqttClient();

  PahoMqttClient(const PahoMqttClient&) = delete;
  PahoMqttClient& operator=(const PahoMqttClient&) = delete;
  PahoMqttClient(PahoMqttClient&&) = delete;
  PahoMqttClient& operator=(PahoMqttClient&&) = delete;

  // Documentation inherited from MqttClientInterface
  void connect() override;

  // Documentation inherited from MqttClientInterface
  void disconnect() override;

  // Documentation inherited from MqttClientInterface
  bool connected() override;

  // Documentation inherited from MqttClientInterface
  void publish(
    const std::string& topic, const std::string& message, int qos,
    bool retain = false) override;

  // Documentation inherited from MqttClientInterface
  void subscribe(
    const std::string& topic, MessageHandler handler, int qos) override;

  // Documentation inherited from MqttClientInterface
  void unsubscribe(const std::string& topic) override;

  // Documentation inherited from MqttClientInterface
  void set_will(
    const std::string& topic, const std::string& message, int qos,
    bool retain = true) override;

  // Documentation inherited from MqttClientInterface
  void set_connection_lost_callback(ConnectionStateHandler handler) override;

  // Documentation inherited from MqttClientInterface
  void set_connected_callback(ConnectionStateHandler handler) override;

  /// \brief Get a mutable reference to Paho configuration options
  ///
  /// \return Mutable reference to Paho configuration options
  mqtt::connect_options& connect_options();

  friend class MqttCallback;

private:
  /// \brief Private constructor for PahoMqttClient
  ///
  /// \param broker_address Address of the MQTT broker
  /// \param client_id ID of the MQTT client
  PahoMqttClient(
    const std::string& broker_address, const std::string& client_id);

  /// \brief Unique pointer to Paho async client
  std::unique_ptr<mqtt::async_client> client_;

  /// \brief Implementation of action listener
  MqttActionListener action_listener_;

  /// \brief Implementation of callback
  MqttCallback callback_;

  /// \brief List of message handlers mapped to topics
  std::unordered_map<std::string, MessageHandler> handlers_;

  /// \brief Mutex protecting list of message handlers
  std::mutex handler_mutex_;

  /// \brief Optional handler invoked when broker connection is lost
  /// (Task #70). Stored under handler_mutex_.
  ConnectionStateHandler connection_lost_handler_;

  /// \brief Optional handler invoked when broker connection is
  /// (re)established (Task #70). Stored under handler_mutex_.
  ConnectionStateHandler connected_handler_;

  /// \brief MQTT connection options
  mqtt::connect_options conn_options_;
};

}  // namespace transport
}  // namespace vda5050_core

#endif  // VDA5050_CORE__TRANSPORT__PAHO_MQTT_CLIENT_HPP_
