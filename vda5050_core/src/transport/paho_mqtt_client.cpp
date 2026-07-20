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

#include "vda5050_core/transport/paho_mqtt_client.hpp"
#include "vda5050_core/logger/logger.hpp"

namespace vda5050_core {

namespace transport {

//=============================================================================
std::shared_ptr<MqttClientInterface> create_default_client_shared(
  const std::string& broker_address, const std::string& client_id)
{
  return PahoMqttClient::make_shared(broker_address, client_id);
}

//=============================================================================
std::unique_ptr<MqttClientInterface> create_default_client_unique(
  const std::string& broker_address, const std::string& client_id)
{
  return PahoMqttClient::make_unique(broker_address, client_id);
}

//=============================================================================
MqttActionListener::MqttActionListener()
{
  // Nothing to do here ...
}

//=============================================================================
void MqttActionListener::on_failure(const mqtt::token& tok)
{
  VDA5050_ERROR_STREAM(
    "Failed to deliver message with ID: "
    << std::to_string(tok.get_message_id()));
}

//=============================================================================
void MqttActionListener::on_success(const mqtt::token& /*tok*/)
{
  // Nothing to do here ...
}

//=============================================================================
MqttCallback::MqttCallback(PahoMqttClient& parent) : parent_(parent)
{
  // Nothing to do here ...
}

//=============================================================================
void MqttCallback::connected(const std::string& cause)
{
  VDA5050_INFO_STREAM(
    "MQTT client [" << parent_.client_->get_client_id() << "] connected to "
                    << parent_.client_->get_server_uri());
  MqttClientInterface::ConnectionStateHandler handler;
  {
    std::lock_guard<std::mutex> lock(parent_.handler_mutex_);
    handler = parent_.connected_handler_;
  }
  if (handler) handler(cause);
}

//=============================================================================
void MqttCallback::connection_lost(const std::string& cause)
{
  VDA5050_ERROR("MQTT client disconnected. Retrying connection ...");
  MqttClientInterface::ConnectionStateHandler handler;
  {
    std::lock_guard<std::mutex> lock(parent_.handler_mutex_);
    handler = parent_.connection_lost_handler_;
  }
  if (handler) handler(cause);
}

//=============================================================================
void MqttCallback::message_arrived(mqtt::const_message_ptr msg)
{
  std::lock_guard<std::mutex> lock(parent_.handler_mutex_);
  auto it = parent_.handlers_.find(msg->get_topic());
  if (it != parent_.handlers_.end())
  {
    it->second(msg->get_topic(), msg->get_payload());
  }
}

//=============================================================================
void MqttCallback::delivery_complete(mqtt::delivery_token_ptr /*tok*/)
{
  // Nothing to do here ...
}

//=============================================================================
std::shared_ptr<PahoMqttClient> PahoMqttClient::make_shared(
  const std::string& broker_address, const std::string& client_id)
{
  auto paho_client = std::shared_ptr<PahoMqttClient>(
    new PahoMqttClient(broker_address, client_id));
  return paho_client;
}

//=============================================================================
std::unique_ptr<PahoMqttClient> PahoMqttClient::make_unique(
  const std::string& broker_address, const std::string& client_id)
{
  auto paho_client = std::unique_ptr<PahoMqttClient>(
    new PahoMqttClient(broker_address, client_id));
  return paho_client;
}

//=============================================================================
PahoMqttClient::~PahoMqttClient() = default;

//=============================================================================
void PahoMqttClient::connect()
{
  if (client_->is_connected()) return;

  try
  {
    client_->connect(conn_options_, nullptr, action_listener_)->wait();
  }
  catch (const mqtt::exception& e)
  {
    VDA5050_ERROR_STREAM(
      "Unable to establish MQTT connection: " << e.get_message());
  }
}

//=============================================================================
void PahoMqttClient::disconnect()
{
  if (client_->is_connected())
  {
    try
    {
      client_->disconnect()->wait();
      VDA5050_INFO_STREAM(
        "MQTT client disconnected: " << client_->get_client_id());
    }
    catch (const mqtt::exception& e)
    {
      VDA5050_ERROR_STREAM("MQTT disconnection failed: " << e.get_message());
    }
  }
}

//=============================================================================
bool PahoMqttClient::connected()
{
  return client_->is_connected();
}

//=============================================================================
void PahoMqttClient::publish(
  const std::string& topic, const std::string& message, int qos, bool retain)
{
  try
  {
    auto msg = std::make_shared<mqtt::message>();
    msg->set_topic(topic);
    msg->set_payload(message);
    msg->set_qos(qos);
    msg->set_retained(retain);

    client_->publish(msg)->wait();
  }
  catch (const mqtt::exception& e)
  {
    VDA5050_ERROR_STREAM("MQTT publish failed: " << e.get_message());
  }
}

//=============================================================================
void PahoMqttClient::subscribe(
  const std::string& topic, MessageHandler handler, int qos)
{
  try
  {
    client_->subscribe(topic, qos)->wait();
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handlers_[topic] = handler;
  }
  catch (const mqtt::exception& e)
  {
    VDA5050_ERROR_STREAM("MQTT subscription failed: " << e.get_message());
  }
}

//=============================================================================
void PahoMqttClient::unsubscribe(const std::string& topic)
{
  try
  {
    client_->unsubscribe(topic)->wait();
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handlers_.erase(topic);
  }
  catch (const mqtt::exception& e)
  {
    VDA5050_ERROR_STREAM("MQTT unsubscription failed: " << e.get_message());
  }
}

//=============================================================================
void PahoMqttClient::set_will(
  const std::string& topic, const std::string& message, int qos, bool retain)
{
  mqtt::will_options will;
  will.set_topic(topic);
  will.set_retained(retain);
  will.set_qos(qos);
  will.set_payload(message);

  conn_options_.set_will(will);
}

//=============================================================================
void PahoMqttClient::set_connection_lost_callback(
  ConnectionStateHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  connection_lost_handler_ = std::move(handler);
}

//=============================================================================
void PahoMqttClient::set_connected_callback(ConnectionStateHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  connected_handler_ = std::move(handler);
}

//=============================================================================
mqtt::connect_options& PahoMqttClient::connect_options()
{
  return conn_options_;
}

//=============================================================================
PahoMqttClient::PahoMqttClient(
  const std::string& broker_address, const std::string& client_id)
: client_(std::make_unique<mqtt::async_client>(broker_address, client_id)),
  action_listener_(MqttActionListener()),
  callback_(MqttCallback(*this))
{
  client_->set_callback(callback_);

  conn_options_.set_mqtt_version(4);
  conn_options_.set_clean_session(false);
  conn_options_.set_user_name("");
  conn_options_.set_password("");
  conn_options_.set_automatic_reconnect(true);
  conn_options_.set_automatic_reconnect(2, 32);
}

}  // namespace transport
}  // namespace vda5050_core
