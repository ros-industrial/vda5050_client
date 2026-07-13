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
void MqttCallback::connected(const std::string& /*cause*/)
{
  VDA5050_INFO_STREAM(
    "MQTT client [" << parent_.client_->get_client_id() << "] connected to "
                    << parent_.client_->get_server_uri());
}

//=============================================================================
void MqttCallback::connection_lost(const std::string& /*cause*/)
{
  VDA5050_ERROR("MQTT client disconnected. Retrying connection ...");
}

//=============================================================================
void MqttCallback::message_arrived(mqtt::const_message_ptr msg)
{
  // Copy the handler out and release the lock before invoking it, so a handler
  // that calls back into subscribe()/unsubscribe() cannot self-deadlock on
  // handler_mutex_.
  MqttClientInterface::MessageHandler handler;
  {
    std::lock_guard<std::mutex> lock(parent_.handler_mutex_);
    auto it = parent_.handlers_.find(msg->get_topic());
    if (it == parent_.handlers_.end()) return;
    handler = it->second;
  }
  handler(msg->get_topic(), msg->get_payload());
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
void PahoMqttClient::set_operation_timeout(std::chrono::milliseconds timeout)
{
  operation_timeout_ = timeout;
}

//=============================================================================
bool PahoMqttClient::wait_for_token(
  mqtt::token_ptr token, const char* operation)
{
  if (!token) return true;

  if (token->wait_for(operation_timeout_)) return true;

  VDA5050_WARN(
    "MQTT {} timed out after {} ms for client [{}]", operation,
    operation_timeout_.count(), client_->get_client_id());
  return false;
}

//=============================================================================
void PahoMqttClient::connect()
{
  if (client_->is_connected()) return;

  try
  {
    wait_for_token(
      client_->connect(conn_options_, nullptr, action_listener_), "connect");
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
  if (!client_->is_connected()) return;

  try
  {
    // Note: do not mutate conn_options_ here. Disabling automatic reconnect on
    // the persistent member has no effect on the already-established session
    // (Paho captured the options at connect time and stops reconnecting on an
    // explicit disconnect anyway), but it would corrupt the options reused by
    // the next connect(), permanently disabling reconnection.
    const auto timeout_ms = static_cast<int>(operation_timeout_.count());
    mqtt::disconnect_options options(timeout_ms);

    if (wait_for_token(client_->disconnect(options), "disconnect"))
    {
      VDA5050_INFO_STREAM(
        "MQTT client disconnected: " << client_->get_client_id());
    }
  }
  catch (const mqtt::exception& e)
  {
    VDA5050_ERROR_STREAM("MQTT disconnection failed: " << e.get_message());
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

    wait_for_token(client_->publish(msg), "publish");
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
  // Register the handler before subscribing so a message arriving between the
  // broker's SUBACK and handler registration is not dropped.
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handlers_[topic] = handler;
  }

  try
  {
    if (!wait_for_token(client_->subscribe(topic, qos), "subscribe"))
    {
      // Subscription did not complete; roll back so local state stays
      // consistent with the broker.
      std::lock_guard<std::mutex> lock(handler_mutex_);
      handlers_.erase(topic);
    }
  }
  catch (const mqtt::exception& e)
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handlers_.erase(topic);
    VDA5050_ERROR_STREAM("MQTT subscription failed: " << e.get_message());
  }
}

//=============================================================================
void PahoMqttClient::unsubscribe(const std::string& topic)
{
  try
  {
    // Only drop the local handler once the broker confirms the unsubscribe;
    // otherwise the broker keeps delivering and we would have no handler for
    // incoming messages.
    if (wait_for_token(client_->unsubscribe(topic), "unsubscribe"))
    {
      std::lock_guard<std::mutex> lock(handler_mutex_);
      handlers_.erase(topic);
    }
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
