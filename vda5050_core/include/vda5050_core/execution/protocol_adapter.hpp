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

#ifndef VDA5050_CORE__EXECUTION__PROTOCOL_ADAPTER_HPP_
#define VDA5050_CORE__EXECUTION__PROTOCOL_ADAPTER_HPP_

#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/json_utils/serialization.hpp"

#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

namespace vda5050_core {

namespace execution {

template <typename T>
struct is_valid_message : std::false_type
{};

template <>
struct is_valid_message<vda5050_core::types::Connection> : std::true_type
{};

template <>
struct is_valid_message<vda5050_core::types::State> : std::true_type
{};

template <>
struct is_valid_message<vda5050_core::types::Order> : std::true_type
{};

template <>
struct is_valid_message<vda5050_core::types::InstantActions> : std::true_type
{};

template <>
struct is_valid_message<vda5050_core::types::Factsheet> : std::true_type
{};

template <>
struct is_valid_message<vda5050_core::types::Visualization> : std::true_type
{};

template <typename T>
inline constexpr bool is_valid_message_v = is_valid_message<T>::value;

class ProtocolAdapter : public std::enable_shared_from_this<ProtocolAdapter>
{
public:
  static std::shared_ptr<ProtocolAdapter> make(
    std::shared_ptr<transport::MqttClientInterface> mqtt_client,
    const std::string& interface, const std::string& version,
    const std::string& manufacturer, const std::string& serial_number);

  void connect();

  void disconnect();

  bool connected();

  template <typename MessageT>
  void publish(MessageT message, int qos, bool retained = false)
  {
    static_assert(
      is_valid_message_v<MessageT>, "Type is not supported in ProtocolAdapter");

    auto type_idx = std::type_index(typeid(MessageT));

    auto it = topic_names_.find(type_idx);
    if (it == topic_names_.end()) return;

    try
    {
      vda5050_core::types::Header header{
        header_ids_[type_idx]++, std::chrono::system_clock::now(), version_,
        manufacturer_, serial_number_};
      message.header = header;

      nlohmann::json j = message;

      if (mqtt_client_)
        mqtt_client_->publish(it->second, j.dump(), qos, retained);
    }
    catch (const nlohmann::json::exception& e)
    {
      VDA5050_ERROR(
        "Serialization failed for message to be published on {}: {}",
        it->second, e.what());
    }
    catch (const std::exception& e)
    {
      VDA5050_ERROR(
        "Unexpected error during publish to {}: {}", it->second, e.what());
    }
  }

  template <typename MessageT>
  void subscribe(
    std::function<void(MessageT, std::optional<vda5050_core::types::Error>)>
      callback,
    int qos)
  {
    static_assert(
      is_valid_message_v<MessageT>, "Type is not supported in ProtocolAdapter");

    auto type_idx = std::type_index(typeid(MessageT));

    auto it = topic_names_.find(type_idx);
    if (it == topic_names_.end()) return;

    // Per-subscription "active" flag captured by the wrapper so any
    // in-flight or post-unsubscribe dispatch becomes a no-op at the
    // adapter layer. Stored in active_flags_ so unsubscribe<T>() can
    // flip it. If a previous subscribe<T>() is still active for this
    // type, deactivate it (newer wins).
    auto active = std::make_shared<std::atomic<bool>>(true);
    {
      std::lock_guard<std::mutex> lock(active_flags_mutex_);
      auto prev = active_flags_.find(type_idx);
      if (prev != active_flags_.end())
      {
        *(prev->second) = false;
      }
      active_flags_[type_idx] = active;
    }

    auto wrapper = [callback, active](
                     const std::string& topic, const std::string& payload) {
      if (!*active) return;

      try
      {
        MessageT message = nlohmann::json::parse(payload);

        callback(message, std::nullopt);
      }
      catch (const nlohmann::json::exception& e)
      {
        vda5050_core::types::Error error;
        error.error_type = "JSON_DESERIALIZATION_ERROR";
        error.error_description = fmt::format(
          "Failed to parse JSOn recevied on topic {}: {}", topic, e.what());
        error.error_level = vda5050_core::types::ErrorLevel::FATAL;
        callback(MessageT{}, error);
      }
      catch (const std::exception& e)
      {
        VDA5050_ERROR(
          "Unexpected error during message parsing on topic {}: {}", topic,
          e.what());
      }
    };

    if (mqtt_client_) mqtt_client_->subscribe(it->second, wrapper, qos);
  }

  template <typename MessageT>
  void unsubscribe()
  {
    static_assert(
      is_valid_message_v<MessageT>, "Type is not supported in ProtocolAdapter");

    auto type_idx = std::type_index(typeid(MessageT));

    // Deactivate the wrapper's flag so any in-flight or
    // post-unsubscribe dispatch becomes a no-op at the adapter layer.
    {
      std::lock_guard<std::mutex> lock(active_flags_mutex_);
      auto flag = active_flags_.find(type_idx);
      if (flag != active_flags_.end())
      {
        *(flag->second) = false;
        active_flags_.erase(flag);
      }
    }

    auto it = topic_names_.find(type_idx);
    if (it == topic_names_.end()) return;

    if (mqtt_client_) mqtt_client_->unsubscribe(it->second);
  }

  static std::string get_topic_version(const std::string& version)
  {
    // TODO(sauk2): Enforce stricter version checking before parsing string
    auto position = version.find('.');
    std::string major = version.substr(0, position);
    return "v" + major;
  }

  void unsubscribe_all();

private:
  ProtocolAdapter(
    std::shared_ptr<transport::MqttClientInterface> mqtt_client,
    const std::string& interface, const std::string& version,
    const std::string& manufacturer, const std::string& serial_number);

  std::shared_ptr<transport::MqttClientInterface> mqtt_client_;

  std::unordered_map<std::type_index, std::string> topic_names_;
  std::unordered_map<std::type_index, uint32_t> header_ids_;

  // Per-type "active" flags. Captured by the wrapper installed on
  // mqtt_client; the wrapper checks the flag before dispatching to
  // the user callback. unsubscribe<T>() flips the flag to false so
  // any in-flight or post-unsubscribe messages are silently dropped
  // at the adapter — the user callback won't fire after
  // unsubscribe<T>() returns.
  std::unordered_map<std::type_index, std::shared_ptr<std::atomic<bool>>>
    active_flags_;
  std::mutex active_flags_mutex_;

  std::string interface_;
  std::string version_;
  std::string manufacturer_;
  std::string serial_number_;
};

}  // namespace execution

}  // namespace vda5050_core

#endif  // VDA5050_CORE__EXECUTION__PROTOCOL_ADAPTER_HPP_
