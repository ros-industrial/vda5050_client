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

#ifndef MASTER__TEST_FIXTURE_COMMUNICATION_MQTT_HPP_
#define MASTER__TEST_FIXTURE_COMMUNICATION_MQTT_HPP_

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

namespace vda5050_core::master::test::mqtt {

// Test constants
namespace constants {
// Increased poll interval for network latency with public brokers
constexpr auto MQTT_POLL_INTERVAL = std::chrono::milliseconds(500);

// Timeout for broker availability check
constexpr auto BROKER_CHECK_TIMEOUT = std::chrono::seconds(5);

// Allow overriding MQTT broker via environment variable
// Default to localhost:1883 for local mosquitto broker
// Use localhost:1884 for automated Docker test broker by setting env var
// NOLINTNEXTLINE(runtime/string)
inline std::string get_mqtt_broker()
{
  const char* env_broker = std::getenv("VDA5050_TEST_MQTT_BROKER");
  if (env_broker != nullptr)
  {
    return std::string(env_broker);
  }
  return "tcp://localhost:1883";
}

// NOLINTNEXTLINE(runtime/string)
const std::string MQTT_BROKER = get_mqtt_broker();

// Check if MQTT broker is available
// Returns true if connection succeeds, false otherwise
inline bool is_broker_available()
{
  static bool checked = false;
  static bool available = false;

  // Cache result to avoid repeated connection attempts
  if (checked)
  {
    return available;
  }

  checked = true;

  try
  {
    auto client = vda5050_core::transport::create_default_client_shared(
      MQTT_BROKER, "availability_check_client");
    client->connect();
    client->disconnect();
    // Small delay to allow Paho MQTT library to complete cleanup
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    available = true;
  }
  catch (...)
  {
    available = false;
  }

  return available;
}
}  // namespace constants
}  // namespace vda5050_core::master::test::mqtt

#endif  // MASTER__TEST_FIXTURE_COMMUNICATION_MQTT_HPP_
