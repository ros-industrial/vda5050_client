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

// Client that connects to an MQTT broker and holds the connection open
// indefinitely, until the user presses Ctrl-C (SIGINT). On signal it
// disconnects cleanly and exits.
//
// Usage:
//   mqtt_stay_connected [broker_address] [client_id] [timeout_ms]
//
// Example:
//   mqtt_stay_connected tcp://localhost:1883 stay_connected 3000
//   ros2 run vda5050_core mqtt_stay_connected tcp://localhost:1883 stay_connected 3000

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

#include <vda5050_core/logger/logger.hpp>
#include <vda5050_core/transport/paho_mqtt_client.hpp>

using PahoMqttClient = vda5050_core::transport::PahoMqttClient;

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int signum)
{
  (void)signum;
  g_running = false;
}

}  // namespace

int main(int argc, char** argv)
{
  const std::string broker = (argc > 1) ? argv[1] : "tcp://localhost:1883";
  const std::string client_id = (argc > 2) ? argv[2] : "stay_connected";
  const auto timeout_ms = (argc > 3)
                            ? std::chrono::milliseconds(std::atol(argv[3]))
                            : std::chrono::milliseconds(3000);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  VDA5050_INFO(
    "Creating client id=[{}] broker=[{}] operation_timeout={} ms", client_id,
    broker, timeout_ms.count());

  auto client = PahoMqttClient::make_unique(broker, client_id);
  client->set_operation_timeout(timeout_ms);

  client->connect();
  VDA5050_INFO("connected() == {}", client->connected());

  if (client->connected())
  {
    const std::string topic = "vda5050/stay_connected";
    client->subscribe(
      topic,
      [](const std::string& t, const std::string& payload) {
        VDA5050_INFO("message on [{}]: {}", t, payload);
      },
      1);
    VDA5050_INFO("subscribed to [{}]", topic);
  }

  VDA5050_INFO("Holding connection open. Press Ctrl-C to disconnect and exit.");
  while (g_running)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  VDA5050_INFO("Signal received, disconnecting ...");
  client->disconnect();
  VDA5050_INFO("connected() == {} (after disconnect)", client->connected());
  VDA5050_INFO("done");
  return 0;
}
