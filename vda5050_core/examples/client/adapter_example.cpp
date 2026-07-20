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

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/client/adapter/action_execution.hpp"
#include "vda5050_core/client/adapter/action_request.hpp"
#include "vda5050_core/client/adapter/adapter.hpp"
#include "vda5050_core/client/adapter/edge_request.hpp"
#include "vda5050_core/client/adapter/localization_request.hpp"
#include "vda5050_core/client/adapter/node_request.hpp"
#include "vda5050_core/client/adapter/order_execution.hpp"

using vda5050_core::client::adapter::ActionExecution;
using vda5050_core::client::adapter::ActionRequest;
using vda5050_core::client::adapter::Adapter;
using vda5050_core::client::adapter::EdgeRequest;
using vda5050_core::client::adapter::LocalizationRequest;
using vda5050_core::client::adapter::NodeRequest;
using vda5050_core::client::adapter::OrderExecution;
using vda5050_core::execution::ProtocolAdapter;

std::atomic_bool running{true};

void signal_handler(int signal)
{
  VDA5050_INFO_STREAM(
    "System Signal [" << signal << "] received. Shutting down ...");
  running = false;
}

int main()
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  auto mqtt_client = vda5050_core::transport::create_default_client_unique(
    "tcp://localhost:1883", "adapter_example");

  auto protocol_adapter = ProtocolAdapter::make(
    std::move(mqtt_client), "uagv", "2.0.0", "Manufacturer", "S001");

  auto adapter = Adapter::make(protocol_adapter);

  auto state_manager = adapter->state_manager();

  adapter->on_navigate([state_manager](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    VDA5050_INFO_STREAM(
      "Navigating to node [" << node_request.node_id() << "]");

    std::thread([node_request, execution, state_manager]() {
      state_manager->set_driving(true);

      std::this_thread::sleep_for(std::chrono::seconds(2));

      state_manager->set_driving(false);

      execution->finished();

      VDA5050_INFO_STREAM("Reached node [" << node_request.node_id() << "]");

      auto position = node_request.node_position();
      if (position.has_value())
      {
        state_manager->set_position(
          position.value().x, position.value().y,
          position.value().theta.value_or(0.0), position.value().map_id);
      }
    }).detach();
  });

  adapter->on_action(
    [state_manager](
      ActionRequest request, std::shared_ptr<ActionExecution> execution) {
      VDA5050_INFO_STREAM(
        "Action of type [" << request.action_type() << "] requested");

      execution->running();

      std::this_thread::sleep_for(std::chrono::seconds(1));

      execution->finished();
    });

  adapter->on_localize(
    [state_manager](
      LocalizationRequest request, std::shared_ptr<ActionExecution> execution) {
      VDA5050_INFO(
        "Received localization request with point [{}, {}, {}] on map: {}",
        request.x(), request.y(), request.theta(), request.map_id());

      state_manager->initialize_position(
        request.x(), request.y(), request.theta(), request.map_id());

      execution->finished();
    });

  adapter->start();
  VDA5050_INFO("Adapter started ...");

  while (running)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  adapter->stop();

  return 0;
}
