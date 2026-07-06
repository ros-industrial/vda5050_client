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

#include <memory>
#include <string>
#include <typeindex>
#include <vector>

#include <vda5050_core/logger/logger.hpp>

#include "vda5050_core/execution/base.hpp"

using Base = vda5050_core::execution::Base;
using EventBase = vda5050_core::execution::EventBase;
using UpdateBase = vda5050_core::execution::UpdateBase;
using ResourceBase = vda5050_core::execution::ResourceBase;

struct MoveCommand
: public vda5050_core::execution::Initialize<MoveCommand, EventBase>
{
  double velocity;

  explicit MoveCommand(double velocity) : velocity(velocity)
  {
    // Nothing to do here ...
  }
};

struct OrderUpdate
: public vda5050_core::execution::Initialize<OrderUpdate, UpdateBase>
{
  std::string order_id;
  uint32_t sequence_id;

  OrderUpdate(const std::string& order_id, uint32_t sequence_id)
  : order_id(order_id), sequence_id(sequence_id)
  {
    // Nothing to do here ...
  }
};

struct ClientConfig
: public vda5050_core::execution::Initialize<ClientConfig, ResourceBase>
{
  std::string server_uri;
  uint32_t port;
  std::string username;
  std::string password;

  ClientConfig(
    const std::string& server_uri, uint32_t port, const std::string& username,
    const std::string& password)
  : server_uri(std::move(server_uri)),
    port(port),
    username(std::move(username)),
    password(std::move(password))
  {
    // Nothing to do here ...
  }
};

int main()
{
  std::vector<std::shared_ptr<Base>> registry;

  registry.push_back(std::make_shared<MoveCommand>(1.5));
  registry.push_back(std::make_shared<OrderUpdate>("order_1", 2));
  registry.push_back(
    std::make_shared<ClientConfig>("localhost", 8080, "user_1", "pass"));

  VDA5050_INFO("Starting Type Index Verification ...");

  for (const auto& item : registry)
  {
    std::type_index idx = item->get_type();

    if (idx == std::type_index(typeid(MoveCommand)))
    {
      auto event = std::static_pointer_cast<MoveCommand>(item);
      VDA5050_INFO_STREAM(
        "[Event] [MoveCommand] velocity: " << event->velocity);
    }
    else if (idx == std::type_index(typeid(OrderUpdate)))
    {
      auto update = std::static_pointer_cast<OrderUpdate>(item);
      VDA5050_INFO_STREAM(
        "[Update] [OrderUpdate] id: " << update->order_id << ", sequence_id: "
                                      << update->sequence_id);
    }
    else if (idx == std::type_index(typeid(ClientConfig)))
    {
      auto resource = std::static_pointer_cast<ClientConfig>(item);
      VDA5050_INFO_STREAM(
        "[Resource] [ClientConfig] server_uri: "
        << resource->server_uri << ", port: " << resource->port
        << ", username: " << resource->username
        << ", password: " << resource->password);
    }
  }

  return 0;
}
