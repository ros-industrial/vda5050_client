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

#ifndef CLIENT__ADAPTER_TEST_FIXTURE_HPP_
#define CLIENT__ADAPTER_TEST_FIXTURE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "vda5050_core/client/adapter/adapter.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"

#include "mock_mqtt_client.hpp"

using namespace vda5050_core::client::adapter;  // NOLINT
using namespace vda5050_core::execution;        // NOLINT
using namespace vda5050_core::transport;        // NOLINT
using namespace vda5050_core::types;            // NOLINT
using namespace testing;                        // NOLINT

class AdapterTest : public testing::Test
{
protected:
  void SetUp() override
  {
    mqtt = std::make_shared<NiceMock<MockMqttClient>>();

    ON_CALL(*mqtt, connected()).WillByDefault(Return(true));

    ON_CALL(*mqtt, subscribe(_, _, _))
      .WillByDefault([this](
                       const std::string& topic,
                       MqttClientInterface::MessageHandler handler,
                       int /*qos*/) {
        std::lock_guard<std::mutex> lock(subscriptions_mutex);
        subscriptions[topic] = std::move(handler);
      });

    ON_CALL(*mqtt, publish(_, _, _, _))
      .WillByDefault([this](
                       const std::string& topic, const std::string& message,
                       int /*qos*/, bool /*retain*/) {
        std::lock_guard<std::mutex> lock(publish_mutex);
        published.push_back({topic, message});
      });

    protocol_adapter =
      ProtocolAdapter::make(mqtt, "uagv", "2.0.0", "Robot", "001");

    adapter = Adapter::make(protocol_adapter);
  }

  bool wait_publish(size_t publish_count)
  {
    while (true)
    {
      {
        std::lock_guard<std::mutex> lock(publish_mutex);
        if (published.size() >= publish_count) return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  Order make_order(
    const std::string& order_id, uint32_t order_update_id, uint32_t released,
    uint32_t unreleased)
  {
    Order order;
    order.order_id = order_id;
    order.order_update_id = order_update_id;

    std::vector<Node> nodes;
    Node node;
    node.released = true;

    std::vector<Edge> edges;
    Edge edge;
    edge.released = true;
    for (uint32_t i = 0; i < released; i++)
    {
      int inner_seq = i * 2;
      node.node_id = "N" + std::to_string(inner_seq);
      node.sequence_id = inner_seq;
      nodes.push_back(node);

      if ((inner_seq - 1) > 0)
      {
        edge.edge_id = "E" + std::to_string(inner_seq - 1);
        edge.sequence_id = inner_seq - 1;
        edge.start_node_id = "N" + std::to_string(inner_seq - 2);
        edge.end_node_id = "N" + std::to_string(inner_seq);
        edges.push_back(edge);
      }
    }

    node.released = false;
    edge.released = false;
    for (uint32_t i = released; i < released + unreleased; i++)
    {
      uint32_t inner_seq = i * 2;
      node.node_id = "N" + std::to_string(inner_seq);
      node.sequence_id = inner_seq;
      nodes.push_back(node);

      if (inner_seq - 1 > 0)
      {
        edge.edge_id = "E" + std::to_string(inner_seq - 1);
        edge.sequence_id = inner_seq - 1;
        edge.start_node_id = "N" + std::to_string(inner_seq - 2);
        edge.end_node_id = "N" + std::to_string(inner_seq);
        edges.push_back(edge);
      }
    }

    order.nodes = std::move(nodes);
    order.edges = std::move(edges);

    return order;
  }

  vda5050_core::types::Action make_action(
    const std::string& action_id, const std::string& action_type)
  {
    vda5050_core::types::Action action;
    action.action_id = action_id;
    action.action_type = action_type;
    action.blocking_type = BlockingType::NONE;

    return action;
  }

  bool wait_until(std::function<bool()> predicate)
  {
    while (!predicate())
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return true;
  }

  void inject_message(const std::string& topic, const std::string& message)
  {
    MqttClientInterface::MessageHandler handler;
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex);
      auto it = subscriptions.find(topic);
      if (it != subscriptions.end())
      {
        handler = it->second;
      }
    }
    if (handler)
    {
      handler(topic, message);
    }
  }

  struct PublishedMessage
  {
    std::string topic;
    std::string message;
  };

  std::shared_ptr<NiceMock<MockMqttClient>> mqtt;
  std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_adapter;
  std::shared_ptr<Adapter> adapter;

  std::mutex subscriptions_mutex;
  std::unordered_map<std::string, MqttClientInterface::MessageHandler>
    subscriptions;

  std::mutex publish_mutex;
  std::vector<PublishedMessage> published;
};

#endif  // CLIENT__ADAPTER_TEST_FIXTURE_HPP_
