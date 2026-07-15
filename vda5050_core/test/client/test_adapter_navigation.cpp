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

#include <gmock/gmock.h>

#include <chrono>
#include <cstdint>
#include <thread>

#include "adapter_test_fixture.hpp"

using namespace vda5050_core::types;  // NOLINT

class AdapterNavigationTest : public AdapterTest
{
protected:
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

  bool wait_until(std::function<bool()> predicate)
  {
    while (!predicate())
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return true;
  }

  void inject_message(const std::string& topic, const std::string& message)
  {
    subscriptions.at(topic)(topic, message);
  }
};

TEST_F(AdapterNavigationTest, ReceivesReleasedNode)
{
  std::atomic_int call_count = 0;
  std::optional<NodeRequest> n_request;
  std::optional<EdgeRequest> e_request;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> edge_request,
                         std::shared_ptr<OrderExecution> /*execution*/) {
    call_count++;
    n_request = std::move(node_request);
    e_request = std::move(edge_request);
  });

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return call_count == 1; }));

  ASSERT_TRUE(n_request.has_value());
  EXPECT_EQ(n_request->node_id(), "N0");
  EXPECT_EQ(n_request->sequence_id(), 0);

  EXPECT_FALSE(e_request.has_value());

  adapter->stop();
}

TEST_F(AdapterNavigationTest, ReceivesReleasedNodeWithEdge)
{
  std::atomic_int call_count = 0;
  std::optional<NodeRequest> n_request;
  std::optional<EdgeRequest> e_request;
  std::shared_ptr<OrderExecution> order_execution;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> edge_request,
                         std::shared_ptr<OrderExecution> execution) {
    call_count++;
    n_request = std::move(node_request);
    e_request = std::move(edge_request);
    order_execution = std::move(execution);
  });

  adapter->start();

  auto order = make_order("order_id", 0, 2, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return call_count == 1; }));

  ASSERT_TRUE(n_request.has_value());
  EXPECT_EQ(n_request->node_id(), "N0");
  EXPECT_EQ(n_request->sequence_id(), 0);

  order_execution->finished();

  ASSERT_TRUE(wait_until([&] { return call_count == 2; }));

  ASSERT_TRUE(e_request.has_value());
  EXPECT_EQ(e_request->edge_id(), "E1");
  EXPECT_EQ(e_request->sequence_id(), 1);

  ASSERT_TRUE(n_request.has_value());
  EXPECT_EQ(n_request->node_id(), "N2");
  EXPECT_EQ(n_request->sequence_id(), 2);

  adapter->stop();
}

TEST_F(AdapterNavigationTest, UnreleasedNodeDoesNotDispatch)
{
  std::atomic_int call_count = 0;

  adapter->on_navigate([&](
                         NodeRequest /*node_request*/,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    execution->finished();
    call_count++;
  });

  adapter->start();

  auto order = make_order("order_id", 0, 1, 1);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(call_count, 1);

  adapter->stop();
}

TEST_F(AdapterNavigationTest, FinishUpdatesState)
{
  std::optional<NodeRequest> n_request;
  std::shared_ptr<OrderExecution> order_execution;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    n_request = std::move(node_request);
    order_execution = std::move(execution);
  });

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_publish(2));
  auto initial_state =
    nlohmann::json::parse(published.back().message).get<State>();

  EXPECT_EQ(initial_state.last_node_id, "");
  EXPECT_EQ(initial_state.last_node_sequence_id, 0);

  ASSERT_TRUE(wait_until([&] { return order_execution != nullptr; }));

  order_execution->finished();

  ASSERT_TRUE(wait_publish(3));

  auto final_state =
    nlohmann::json::parse(published.back().message).get<State>();

  EXPECT_EQ(final_state.last_node_id, n_request->node_id());
  EXPECT_EQ(final_state.last_node_sequence_id, n_request->sequence_id());

  adapter->stop();
}

TEST_F(AdapterNavigationTest, FailureAddsError)
{
  adapter->on_navigate([&](
                         NodeRequest /*node_request*/,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    execution->failed("navigation failed");
  });

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_publish(3));
  auto state = nlohmann::json::parse(published.back().message).get<State>();

  ASSERT_FALSE(state.errors.empty());
  EXPECT_EQ(state.errors.front().error_description, "navigation failed");

  adapter->stop();
}

TEST_F(AdapterNavigationTest, ContinuesToNextNode)
{
  std::vector<uint32_t> visited;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    visited.push_back(node_request.sequence_id());
    execution->finished();
  });

  adapter->start();

  auto order = make_order("order_id", 0, 2, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return visited.size() == 2; }));

  EXPECT_EQ(visited[0], 0);
  EXPECT_EQ(visited[1], 2);

  adapter->stop();
}

TEST_F(AdapterNavigationTest, NavigationExceptionHandled)
{
  adapter->on_navigate([&](
                         NodeRequest /*node_request*/,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> /*execution*/) {
    throw std::runtime_error("failure");
  });

  adapter->start();

  auto order = make_order("order_id", 0, 2, 0);
  EXPECT_NO_THROW(inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump()));

  adapter->stop();
}
