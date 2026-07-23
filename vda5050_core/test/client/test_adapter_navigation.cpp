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

class AdapterNavigationTest : public AdapterTest
{};

TEST_F(AdapterNavigationTest, ReceivesReleasedNode)
{
  std::atomic_int call_count = 0;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> edge_request,
                         std::shared_ptr<OrderExecution> execution) {
    call_count++;

    EXPECT_EQ(node_request.node_id(), "N0");
    EXPECT_EQ(node_request.sequence_id(), 0);
    EXPECT_FALSE(edge_request.has_value());

    execution->finished();
  });

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return call_count == 1; }));

  adapter->stop();
}

TEST_F(AdapterNavigationTest, ReceivesReleasedNodeWithEdge)
{
  std::atomic_int call_count = 0;
  std::mutex nav_mutex;
  std::optional<NodeRequest> n_request;
  std::optional<EdgeRequest> e_request;
  std::shared_ptr<OrderExecution> order_execution;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> edge_request,
                         std::shared_ptr<OrderExecution> execution) {
    call_count++;

    std::lock_guard<std::mutex> lock(nav_mutex);
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

  {
    std::lock_guard<std::mutex> lock(nav_mutex);

    ASSERT_TRUE(n_request.has_value());
    EXPECT_EQ(n_request->node_id(), "N0");
    EXPECT_EQ(n_request->sequence_id(), 0);

    order_execution->finished();
  }

  ASSERT_TRUE(wait_until([&] { return call_count == 2; }));

  {
    std::lock_guard<std::mutex> lock(nav_mutex);

    ASSERT_TRUE(e_request.has_value());
    EXPECT_EQ(e_request->edge_id(), "E1");
    EXPECT_EQ(e_request->sequence_id(), 1);

    ASSERT_TRUE(n_request.has_value());
    EXPECT_EQ(n_request->node_id(), "N2");
    EXPECT_EQ(n_request->sequence_id(), 2);
  }

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
  std::mutex mutex;
  std::optional<NodeRequest> n_request;
  std::shared_ptr<OrderExecution> order_execution;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    std::lock_guard<std::mutex> lock(mutex);
    n_request = std::move(node_request);
    order_execution = std::move(execution);
  });

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_publish(2));

  State initial_state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    initial_state =
      nlohmann::json::parse(published.back().message).get<State>();
  }

  EXPECT_EQ(initial_state.last_node_id, "");
  EXPECT_EQ(initial_state.last_node_sequence_id, 0);

  {
    std::lock_guard<std::mutex> lock(mutex);

    ASSERT_NE(order_execution, nullptr);
    order_execution->finished();
  }

  ASSERT_TRUE(wait_publish(3));

  State final_state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    final_state = nlohmann::json::parse(published.back().message).get<State>();
  }

  {
    std::lock_guard<std::mutex> lock(mutex);

    EXPECT_EQ(final_state.last_node_id, n_request->node_id());
    EXPECT_EQ(final_state.last_node_sequence_id, n_request->sequence_id());
  }

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

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  ASSERT_FALSE(state.errors.empty());
  EXPECT_EQ(state.errors.front().error_description, "navigation failed");

  adapter->stop();
}

TEST_F(AdapterNavigationTest, ContinuesToNextNode)
{
  std::mutex mutex;
  std::vector<uint32_t> visited;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    std::lock_guard<std::mutex> lock(mutex);
    visited.push_back(node_request.sequence_id());
    execution->finished();
  });

  adapter->start();

  auto order = make_order("order_id", 0, 2, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return visited.size() == 2;
  }));

  std::lock_guard<std::mutex> lock(mutex);
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
TEST_F(AdapterNavigationTest, TransformsCoordinatesUsingInitializePosition)
{
  std::mutex mutex;
  std::optional<NodeRequest> received_request;
  std::atomic_bool nav_called = false;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    std::lock_guard<std::mutex> lock(mutex);
    received_request = std::move(node_request);
    nav_called = true;
    execution->finished();
  });

  const std::string map_id = "floor_1";

  adapter->state_manager()->initialize_position(10.0, 5.0, M_PI_2, map_id);

  EXPECT_TRUE(adapter->state_manager()->position_initialized());

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  order.nodes[0].node_position = vda5050_core::types::NodePosition{};
  order.nodes[0].node_position->map_id = map_id;
  order.nodes[0].node_position->x = 10.0;
  order.nodes[0].node_position->y = 15.0;
  order.nodes[0].node_position->theta = M_PI_2;

  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return nav_called.load(); }));

  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_TRUE(received_request.has_value());
    const auto& pos = received_request->node_position();
    ASSERT_TRUE(pos.has_value());

    EXPECT_DOUBLE_EQ(pos->x, 10.0);
    EXPECT_DOUBLE_EQ(pos->y, 15.0);
    ASSERT_TRUE(pos->theta.has_value());
    EXPECT_DOUBLE_EQ(pos->theta.value(), M_PI_2);
  }

  adapter->stop();
}

TEST_F(AdapterNavigationTest, TransformsCoordinatesUsingSetTransform)
{
  std::mutex mutex;
  std::optional<NodeRequest> received_request;
  std::atomic_bool nav_called = false;

  adapter->on_navigate([&](
                         NodeRequest node_request,
                         std::optional<EdgeRequest> /*edge_request*/,
                         std::shared_ptr<OrderExecution> execution) {
    std::lock_guard<std::mutex> lock(mutex);
    received_request = std::move(node_request);
    nav_called = true;
    execution->finished();
  });

  const std::string map_id = "floor_1";

  vda5050_core::client::adapter::Pose2D world_ref{10.0, 5.0, M_PI_2};
  vda5050_core::client::adapter::Pose2D agv_ref{0.0, 0.0, 0.0};

  auto tf = vda5050_core::client::adapter::Transformation::calibrate(
    world_ref, agv_ref);

  adapter->state_manager()->set_transformation(tf, map_id);
  adapter->state_manager()->set_position(0.0, 0.0, 0.0, map_id);

  EXPECT_TRUE(adapter->state_manager()->position_initialized());

  adapter->start();

  auto order = make_order("order_id", 0, 1, 0);
  order.nodes[0].node_position = vda5050_core::types::NodePosition{};
  order.nodes[0].node_position->map_id = map_id;
  order.nodes[0].node_position->x = 10.0;
  order.nodes[0].node_position->y = 15.0;
  order.nodes[0].node_position->theta = M_PI_2;

  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return nav_called.load(); }));

  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_TRUE(received_request.has_value());
    const auto& pos = received_request->node_position();
    ASSERT_TRUE(pos.has_value());

    EXPECT_NEAR(pos->x, 10.0, 1e-4);
    EXPECT_NEAR(pos->y, 0.0, 1e-4);
    ASSERT_TRUE(pos->theta.has_value());
    EXPECT_NEAR(pos->theta.value(), 0.0, 1e-4);
  }

  adapter->state_manager()->set_position(5.0, 0.0, 0.0, map_id);

  auto current_state = adapter->state_manager()->state();
  ASSERT_TRUE(current_state.agv_position.has_value());
  EXPECT_NEAR(current_state.agv_position->x, 10.0, 1e-4);
  EXPECT_NEAR(current_state.agv_position->y, 10.0, 1e-4);
  EXPECT_NEAR(current_state.agv_position->theta, M_PI_2, 1e-4);

  adapter->stop();
}
