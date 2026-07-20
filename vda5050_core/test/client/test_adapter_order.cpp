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

#include "adapter_test_fixture.hpp"

class AdapterOrderTest : public AdapterTest
{};

TEST_F(AdapterOrderTest, RejectsInvalidOrder)
{
  std::atomic_bool called = false;

  adapter->on_navigate(
    [&](
      NodeRequest /*node_request*/, std::optional<EdgeRequest> /*edge_request*/,
      std::shared_ptr<OrderExecution> /*execution*/) { called = true; });

  adapter->start();

  auto order = make_order("order_id", 0, 2, 0);
  order.nodes[1].sequence_id = order.nodes[0].sequence_id;
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_publish(2));

  EXPECT_FALSE(called);

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  EXPECT_FALSE(state.errors.empty());
  EXPECT_EQ(state.errors[0].error_type, "graphValidationError");

  adapter->stop();
}

TEST_F(AdapterOrderTest, AcceptsOrderUpdate)
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

  ASSERT_TRUE(wait_until([&] { return call_count == 1; }));

  order = make_order("order_id", 1, 2, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_until([&] { return call_count == 2; }));

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  EXPECT_EQ(state.order_id, "order_id");
  EXPECT_EQ(state.order_update_id, 1);

  adapter->stop();
}

TEST_F(AdapterOrderTest, RejectsInvalidOrderUpdate)
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

  ASSERT_TRUE(wait_publish(3));

  order = make_order("order_id", 0, 2, 0);
  inject_message(
    fmt::format("{}/order", protocol_adapter->get_topic_prefix()),
    nlohmann::json(order).dump());

  ASSERT_TRUE(wait_publish(4));

  EXPECT_EQ(call_count, 1);

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  EXPECT_FALSE(state.errors.empty());
  EXPECT_EQ(state.errors[0].error_type, "orderUpdateError");

  adapter->stop();
}
