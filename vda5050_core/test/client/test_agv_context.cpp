/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gmock/gmock.h>

#include <atomic>
#include <memory>
#include <stdexcept>

#include "vda5050_core/client/contexts/agv_context.hpp"
#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/updates/order.hpp"
#include "vda5050_core/types/order.hpp"

namespace {

using AGVContext = vda5050_core::client::AGVContext;
using HeaderConfigResource = vda5050_core::client::HeaderConfigResource;
using OrderExecutionResource = vda5050_core::client::OrderExecutionResource;
using OrderUpdate = vda5050_core::client::OrderUpdate;
namespace execution = vda5050_core::execution;
namespace types = vda5050_core::types;

std::shared_ptr<AGVContext> make_context()
{
  auto config =
    std::make_shared<HeaderConfigResource>("uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);
  context->init();
  return context;
}

// Test 1: Make sure cannot create context without config.
TEST(AGVContextTest, ThrowsForNullConfig)
{
  EXPECT_THROW(AGVContext::make(nullptr), std::invalid_argument);
}

// Test 2: Make sure robot identity metadata matched what has passed in.
TEST(AGVContextTest, SeedsHeaderConfigResource)
{
  auto context = make_context();

  auto config = context->get_resource<HeaderConfigResource>();
  ASSERT_NE(config, nullptr);
  EXPECT_EQ(config->interface_name, "uagv");
  EXPECT_EQ(config->version, "2.0.0");
  EXPECT_EQ(config->manufacturer, "ROS-I");
  EXPECT_EQ(config->serial_number, "S001");
}

// Test 3: Ensure the robot starts in a clean state.
TEST(AGVContextTest, SeedsOrderExecutionResource)
{
  auto context = make_context();

  auto execution = context->get_resource<OrderExecutionResource>();
  ASSERT_NE(execution, nullptr);
  EXPECT_FALSE(execution->is_executing_order());
  EXPECT_FALSE(execution->is_awaiting_order_update());

  auto snapshot = execution->get_state();
  EXPECT_TRUE(snapshot.order_id.empty());
  EXPECT_EQ(snapshot.order_update_id, 0u);
  EXPECT_TRUE(snapshot.last_node_id.empty());
  EXPECT_EQ(snapshot.last_node_sequence_id, 0u);
  EXPECT_TRUE(snapshot.node_states.empty());
  EXPECT_TRUE(snapshot.edge_states.empty());
  EXPECT_TRUE(snapshot.action_states.empty());
}

// Test 4: Make sure get_resource() always return same object.
TEST(AGVContextTest, OrderExecutionResourceIsSharedAcrossLookups)
{
  auto context = make_context();

  auto a = context->get_resource<OrderExecutionResource>();
  a->set_executing_order(true);

  types::State state;
  state.order_id = "order_1";
  state.order_update_id = 2;
  state.last_node_id = "node_a";
  state.last_node_sequence_id = 4;
  a->set_state(state);

  auto b = context->get_resource<OrderExecutionResource>();
  ASSERT_EQ(a.get(), b.get());
  EXPECT_TRUE(b->is_executing_order());

  auto snapshot = b->get_state();
  EXPECT_EQ(snapshot.order_id, "order_1");
  EXPECT_EQ(snapshot.order_update_id, 2u);
  EXPECT_EQ(snapshot.last_node_id, "node_a");
  EXPECT_EQ(snapshot.last_node_sequence_id, 4u);
}

// Test 5: Make sure the incoming order is received and stored in context.
TEST(AGVContextTest, CachesOrderUpdate)
{
  auto context = make_context();

  types::Order order;
  order.order_id = "order_42";
  order.order_update_id = 7;
  context->provider()->push<OrderUpdate>(order);

  auto retrieved = context->get_update<OrderUpdate>();
  ASSERT_NE(retrieved, nullptr);
  EXPECT_EQ(retrieved->order.order_id, "order_42");
  EXPECT_EQ(retrieved->order.order_update_id, 7u);
}

// Test 6: Check that the latest OrderUpdate will overwrite the previous one.
TEST(AGVContextTest, ReplacesOrderUpdateWithLatest)
{
  auto context = make_context();

  types::Order order1;
  order1.order_id = "order_1";
  order1.order_update_id = 1;
  context->provider()->push<OrderUpdate>(order1);

  types::Order order2;
  order2.order_id = "order_2";
  order2.order_update_id = 2;
  context->provider()->push<OrderUpdate>(order2);

  auto retrieved = context->get_update<OrderUpdate>();
  ASSERT_NE(retrieved, nullptr);
  EXPECT_EQ(retrieved->order.order_id, "order_2");
  EXPECT_EQ(retrieved->order.order_update_id, 2u);
}

// Test 7: Make sure no crash before any update arrives.
TEST(AGVContextTest, ReturnsNullptrForUnpushedUpdate)
{
  auto context = make_context();
  EXPECT_EQ(context->get_update<OrderUpdate>(), nullptr);
}

// Test 8: Make sure on_change fires once per OrderUpdate push.
TEST(AGVContextTest, InvokesChangeCallbackOnOrderUpdate)
{
  auto config =
    std::make_shared<HeaderConfigResource>("uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);

  std::atomic_int count = 0;
  context->on_change([&count]() { count++; });
  context->init();

  types::Order order;
  order.order_id = "order_1";
  context->provider()->push<OrderUpdate>(order);

  EXPECT_EQ(count, 1);
}

// Test 9: Make sure each update triggers the event separately.
TEST(AGVContextTest, MultipleUpdatesInvokeCallbackMultipleTimes)
{
  auto config =
    std::make_shared<HeaderConfigResource>("uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);

  std::atomic_int count = 0;
  context->on_change([&count]() { count++; });
  context->init();

  types::Order order1;
  order1.order_id = "order_1";
  context->provider()->push<OrderUpdate>(order1);

  types::Order order2;
  order2.order_id = "order_2";
  context->provider()->push<OrderUpdate>(order2);

  EXPECT_EQ(count, 2);
}

// Test 10: Make sure get_resource returns nullptr for an unknown type.
TEST(AGVContextTest, ReturnsNullptrForUnknownResourceType)
{
  struct UnknownResource
  : public execution::Initialize<UnknownResource, execution::ResourceBase>
  {};

  auto context = make_context();
  EXPECT_EQ(context->get_resource<UnknownResource>(), nullptr);
}

}  // namespace
