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

#include <memory>
#include <typeindex>

#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/updates/order.hpp"
#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/types/order.hpp"

namespace {

using namespace vda5050_core::execution;  // NOLINT
using namespace vda5050_core::client;     // NOLINT
namespace types = vda5050_core::types;

// Test if the OrderUpdate struct correctly carries the Order message and reports its type
// Test 1: Create an OrderUpdate with a sample Order and check if the fields are correctly stored.
TEST(OrderTypesTest, OrderUpdateCarriesOrder)
{
  types::Order order;
  order.order_id = "order_1";
  order.order_update_id = 3;

  OrderUpdate update(order);

  EXPECT_EQ(update.order.order_id, "order_1");
  EXPECT_EQ(update.order.order_update_id, 3u);
}

// Test 2: Check if OrderUpdate reports its type correctly through the base class.
TEST(OrderTypesTest, OrderUpdateReportsItsTypeThroughBase)
{
  // Create an OrderUpdate and store it as an UpdateBase pointer
  std::shared_ptr<UpdateBase> base =
    std::make_shared<OrderUpdate>(types::Order{});

  EXPECT_EQ(base->get_type(), std::type_index(typeid(OrderUpdate)));

  // The type tag must allow recovery of the concrete type, which is how the
  // Provider/Context dispatch updates back to typed handlers.
  auto recovered = std::static_pointer_cast<OrderUpdate>(base);
  ASSERT_NE(recovered, nullptr);
  EXPECT_TRUE(recovered->order.order_id.empty());
}

// Test 3: Check whether ConfigResource stores all four strings correctly
TEST(OrderTypesTest, ConfigResourceCarriesAllFields)
{
  HeaderConfigResource config("uagv", "2.0.0", "ROS-I", "S001");

  EXPECT_EQ(config.interface_name, "uagv");
  EXPECT_EQ(config.version, "2.0.0");
  EXPECT_EQ(config.manufacturer, "ROS-I");
  EXPECT_EQ(config.serial_number, "S001");
}

// Test 4: Check if ConfigResource reports its type correctly through the base class.
TEST(OrderTypesTest, ConfigResourceReportsItsTypeThroughBase)
{
  std::shared_ptr<ResourceBase> base =
    std::make_shared<HeaderConfigResource>("uagv", "2.0.0", "ROS-I", "S001");

  EXPECT_EQ(base->get_type(), std::type_index(typeid(HeaderConfigResource)));
}

// Test 5: Check that OrderUpdate and ConfigResource have distinct type indices,
// ensuring they are treated as distinct types in the Provider/Context system.
TEST(OrderTypesTest, DistinctTypesHaveDistinctTypeIndex)
{
  EXPECT_NE(
    std::type_index(typeid(OrderUpdate)),
    std::type_index(typeid(HeaderConfigResource)));
}

}  // namespace
