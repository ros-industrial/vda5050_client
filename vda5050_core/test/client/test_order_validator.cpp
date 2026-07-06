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
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/client/contexts/agv_context.hpp"
#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/client/strategies/order_validator.hpp"
#include "vda5050_core/types/order.hpp"

namespace {

using AGVContext = vda5050_core::client::AGVContext;
using OrderExecutionResource = vda5050_core::client::OrderExecutionResource;
using OrderValidator = vda5050_core::client::OrderValidator;
namespace types = vda5050_core::types;

std::shared_ptr<AGVContext> make_context()
{
  auto config = std::make_shared<vda5050_core::client::HeaderConfigResource>(
    "uagv", "2.0.0", "ROS-I", "S001");
  auto context = AGVContext::make(config);
  context->init();
  return context;
}

types::Node make_node(const std::string& node_id, uint32_t sequence_id)
{
  types::Node node;
  node.node_id = node_id;
  node.sequence_id = sequence_id;
  node.released = true;
  return node;
}

types::Edge make_edge(
  const std::string& edge_id, uint32_t sequence_id,
  const std::string& start_node_id, const std::string& end_node_id)
{
  types::Edge edge;
  edge.edge_id = edge_id;
  edge.sequence_id = sequence_id;
  edge.start_node_id = start_node_id;
  edge.end_node_id = end_node_id;
  edge.released = true;
  return edge;
}

// A minimal structurally-valid single-node order (node 0, no edges).
types::Order single_node_order(
  const std::string& order_id, uint32_t order_update_id)
{
  types::Order order;
  order.order_id = order_id;
  order.order_update_id = order_update_id;
  order.nodes.push_back(make_node("node_0", 0));
  return order;
}

void seed_execution_state(
  const std::shared_ptr<AGVContext>& context, const std::string& order_id,
  uint32_t order_update_id, const std::string& last_node_id,
  uint32_t last_node_sequence_id,
  const std::vector<types::NodeState>& node_states = {})
{
  auto execution = context->get_resource<OrderExecutionResource>();
  ASSERT_NE(execution, nullptr);
  types::State state = execution->get_state();
  state.order_id = order_id;
  state.order_update_id = order_update_id;
  state.last_node_id = last_node_id;
  state.last_node_sequence_id = last_node_sequence_id;
  state.node_states = node_states;
  execution->set_state(std::move(state));
}

// Test 1: Accept a valid, structurally sound new order.
TEST(OrderValidatorTest, AcceptsValidNewOrder)
{
  OrderValidator validator;
  auto context = make_context();

  const auto result =
    validator.validate_order(single_node_order("new_1", 0), context);

  EXPECT_TRUE(result.accepted());
  EXPECT_TRUE(result.errors.empty());
}

// Test 2: Reject a structurally invalid order (step 1, reuses is_valid_graph).
TEST(OrderValidatorTest, RejectsStructurallyInvalidOrder)
{
  OrderValidator validator;
  auto context = make_context();

  // Two nodes but no connecting edge violates edges == nodes - 1.
  types::Order order;
  order.order_id = "bad";
  order.order_update_id = 0;
  order.nodes.push_back(make_node("node_0", 0));
  order.nodes.push_back(make_node("node_2", 2));

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.rejected());
  ASSERT_FALSE(result.errors.empty());
  EXPECT_EQ(result.errors.front().error_type, "graphValidationError");
}

// Test 3: Reject a new order while the vehicle is still busy.
TEST(OrderValidatorTest, RejectsNewOrderWhileBusy)
{
  OrderValidator validator;
  auto context = make_context();

  types::NodeState pending;
  pending.node_id = "node_x";
  pending.sequence_id = 4;
  seed_execution_state(context, "active", 1, "node_x", 4, {pending});

  const auto result =
    validator.validate_order(single_node_order("new_3", 0), context);

  EXPECT_TRUE(result.rejected());
  ASSERT_FALSE(result.errors.empty());
  EXPECT_EQ(result.errors.front().error_type, "validationError");
}

// Test 4: Accept a valid stitched update onto the active order.
TEST(OrderValidatorTest, AcceptsValidStitchedUpdate)
{
  OrderValidator validator;
  auto context = make_context();
  seed_execution_state(context, "active", 1, "node_a", 10);

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 2;
  order.nodes.push_back(make_node("node_a", 10));
  order.nodes.push_back(make_node("node_b", 12));
  order.edges.push_back(make_edge("edge_ab", 11, "node_a", "node_b"));

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.accepted());
}

// Test 5: A non-consecutive (but strictly greater) update id is still valid;
// VDA5050 does not require updates to increment by exactly one.
TEST(OrderValidatorTest, AcceptsNonConsecutiveUpdateId)
{
  OrderValidator validator;
  auto context = make_context();
  seed_execution_state(context, "active", 1, "node_a", 10);

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 4;  // jumps 1 -> 4, no edges needed (single node)
  order.nodes.push_back(make_node("node_a", 10));

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.accepted());
}

// Test 6: A duplicate update (same orderUpdateId) is ignored silently.
TEST(OrderValidatorTest, IgnoresDuplicateUpdate)
{
  OrderValidator validator;
  auto context = make_context();
  seed_execution_state(context, "active", 3, "node_a", 10);

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 3;
  order.nodes.push_back(make_node("node_a", 10));

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.ignored());
  EXPECT_FALSE(result.accepted());
  EXPECT_TRUE(result.errors.empty());
}

// Test 7: A deprecated update (lower id) is rejected with orderUpdateError
// and the previous order is kept -- not silently dropped.
TEST(OrderValidatorTest, RejectsDeprecatedUpdate)
{
  OrderValidator validator;
  auto context = make_context();
  seed_execution_state(context, "active", 5, "node_a", 10);

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 2;
  order.nodes.push_back(make_node("node_a", 10));

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.rejected());
  ASSERT_FALSE(result.errors.empty());
  EXPECT_EQ(result.errors.front().error_type, "orderUpdateError");
}

// Test 8a: An update is accepted when it stitches at the decision point (the
// last released base node) even though the AGV has not reached it yet. Here the
// AGV last reached node_a (seq 10) but node_c (seq 14) is still pending in the
// released base, so the update must stitch at node_c, not node_a.
TEST(OrderValidatorTest, AcceptsUpdateStitchedAtBaseEndAheadOfLastReached)
{
  OrderValidator validator;
  auto context = make_context();

  types::NodeState pending_c;
  pending_c.node_id = "node_c";
  pending_c.sequence_id = 14;
  pending_c.released = true;
  seed_execution_state(context, "active", 1, "node_a", 10, {pending_c});

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 2;
  order.nodes.push_back(
    make_node("node_c", 14));  // stitch at the decision point
  order.nodes.push_back(make_node("node_d", 16));
  order.edges.push_back(make_edge("edge_cd", 15, "node_c", "node_d"));

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.accepted());
}

// Test 8b: With a released base node still pending ahead of the last reached
// node, stitching at the last reached node (instead of the decision point) is
// rejected.
TEST(OrderValidatorTest, RejectsUpdateStitchedAtLastReachedWhenBaseStillPending)
{
  OrderValidator validator;
  auto context = make_context();

  types::NodeState pending_c;
  pending_c.node_id = "node_c";
  pending_c.sequence_id = 14;
  pending_c.released = true;
  seed_execution_state(context, "active", 1, "node_a", 10, {pending_c});

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 2;
  order.nodes.push_back(make_node("node_a", 10));  // stale anchor, not base end

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.rejected());
  ASSERT_FALSE(result.errors.empty());
  EXPECT_EQ(result.errors.front().error_type, "orderUpdateError");
}

// Test 9: An update whose stitch node does not match the last base node fails.
TEST(OrderValidatorTest, RejectsStitchNodeMismatch)
{
  OrderValidator validator;
  auto context = make_context();
  seed_execution_state(context, "active", 1, "node_a", 10);

  types::Order order;
  order.order_id = "active";
  order.order_update_id = 2;
  order.nodes.push_back(make_node("node_z", 10));  // wrong node id

  const auto result = validator.validate_order(order, context);

  EXPECT_TRUE(result.rejected());
  ASSERT_FALSE(result.errors.empty());
  EXPECT_EQ(result.errors.front().error_type, "orderUpdateError");
}

}  // namespace
