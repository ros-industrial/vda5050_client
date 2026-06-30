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

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/validation/protocol_limits_validator.hpp"

namespace {

::testing::AssertionResult AllErrorsHaveProtocolLimitType(
  const vda5050_core::order_utils::ValidationResult& res)
{
  for (const auto& e : res.errors)
  {
    if (e.error_type != vda5050_core::errors::ProtocolLimitError)
    {
      return ::testing::AssertionFailure()
             << "found error with type '" << e.error_type
             << "' (expected ProtocolLimitError)";
    }
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult AnyErrorMentions(
  const vda5050_core::order_utils::ValidationResult& res,
  const std::string& needle)
{
  for (const auto& e : res.errors)
  {
    if (
      e.error_description &&
      e.error_description->find(needle) != std::string::npos)
    {
      return ::testing::AssertionSuccess();
    }
  }
  return ::testing::AssertionFailure()
         << "no error description mentions '" << needle << "'";
}

}  // namespace

namespace vda5050_core::master::test {

namespace {

// Factsheet with generous array limits; individual tests tighten one limit.
vda5050_core::types::Factsheet make_factsheet()
{
  vda5050_core::types::Factsheet fs;
  fs.protocol_limits.max_array_lens.order_nodes = 100;
  fs.protocol_limits.max_array_lens.order_edges = 100;
  fs.protocol_limits.max_array_lens.node_actions = 10;
  fs.protocol_limits.max_array_lens.edge_actions = 10;
  fs.protocol_limits.max_array_lens.actions_actions_parameters = 10;
  fs.protocol_limits.max_array_lens.instant_actions = 10;
  return fs;
}

vda5050_core::types::State make_state_on_node(const std::string& node_id)
{
  vda5050_core::types::State s;
  s.last_node_id = node_id;
  vda5050_core::types::AGVPosition pos;
  pos.position_initialized = true;
  s.agv_position = pos;
  return s;
}

PreSendContext make_ctx(
  const vda5050_core::types::State& state,
  const vda5050_core::types::Factsheet& fs)
{
  return PreSendContext{
    vda5050_core::types::ConnectionState::ONLINE, state, fs,
    AGVState::AVAILABLE, nullptr};
}

vda5050_core::types::Order make_minimal_order()
{
  vda5050_core::types::Order o;
  o.order_id = "ORDER_L1";
  o.order_update_id = 0;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  o.nodes.push_back(n);
  return o;
}

vda5050_core::types::Action make_action(const std::string& id)
{
  vda5050_core::types::Action a;
  a.action_id = id;
  a.action_type = "pick";
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  return a;
}

}  // namespace

// ============================================================================
// Order — array-size limit rejections
// ============================================================================

TEST(ProtocolLimitsValidatorTest, OrderWithinLimitsPasses)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  order.nodes.front().actions = {make_action("A1")};
  auto res = validate_protocol_limits(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(ProtocolLimitsValidatorTest, OrderRejectExceedsMaxNodes)
{
  auto fs = make_factsheet();
  fs.protocol_limits.max_array_lens.order_nodes = 1;  // tight limit
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  vda5050_core::types::Node n2;
  n2.node_id = "N1";
  n2.sequence_id = 2;
  n2.released = true;
  order.nodes.push_back(n2);

  auto res = validate_protocol_limits(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveProtocolLimitType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "order_nodes"));
}

TEST(ProtocolLimitsValidatorTest, OrderRejectExceedsMaxNodeActions)
{
  auto fs = make_factsheet();
  fs.protocol_limits.max_array_lens.node_actions = 1;
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  order.nodes.front().actions = {make_action("A1"), make_action("A2")};

  auto res = validate_protocol_limits(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveProtocolLimitType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "node_actions"));
}

// ============================================================================
// InstantActions — array-size limit
// ============================================================================

TEST(ProtocolLimitsValidatorTest, InstantActionsRejectExceedsMaxLength)
{
  auto fs = make_factsheet();
  fs.protocol_limits.max_array_lens.instant_actions = 1;
  auto ctx = make_ctx(make_state_on_node("N0"), fs);
  vda5050_core::types::InstantActions ia;
  ia.actions = {make_action("A1"), make_action("A2")};
  auto res = validate_protocol_limits(ctx, ia);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveProtocolLimitType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "instant_actions"));
}

// ============================================================================
// Skip-when-factsheet-missing
// ============================================================================

TEST(ProtocolLimitsValidatorTest, OrderSkipsLimitChecksWhenNoFactsheet)
{
  PreSendContext ctx{
    vda5050_core::types::ConnectionState::ONLINE, make_state_on_node("N0"),
    std::nullopt, AGVState::AVAILABLE, nullptr};
  auto order = make_minimal_order();
  vda5050_core::types::Node n2;
  n2.node_id = "N1";
  n2.sequence_id = 2;
  n2.released = true;
  order.nodes.push_back(n2);
  auto res = validate_protocol_limits(ctx, order);
  EXPECT_FALSE(res.has_fatal());  // valid — skip is advisory, not FATAL
  EXPECT_EQ(res.warnings().size(), 1u);
}

// ============================================================================
// Multi-error accumulation
// ============================================================================

TEST(ProtocolLimitsValidatorTest, MultipleLimitErrorsAccumulate)
{
  auto fs = make_factsheet();
  fs.protocol_limits.max_array_lens.order_nodes = 1;
  fs.protocol_limits.max_array_lens.node_actions = 1;
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  vda5050_core::types::Node n2;
  n2.node_id = "N1";
  n2.sequence_id = 2;
  n2.released = true;
  order.nodes.push_back(n2);  // exceeds order_nodes
  order.nodes.front().actions = {
    make_action("A1"), make_action("A2")};  // exceeds node_actions

  auto res = validate_protocol_limits(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_GE(res.errors.size(), 2u);
  EXPECT_TRUE(AllErrorsHaveProtocolLimitType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "order_nodes"));
  EXPECT_TRUE(AnyErrorMentions(res, "node_actions"));
}

}  // namespace vda5050_core::master::test
