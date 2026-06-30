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
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/validation/capability_validator.hpp"

namespace {

::testing::AssertionResult AllErrorsHaveCapabilityType(
  const vda5050_core::order_utils::ValidationResult& res)
{
  for (const auto& e : res.errors)
  {
    if (e.error_type != vda5050_core::errors::CapabilityValidationError)
    {
      return ::testing::AssertionFailure()
             << "found error with type '" << e.error_type
             << "' (expected CapabilityValidationError)";
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

// Factsheet that supports action_type "pick" on NODE/EDGE/INSTANT scopes with
// a "loadId" parameter key.
vda5050_core::types::Factsheet make_factsheet()
{
  vda5050_core::types::Factsheet fs;
  vda5050_core::types::AGVAction supported;
  supported.action_type = "pick";
  supported.action_scopes = {
    vda5050_core::types::ActionScope::NODE,
    vda5050_core::types::ActionScope::EDGE,
    vda5050_core::types::ActionScope::INSTANT};
  vda5050_core::types::ActionParameterFactsheet param;
  param.key = "loadId";
  param.is_optional = true;
  supported.action_parameters =
    std::vector<vda5050_core::types::ActionParameterFactsheet>{param};
  fs.protocol_features.agv_actions = {supported};
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
  o.order_id = "ORDER_C1";
  o.order_update_id = 0;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  o.nodes.push_back(n);
  return o;
}

vda5050_core::types::Action make_pick_action()
{
  vda5050_core::types::Action a;
  a.action_id = "A1";
  a.action_type = "pick";
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  return a;
}

}  // namespace

// ============================================================================
// Order — action capability rejections
// ============================================================================

TEST(CapabilityValidatorTest, OrderRejectActionTypeNotInFactsheet)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  vda5050_core::types::Action unsupported;
  unsupported.action_id = "A1";
  unsupported.action_type = "fly";  // not in factsheet
  unsupported.blocking_type = vda5050_core::types::BlockingType::NONE;
  order.nodes.front().actions = {unsupported};

  auto res = validate_capability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "fly"));
}

TEST(CapabilityValidatorTest, OrderRejectActionScopeNotNode)
{
  // Build factsheet where "pick" is INSTANT-only — not allowed on Nodes.
  auto fs = make_factsheet();
  fs.protocol_features.agv_actions[0].action_scopes = {
    vda5050_core::types::ActionScope::INSTANT};
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  order.nodes.front().actions = {make_pick_action()};

  auto res = validate_capability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "scope"));
}

TEST(CapabilityValidatorTest, OrderRejectBlockingTypeNotSupported)
{
  // Factsheet declares "pick" supports HARD only; order requests SOFT.
  auto fs = make_factsheet();
  fs.protocol_features.agv_actions[0].blocking_types = {
    vda5050_core::types::BlockingType::HARD};
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  auto a = make_pick_action();
  a.blocking_type = vda5050_core::types::BlockingType::SOFT;
  order.nodes.front().actions = {a};

  auto res = validate_capability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "blocking_type"));
}

TEST(CapabilityValidatorTest, OrderAcceptBlockingTypeSupported)
{
  // Factsheet declares "pick" supports NONE and HARD; order requests NONE.
  auto fs = make_factsheet();
  fs.protocol_features.agv_actions[0].blocking_types = {
    vda5050_core::types::BlockingType::NONE,
    vda5050_core::types::BlockingType::HARD};
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  order.nodes.front().actions = {make_pick_action()};

  auto res = validate_capability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(CapabilityValidatorTest, OrderSkipsBlockingTypeCheckWhenFactsheetSilent)
{
  // make_factsheet leaves blocking_types unset — any blocking_type accepted.
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  auto a = make_pick_action();
  a.blocking_type = vda5050_core::types::BlockingType::SOFT;
  order.nodes.front().actions = {a};

  auto res = validate_capability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(CapabilityValidatorTest, OrderRejectActionParameterKeyNotInFactsheet)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  auto a = make_pick_action();
  vda5050_core::types::ActionParameter p;
  p.key = "weirdKey";  // not declared by factsheet
  p.value = "x";
  a.action_parameters = std::vector<vda5050_core::types::ActionParameter>{p};
  order.nodes.front().actions = {a};

  auto res = validate_capability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "weirdKey"));
}

TEST(CapabilityValidatorTest, OrderRejectMissingRequiredActionParam)
{
  auto fs = make_factsheet();
  fs.protocol_features.agv_actions[0].action_parameters->front().is_optional =
    false;  // loadId now required
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  order.nodes.front().actions = {make_pick_action()};  // no loadId supplied

  auto res = validate_capability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "missing required parameter"));
}

TEST(CapabilityValidatorTest, OrderAcceptsRequiredActionParamPresent)
{
  auto fs = make_factsheet();
  fs.protocol_features.agv_actions[0].action_parameters->front().is_optional =
    false;
  auto ctx = make_ctx(make_state_on_node("N0"), fs);

  auto order = make_minimal_order();
  auto a = make_pick_action();
  vda5050_core::types::ActionParameter p;
  p.key = "loadId";
  p.value = "L1";
  a.action_parameters = std::vector<vda5050_core::types::ActionParameter>{p};
  order.nodes.front().actions = {a};

  auto res = validate_capability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

// ============================================================================
// InstantActions — capability
// ============================================================================

TEST(CapabilityValidatorTest, InstantActionsHappyPath)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  vda5050_core::types::InstantActions ia;
  ia.actions = {make_pick_action()};
  auto res = validate_capability(ctx, ia);
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(CapabilityValidatorTest, InstantActionsRejectActionTypeNotInFactsheet)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  vda5050_core::types::InstantActions ia;
  vda5050_core::types::Action unsupported;
  unsupported.action_id = "A1";
  unsupported.action_type = "teleport";
  unsupported.blocking_type = vda5050_core::types::BlockingType::NONE;
  ia.actions = {unsupported};
  auto res = validate_capability(ctx, ia);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "teleport"));
}

TEST(CapabilityValidatorTest, InstantActionsRejectScopeNotInstant)
{
  auto fs = make_factsheet();
  fs.protocol_features.agv_actions[0].action_scopes = {
    vda5050_core::types::ActionScope::NODE};  // NODE only, NOT INSTANT
  auto ctx = make_ctx(make_state_on_node("N0"), fs);
  vda5050_core::types::InstantActions ia;
  ia.actions = {make_pick_action()};
  auto res = validate_capability(ctx, ia);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "scope"));
}

// ============================================================================
// Skip-when-factsheet-missing
// ============================================================================

TEST(CapabilityValidatorTest, OrderSkipsCapabilityChecksWhenNoFactsheet)
{
  PreSendContext ctx{
    vda5050_core::types::ConnectionState::ONLINE, make_state_on_node("N0"),
    std::nullopt, AGVState::AVAILABLE, nullptr};
  auto order = make_minimal_order();
  vda5050_core::types::Action unsupported;
  unsupported.action_id = "A1";
  unsupported.action_type = "fly";
  unsupported.blocking_type = vda5050_core::types::BlockingType::NONE;
  order.nodes.front().actions = {unsupported};
  auto res = validate_capability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));  // valid — skip is advisory, not FATAL
  EXPECT_EQ(res.warnings().size(), 1u);
}

TEST(
  CapabilityValidatorTest, InstantActionsSkipsCapabilityChecksWhenNoFactsheet)
{
  PreSendContext ctx{
    vda5050_core::types::ConnectionState::ONLINE, make_state_on_node("N0"),
    std::nullopt, AGVState::AVAILABLE, nullptr};
  vda5050_core::types::InstantActions ia;
  vda5050_core::types::Action a;
  a.action_id = "A1";
  a.action_type = "anything_goes";
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  ia.actions = {a};
  auto res = validate_capability(ctx, ia);
  EXPECT_TRUE(static_cast<bool>(res));  // valid — skip is advisory, not FATAL
  EXPECT_EQ(res.warnings().size(), 1u);
}

// ============================================================================
// Multi-error accumulation
// ============================================================================

TEST(CapabilityValidatorTest, MultipleCapabilityErrorsAccumulate)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  vda5050_core::types::Action a1;
  a1.action_id = "A1";
  a1.action_type = "fly";
  a1.blocking_type = vda5050_core::types::BlockingType::NONE;
  vda5050_core::types::Action a2;
  a2.action_id = "A2";
  a2.action_type = "teleport";
  a2.blocking_type = vda5050_core::types::BlockingType::NONE;
  order.nodes.front().actions = {a1, a2};

  auto res = validate_capability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_GE(res.errors.size(), 2u);
  EXPECT_TRUE(AllErrorsHaveCapabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "fly"));
  EXPECT_TRUE(AnyErrorMentions(res, "teleport"));
}

}  // namespace vda5050_core::master::test
