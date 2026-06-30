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

#include <algorithm>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/validation/content_validator.hpp"

namespace {

// Returns true iff every error in `res` has type ContentValidationError.
::testing::AssertionResult AllErrorsHaveContentType(
  const vda5050_core::errors::ValidationResult& res)
{
  for (const auto& e : res.errors)
  {
    if (e.error_type != vda5050_core::errors::ContentValidationError)
    {
      return ::testing::AssertionFailure()
             << "found error with type '" << e.error_type
             << "' (expected ContentValidationError)";
    }
  }
  return ::testing::AssertionSuccess();
}

// Returns true iff at least one error's description contains `needle`.
::testing::AssertionResult AnyErrorMentions(
  const vda5050_core::errors::ValidationResult& res, const std::string& needle)
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

// Fill a Header with valid mandatory fields. Tests can clobber individual
// fields to exercise specific failure paths.
void fill_valid_header(vda5050_core::types::Header& h)
{
  h.version = "2.0.0";
  h.manufacturer = "ACME";
  h.serial_number = "AGV001";
}

vda5050_core::types::Order make_valid_order()
{
  vda5050_core::types::Order o;
  fill_valid_header(o.header);
  o.order_id = "ORDER_1";
  o.order_update_id = 0;

  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  o.nodes.push_back(n);
  return o;
}

vda5050_core::types::InstantActions make_valid_instant_actions()
{
  vda5050_core::types::InstantActions ia;
  fill_valid_header(ia.header);

  vda5050_core::types::Action a;
  a.action_id = "A1";
  a.action_type = "stateRequest";
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  ia.actions.push_back(a);
  return ia;
}

vda5050_core::types::State make_valid_state()
{
  vda5050_core::types::State s;
  fill_valid_header(s.header);
  s.order_id = "";  // legitimately empty when no current order
  return s;
}

vda5050_core::types::Connection make_valid_connection()
{
  vda5050_core::types::Connection c;
  fill_valid_header(c.header);
  c.connection_state = vda5050_core::types::ConnectionState::ONLINE;
  return c;
}

vda5050_core::types::Factsheet make_valid_factsheet()
{
  vda5050_core::types::Factsheet f;
  fill_valid_header(f.header);
  return f;
}

vda5050_core::types::Visualization make_valid_visualization()
{
  vda5050_core::types::Visualization v;
  fill_valid_header(v.header);
  return v;
}

}  // namespace

// ============================================================================
// Header — common to all messages.
// ============================================================================

TEST(ContentValidatorTest, HeaderVersionMatch)
{
  auto o = make_valid_order();
  auto res = validate_order_content(o);
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_TRUE(res.errors.empty());
}

TEST(ContentValidatorTest, HeaderVersionMismatchRejected)
{
  auto o = make_valid_order();
  o.header.version = "1.3.2";
  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::ContentValidationError);
}

TEST(ContentValidatorTest, HeaderEmptyManufacturerRejected)
{
  auto o = make_valid_order();
  o.header.manufacturer = "";
  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "manufacturer"));
}

// ============================================================================
// Order
// ============================================================================

TEST(ContentValidatorTest, OrderValidPasses)
{
  auto o = make_valid_order();
  EXPECT_TRUE(static_cast<bool>(validate_order_content(o)));
}

TEST(ContentValidatorTest, OrderEmptyOrderIdRejected)
{
  auto o = make_valid_order();
  o.order_id = "";
  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "order_id"));
}

TEST(ContentValidatorTest, OrderNodeWithEmptyNodeIdRejected)
{
  auto o = make_valid_order();
  o.nodes.front().node_id = "";
  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "node_id"));
}

TEST(ContentValidatorTest, OrderActionWithEmptyActionTypeRejected)
{
  auto o = make_valid_order();
  vda5050_core::types::Action a;
  a.action_id = "A1";
  a.action_type = "";  // bad
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  o.nodes.front().actions.push_back(a);
  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "action_type"));
}

TEST(ContentValidatorTest, OrderEdgeWithEmptyEdgeIdRejected)
{
  auto o = make_valid_order();
  // Add a second node + connecting edge with empty edge_id.
  vda5050_core::types::Node n1;
  n1.node_id = "N1";
  n1.sequence_id = 2;
  n1.released = true;
  o.nodes.push_back(n1);

  vda5050_core::types::Edge e;
  e.edge_id = "";  // bad
  e.sequence_id = 1;
  e.start_node_id = "N0";
  e.end_node_id = "N1";
  e.released = true;
  o.edges.push_back(e);

  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "edge_id"));
}

TEST(ContentValidatorTest, OrderWithEmptyNodesPasses)
{
  // Content-validator does NOT enforce graph topology — empty `nodes`
  // is the graph validator's concern. Locks in layer separation:
  // content only checks shape, graph checks topology.
  auto o = make_valid_order();
  o.nodes.clear();
  EXPECT_TRUE(static_cast<bool>(validate_order_content(o)));
}

// ============================================================================
// InstantActions
// ============================================================================

TEST(ContentValidatorTest, InstantActionsValidPasses)
{
  auto ia = make_valid_instant_actions();
  EXPECT_TRUE(static_cast<bool>(validate_instant_actions_content(ia)));
}

TEST(ContentValidatorTest, InstantActionsEmptyActionIdRejected)
{
  auto ia = make_valid_instant_actions();
  ia.actions.front().action_id = "";
  auto res = validate_instant_actions_content(ia);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "action_id"));
}

TEST(ContentValidatorTest, InstantActionsEmptyActionTypeRejected)
{
  auto ia = make_valid_instant_actions();
  ia.actions.front().action_type = "";
  auto res = validate_instant_actions_content(ia);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "action_type"));
}

// ============================================================================
// State
// ============================================================================

TEST(ContentValidatorTest, StateValidPasses)
{
  auto s = make_valid_state();
  EXPECT_TRUE(static_cast<bool>(validate_state_content(s)));
}

TEST(ContentValidatorTest, StateEmptyOrderIdPassesByDesign)
{
  // Spec allows empty order_id when AGV has no current order. Content
  // validator must NOT reject this — a State with empty order_id is
  // structurally valid.
  auto s = make_valid_state();
  s.order_id = "";
  EXPECT_TRUE(static_cast<bool>(validate_state_content(s)));
}

TEST(ContentValidatorTest, StateNodeStateWithEmptyNodeIdRejected)
{
  auto s = make_valid_state();
  vda5050_core::types::NodeState ns;
  ns.node_id = "";
  ns.sequence_id = 1;
  ns.released = true;
  s.node_states.push_back(ns);
  auto res = validate_state_content(s);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "node_id"));
}

TEST(ContentValidatorTest, StateErrorWithEmptyErrorTypeRejected)
{
  auto s = make_valid_state();
  vda5050_core::types::Error err;
  err.error_type = "";
  err.error_level = vda5050_core::types::ErrorLevel::WARNING;
  s.errors.push_back(err);
  auto res = validate_state_content(s);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "error_type"));
}

// ============================================================================
// Connection / Factsheet / Visualization — minimal beyond header
// ============================================================================

TEST(ContentValidatorTest, ConnectionValidPasses)
{
  auto c = make_valid_connection();
  EXPECT_TRUE(static_cast<bool>(validate_connection_content(c)));
}

TEST(ContentValidatorTest, FactsheetValidPasses)
{
  auto f = make_valid_factsheet();
  EXPECT_TRUE(static_cast<bool>(validate_factsheet_content(f)));
}

TEST(ContentValidatorTest, VisualizationValidPasses)
{
  auto v = make_valid_visualization();
  EXPECT_TRUE(static_cast<bool>(validate_visualization_content(v)));
}

// ============================================================================
// Header version negative paths
// ============================================================================

TEST(ContentValidatorTest, HeaderVersionEmptyRejected)
{
  auto c = make_valid_connection();
  c.header.version = "";
  auto res = validate_connection_content(c);
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::ContentValidationError);
}

TEST(ContentValidatorTest, HeaderEmptySerialNumberRejected)
{
  auto f = make_valid_factsheet();
  f.header.serial_number = "";
  auto res = validate_factsheet_content(f);
  EXPECT_FALSE(static_cast<bool>(res));
}

// ============================================================================
// ValidationResult plumbing — multiple errors accumulate
// ============================================================================

TEST(ContentValidatorTest, MultipleErrorsAccumulate)
{
  auto o = make_valid_order();
  o.order_id = "";               // -> 1 error
  o.header.version = "9.9.9";    // -> 1 error
  o.header.manufacturer = "";    // -> 1 error
  o.nodes.front().node_id = "";  // -> 1 error
  auto res = validate_order_content(o);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_GE(res.errors.size(), 4u);
  EXPECT_TRUE(AllErrorsHaveContentType(res));
  // Spot-check that each independent issue surfaced its own message.
  EXPECT_TRUE(AnyErrorMentions(res, "order_id"));
  EXPECT_TRUE(AnyErrorMentions(res, "version"));
  EXPECT_TRUE(AnyErrorMentions(res, "manufacturer"));
  EXPECT_TRUE(AnyErrorMentions(res, "node_id"));
}

}  // namespace vda5050_core::master::test
