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

#include <memory>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/master/validation/traversability_validator.hpp"

namespace {

::testing::AssertionResult AllErrorsHaveTraversabilityType(
  const vda5050_core::order_utils::ValidationResult& res)
{
  for (const auto& e : res.errors)
  {
    if (e.error_type != vda5050_core::errors::TraversabilityValidationError)
    {
      return ::testing::AssertionFailure()
             << "found error with type '" << e.error_type
             << "' (expected TraversabilityValidationError)";
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

// Factsheet is irrelevant to traversability (which is state + layout driven),
// but PreSendContext requires one; this is a minimal stand-in.
vda5050_core::types::Factsheet make_factsheet()
{
  return vda5050_core::types::Factsheet{};
}

// State where AGV is on node "N0" — trivial reachability for an order whose
// first node is "N0".
vda5050_core::types::State make_state_on_node(const std::string& node_id)
{
  vda5050_core::types::State s;
  s.last_node_id = node_id;
  vda5050_core::types::AGVPosition pos;
  pos.position_initialized = true;
  pos.x = 0.0;
  pos.y = 0.0;
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
  o.order_id = "ORDER_T1";
  o.order_update_id = 0;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  o.nodes.push_back(n);
  return o;
}

}  // namespace

// ============================================================================
// Order — happy paths
// ============================================================================

TEST(TraversabilityValidatorTest, OrderHappyPathOnNode)
{
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  auto res = validate_traversability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_TRUE(res.errors.empty());
}

TEST(TraversabilityValidatorTest, OrderHappyPathByPositionWithinDeviation)
{
  auto state = make_state_on_node("OTHER");  // AGV not on first node
  state.agv_position->x = 0.05;              // 5cm offset
  state.agv_position->y = 0.0;
  auto ctx = make_ctx(state, make_factsheet());

  auto order = make_minimal_order();
  vda5050_core::types::NodePosition np;
  np.x = 0.0;
  np.y = 0.0;
  np.allowed_deviation_x_y = 0.10;  // 10cm tolerance
  order.nodes.front().node_position = np;

  auto res = validate_traversability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

// ============================================================================
// Order — reachability rejections
// ============================================================================

TEST(TraversabilityValidatorTest, OrderRejectFirstNodeUnreachable)
{
  auto state = make_state_on_node("OTHER");
  state.agv_position->x = 5.0;  // 5m away
  state.agv_position->y = 0.0;
  auto ctx = make_ctx(state, make_factsheet());

  auto order = make_minimal_order();
  vda5050_core::types::NodePosition np;
  np.x = 0.0;
  np.y = 0.0;
  np.allowed_deviation_x_y = 0.10;  // 10cm tolerance
  order.nodes.front().node_position = np;

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveTraversabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "deviation"));
}

TEST(TraversabilityValidatorTest, OrderRejectInsufficientReachabilityData)
{
  // AGV not on node, AND first node has no node_position.
  auto state = make_state_on_node("OTHER");
  auto ctx = make_ctx(state, make_factsheet());
  auto order = make_minimal_order();
  // node_position omitted
  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveTraversabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "Cannot determine reachability"));
}

TEST(TraversabilityValidatorTest, OrderRejectUninitializedPosition)
{
  auto state = make_state_on_node("OTHER");
  state.agv_position->position_initialized = false;
  state.agv_position->x = 0.0;
  state.agv_position->y = 0.0;
  auto ctx = make_ctx(state, make_factsheet());

  auto order = make_minimal_order();
  vda5050_core::types::NodePosition np;
  np.x = 0.0;
  np.y = 0.0;
  order.nodes.front().node_position = np;

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveTraversabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "not initialized"));
}

TEST(TraversabilityValidatorTest, OrderRejectFirstNodeOnDifferentMap)
{
  auto state = make_state_on_node("OTHER");
  state.agv_position->map_id = "L1";
  state.agv_position->x = 0.0;
  state.agv_position->y = 0.0;
  auto ctx = make_ctx(state, make_factsheet());

  auto order = make_minimal_order();
  vda5050_core::types::NodePosition np;
  np.x = 0.0;
  np.y = 0.0;
  np.map_id = "L2";
  np.allowed_deviation_x_y = 1.0;
  order.nodes.front().node_position = np;

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHaveTraversabilityType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "different map"));
}

// ============================================================================
// Graph integrity — cross-checks Order topology against the loaded layout.
// ============================================================================
//
// Topological only: node/edge existence, directed endpoint match, and node
// map_id agreement. Runs when ctx.loaded_graph is non-null; a null graph is
// skipped (logged) rather than rejected.

namespace {

using ::vda5050_core::layout::Edge;
using ::vda5050_core::layout::Graph;
using ::vda5050_core::layout::Layout;
using ::vda5050_core::layout::LIF;
using ::vda5050_core::layout::Node;
using ::vda5050_core::layout::VehicleTypeEdgeProperty;

Node make_layout_node(const std::string& id, const std::string& map_id)
{
  Node n;
  n.node_id = id;
  n.map_id = map_id;
  n.node_position = {0.0, 0.0};
  n.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  return n;
}

Edge make_layout_edge(
  const std::string& id, const std::string& start, const std::string& end)
{
  Edge e;
  e.edge_id = id;
  e.start_node_id = start;
  e.end_node_id = end;
  VehicleTypeEdgeProperty p;
  p.vehicle_type_id = "v1";
  e.vehicle_type_edge_properties.push_back(p);
  return e;
}

// Two nodes on map "L1" with one directed edge N0 --E1--> N1.
Graph::ConstPtr make_two_node_graph()
{
  LIF lif;
  lif.meta_information = {"p", "c", "2026-01-01T00:00:00Z", "0.11.0"};
  Layout layout;
  layout.layout_id = "L1";
  layout.layout_version = "1";
  layout.nodes.push_back(make_layout_node("N0", "L1"));
  layout.nodes.push_back(make_layout_node("N1", "L1"));
  layout.edges.push_back(make_layout_edge("E1", "N0", "N1"));
  lif.layouts.push_back(layout);
  return Graph::from_lif(std::move(lif));
}

PreSendContext make_ctx_with_graph(
  const vda5050_core::types::State& state,
  const vda5050_core::types::Factsheet& fs, Graph::ConstPtr graph)
{
  return PreSendContext{
    vda5050_core::types::ConnectionState::ONLINE, state, fs,
    AGVState::AVAILABLE, std::move(graph)};
}

vda5050_core::types::Edge make_order_edge(
  const std::string& id, const std::string& start, const std::string& end)
{
  vda5050_core::types::Edge e;
  e.edge_id = id;
  e.sequence_id = 1;
  e.released = true;
  e.start_node_id = start;
  e.end_node_id = end;
  return e;
}

}  // namespace

TEST(TraversabilityValidatorTest, GraphIntegrity_HappyPath_Accepts)
{
  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(), make_two_node_graph());
  auto order = make_minimal_order();
  order.edges.push_back(make_order_edge("E1", "N0", "N1"));

  auto res = validate_traversability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(TraversabilityValidatorTest, GraphIntegrity_OrderNodeNotInLayout_Rejects)
{
  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(), make_two_node_graph());
  auto order = make_minimal_order();
  order.nodes.front().node_id = "GHOST";  // not in the layout

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AnyErrorMentions(res, "GHOST"));
  EXPECT_TRUE(
    AnyErrorMentions(res, "not present in the master's loaded layout"));
}

TEST(TraversabilityValidatorTest, GraphIntegrity_OrderEdgeNotInLayout_Rejects)
{
  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(), make_two_node_graph());
  auto order = make_minimal_order();
  order.edges.push_back(make_order_edge("GHOST_EDGE", "N0", "N1"));

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AnyErrorMentions(res, "GHOST_EDGE"));
}

TEST(TraversabilityValidatorTest, GraphIntegrity_EdgeEndpointMismatch_Rejects)
{
  // Layout has N0->N1; order references E1 but with reversed endpoints.
  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(), make_two_node_graph());
  auto order = make_minimal_order();
  order.edges.push_back(make_order_edge("E1", "N1", "N0"));

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AnyErrorMentions(res, "endpoints"));
}

TEST(TraversabilityValidatorTest, GraphIntegrity_ReverseViaSeparateEdge_Accepts)
{
  // Reverse traversal in LIF is a distinct directed edge with its own id.
  LIF lif;
  lif.meta_information = {"p", "c", "2026-01-01T00:00:00Z", "0.11.0"};
  Layout layout;
  layout.layout_id = "L1";
  layout.layout_version = "1";
  layout.nodes.push_back(make_layout_node("N0", "L1"));
  layout.nodes.push_back(make_layout_node("N1", "L1"));
  layout.edges.push_back(make_layout_edge("E1", "N0", "N1"));
  layout.edges.push_back(make_layout_edge("E2", "N1", "N0"));
  lif.layouts.push_back(layout);

  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(),
    Graph::from_lif(std::move(lif)));
  auto order = make_minimal_order();
  order.edges.push_back(make_order_edge("E2", "N1", "N0"));

  auto res = validate_traversability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res)) << "reverse traversal via a distinct "
                                         "directed edge should be accepted";
}

TEST(TraversabilityValidatorTest, GraphIntegrity_EdgeIdMustMatchLayout_Rejects)
{
  // Endpoints exist as an edge (N0->N1), but the order's edge_id is not the
  // layout edge id — lookup is by edge_id, so this is rejected.
  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(), make_two_node_graph());
  auto order = make_minimal_order();
  order.edges.push_back(make_order_edge("WRONG_ID", "N0", "N1"));

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AnyErrorMentions(res, "WRONG_ID"));
}

TEST(TraversabilityValidatorTest, GraphIntegrity_NodeMapIdMismatch_Rejects)
{
  auto ctx = make_ctx_with_graph(
    make_state_on_node("N0"), make_factsheet(), make_two_node_graph());
  auto order = make_minimal_order();
  vda5050_core::types::NodePosition np;
  np.x = 0.0;
  np.y = 0.0;
  np.map_id = "L2";  // layout node N0 is on map "L1"
  order.nodes.front().node_position = np;

  auto res = validate_traversability(ctx, order);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AnyErrorMentions(res, "does not match the layout's map_id"));
}

TEST(TraversabilityValidatorTest, GraphIntegrity_SkippedWhenNoGraph_Accepts)
{
  // Null graph → integrity checks skipped (logged), not rejected.
  auto ctx = make_ctx(make_state_on_node("N0"), make_factsheet());
  auto order = make_minimal_order();
  order.edges.push_back(make_order_edge("ANY", "N0", "N1"));

  auto res = validate_traversability(ctx, order);
  EXPECT_TRUE(static_cast<bool>(res));
}

}  // namespace vda5050_core::master::test
