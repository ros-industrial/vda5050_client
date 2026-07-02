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
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/layout/layout_loader.hpp"

namespace vda5050_core::layout::test {

namespace {

LIF make_minimal_lif()
{
  LIF lif;
  lif.meta_information = {"p", "c", "2026-01-01T00:00:00Z", "0.11.0"};

  Layout layout;
  layout.layout_id = "L1";
  layout.layout_version = "1";

  Node n0;
  n0.node_id = "N0";
  n0.map_id = "L1";
  n0.node_position = {0.0, 0.0};
  n0.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  layout.nodes.push_back(n0);

  Node n1;
  n1.node_id = "N1";
  n1.map_id = "L1";
  n1.node_position = {5.0, 0.0};
  n1.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  layout.nodes.push_back(n1);

  Edge e1;
  e1.edge_id = "E1";
  e1.start_node_id = "N0";
  e1.end_node_id = "N1";
  VehicleTypeEdgeProperty p;
  p.vehicle_type_id = "v1";
  p.rotation_allowed = false;
  e1.vehicle_type_edge_properties.push_back(p);
  layout.edges.push_back(e1);

  Station s1;
  s1.station_id = "S1";
  s1.interaction_node_ids = {"N0"};
  layout.stations.push_back(s1);

  lif.layouts.push_back(layout);
  return lif;
}

LIF make_multi_layout_lif()
{
  LIF lif;
  lif.meta_information = {"p", "c", "2026-01-01T00:00:00Z", "0.11.0"};

  Layout la;
  la.layout_id = "LA";
  la.layout_version = "1";

  Node n0;
  n0.node_id = "N0";
  n0.map_id = "LA";
  n0.node_position = {0.0, 0.0};
  n0.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  la.nodes.push_back(n0);

  Node n1;
  n1.node_id = "N1";
  n1.map_id = "LA";
  n1.node_position = {5.0, 0.0};
  n1.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  la.nodes.push_back(n1);

  Edge e1;
  e1.edge_id = "E1";
  e1.start_node_id = "N0";
  e1.end_node_id = "N1";
  VehicleTypeEdgeProperty p1;
  p1.vehicle_type_id = "v1";
  p1.rotation_allowed = false;
  e1.vehicle_type_edge_properties.push_back(p1);
  la.edges.push_back(e1);

  Edge e_xfer;
  e_xfer.edge_id = "E_xfer";
  e_xfer.start_node_id = "N1";
  e_xfer.end_node_id = "N5";
  VehicleTypeEdgeProperty p_xfer;
  p_xfer.vehicle_type_id = "v1";
  p_xfer.rotation_allowed = false;
  e_xfer.vehicle_type_edge_properties.push_back(p_xfer);
  la.edges.push_back(e_xfer);

  lif.layouts.push_back(la);

  Layout lb;
  lb.layout_id = "LB";
  lb.layout_version = "1";

  Node n5;
  n5.node_id = "N5";
  n5.map_id = "LB";
  n5.node_position = {10.0, 0.0};
  n5.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  lb.nodes.push_back(n5);

  lif.layouts.push_back(lb);
  return lif;
}

LIF make_station_only_lif()
{
  LIF lif;
  lif.meta_information = {"p", "c", "2026-01-01T00:00:00Z", "0.11.0"};
  Layout layout;
  layout.layout_id = "L1";
  Node n;
  n.node_id = "N0";
  n.map_id = "L1";
  n.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  layout.nodes.push_back(n);
  Station s;
  s.station_id = "S1";
  s.interaction_node_ids = {"N0"};
  layout.stations.push_back(s);
  lif.layouts.push_back(layout);
  return lif;
}

}  // namespace

TEST(GraphTest, FromLif_EmptyLIF_Throws)
{
  // A LIF with no layouts is invalid (validate_layout rejects it), so it
  // cannot be wrapped in a Graph.
  EXPECT_THROW(Graph::from_lif(LIF{}), std::invalid_argument);
}

TEST(GraphTest, FromLif_MinimalLIF_Populates)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_FALSE(g->empty());
  EXPECT_EQ(g->node_count(), 2u);
  EXPECT_EQ(g->edge_count(), 1u);
  EXPECT_EQ(g->station_count(), 1u);
  EXPECT_TRUE(g->has_node("N0"));
  EXPECT_TRUE(g->has_node("N1"));
  EXPECT_TRUE(g->has_edge("E1"));
  EXPECT_TRUE(g->has_station("S1"));
}

TEST(GraphTest, FromLif_MultiLayout_IndicesSpanAllLayouts)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  EXPECT_EQ(g->node_count(), 3u);  // N0, N1 (LA) + N5 (LB)
  EXPECT_EQ(g->edge_count(), 2u);  // E1, E_xfer
  EXPECT_TRUE(g->has_node("N0"));
  EXPECT_TRUE(g->has_node("N1"));
  EXPECT_TRUE(g->has_node("N5"));
  EXPECT_NE(g->find_layout("LA"), nullptr);
  EXPECT_NE(g->find_layout("LB"), nullptr);
}

TEST(GraphTest, FromLif_DuplicateNodeId_Throws)
{
  LIF lif = make_minimal_lif();
  Node dup;
  dup.node_id = "N0";  // collision
  dup.map_id = "L1";
  dup.vehicle_type_node_properties.push_back(
    {"v1", std::nullopt, std::nullopt});
  lif.layouts[0].nodes.push_back(dup);
  EXPECT_THROW(Graph::from_lif(lif), std::invalid_argument);
}

TEST(GraphTest, FromLif_DanglingEdge_Throws)
{
  LIF lif = make_minimal_lif();
  Edge bad;
  bad.edge_id = "E_bad";
  bad.start_node_id = "N0";
  bad.end_node_id = "N_missing";  // references a node that does not exist
  VehicleTypeEdgeProperty p;
  p.vehicle_type_id = "v1";
  p.rotation_allowed = false;
  bad.vehicle_type_edge_properties.push_back(p);
  lif.layouts[0].edges.push_back(bad);
  EXPECT_THROW(Graph::from_lif(lif), std::invalid_argument);
}

TEST(GraphTest, HasFindReturnsFalseAndNullForMissing)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_FALSE(g->has_node("MISSING"));
  EXPECT_FALSE(g->has_edge("MISSING"));
  EXPECT_FALSE(g->has_station("MISSING"));
  EXPECT_EQ(g->find_node("MISSING"), nullptr);
  EXPECT_EQ(g->find_edge("MISSING"), nullptr);
  EXPECT_EQ(g->find_station("MISSING"), nullptr);
}

TEST(GraphTest, GetThrowsOnMissing)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_THROW(g->get_node("MISSING"), std::out_of_range);
  EXPECT_THROW(g->get_edge("MISSING"), std::out_of_range);
  EXPECT_THROW(g->get_station("MISSING"), std::out_of_range);
}

TEST(GraphTest, GetReturnsCorrectEntity)
{
  const LIF lif = make_minimal_lif();
  auto g = Graph::from_lif(lif);
  EXPECT_EQ(g->get_node("N0"), lif.layouts[0].nodes[0]);
  EXPECT_EQ(g->get_edge("E1"), lif.layouts[0].edges[0]);
  EXPECT_EQ(g->get_station("S1"), lif.layouts[0].stations[0]);
}

TEST(GraphTest, FindLayoutWorks)
{
  auto g = Graph::from_lif(make_minimal_lif());
  const auto* layout = g->find_layout("L1");
  ASSERT_NE(layout, nullptr);
  EXPECT_EQ(layout->layout_id, "L1");
  EXPECT_EQ(g->find_layout("MISSING"), nullptr);
}

TEST(GraphTest, HasNodeIsConsistentWithFindNode)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_EQ(g->has_node("N0"), g->find_node("N0") != nullptr);
  EXPECT_EQ(g->has_node("MISSING"), g->find_node("MISSING") != nullptr);
}

TEST(GraphTest, FindNodeReturnsPointerInUnderlyingLIF)
{
  auto g = Graph::from_lif(make_minimal_lif());
  const auto* n = g->find_node("N0");
  ASSERT_NE(n, nullptr);
  EXPECT_EQ(n, &g->lif().layouts[0].nodes[0]);
}

TEST(GraphTest, OutboundEdges_SingleEdge)
{
  auto g = Graph::from_lif(make_minimal_lif());
  const auto& out = g->outbound_edges("N0");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0], "E1");
}

TEST(GraphTest, InboundEdges_SingleEdge)
{
  auto g = Graph::from_lif(make_minimal_lif());
  const auto& in = g->inbound_edges("N1");
  ASSERT_EQ(in.size(), 1u);
  EXPECT_EQ(in[0], "E1");
  EXPECT_TRUE(g->outbound_edges("N1").empty());
  EXPECT_TRUE(g->inbound_edges("N0").empty());
}

TEST(GraphTest, MultiEdge_SameStartEnd_BothListed)
{
  // Parallel edges between the same node pair with different
  // vehicleTypeEdgeProperties are legal.
  LIF lif = make_minimal_lif();
  Edge e2;
  e2.edge_id = "E2";
  e2.start_node_id = "N0";
  e2.end_node_id = "N1";
  VehicleTypeEdgeProperty p2;
  p2.vehicle_type_id = "v2";
  p2.rotation_allowed = true;
  e2.vehicle_type_edge_properties.push_back(p2);
  lif.layouts[0].edges.push_back(e2);

  auto g = Graph::from_lif(lif);
  const auto& out = g->outbound_edges("N0");
  EXPECT_EQ(out.size(), 2u);
  std::set<std::string> ids(out.begin(), out.end());
  EXPECT_EQ(ids.count("E1"), 1u);
  EXPECT_EQ(ids.count("E2"), 1u);
}

TEST(GraphTest, EdgesBetween_MultiEdge_ReturnsAll)
{
  LIF lif = make_minimal_lif();
  Edge e2;
  e2.edge_id = "E2";
  e2.start_node_id = "N0";
  e2.end_node_id = "N1";
  VehicleTypeEdgeProperty p2;
  p2.vehicle_type_id = "v2";
  p2.rotation_allowed = true;
  e2.vehicle_type_edge_properties.push_back(p2);
  lif.layouts[0].edges.push_back(e2);

  auto g = Graph::from_lif(lif);
  auto between = g->edges_between("N0", "N1");
  EXPECT_EQ(between.size(), 2u);
  std::set<std::string> ids(between.begin(), between.end());
  EXPECT_EQ(ids.count("E1"), 1u);
  EXPECT_EQ(ids.count("E2"), 1u);
  EXPECT_TRUE(g->edges_between("N1", "N0").empty());
}

TEST(GraphTest, Adjacency_CrossLayoutEdge_OutboundInSource_InboundInDest)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  const auto& out_n1 = g->outbound_edges("N1");
  EXPECT_TRUE(
    std::find(out_n1.begin(), out_n1.end(), "E_xfer") != out_n1.end());
  const auto& in_n5 = g->inbound_edges("N5");
  EXPECT_TRUE(std::find(in_n5.begin(), in_n5.end(), "E_xfer") != in_n5.end());
}

TEST(GraphTest, ForEachNode_VisitsAll)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  std::vector<std::string> visited;
  g->for_each_node([&](const Node& n) { visited.push_back(n.node_id); });
  EXPECT_EQ(visited.size(), 3u);
  std::set<std::string> as_set(visited.begin(), visited.end());
  EXPECT_EQ(as_set.size(), 3u);
}

TEST(GraphTest, ForEachNodeOrdered_VisitsInIdOrder)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  std::vector<std::string> visited;
  g->for_each_node_ordered(
    [&](const Node& n) { visited.push_back(n.node_id); });
  std::vector<std::string> sorted_copy = visited;
  std::sort(sorted_copy.begin(), sorted_copy.end());
  EXPECT_EQ(visited, sorted_copy);
}

TEST(GraphTest, ForEachEdge_VisitsAll)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  std::vector<std::string> visited;
  g->for_each_edge([&](const Edge& e) { visited.push_back(e.edge_id); });
  EXPECT_EQ(visited.size(), 2u);
}

TEST(GraphTest, ForEachStation_VisitsAll)
{
  auto g = Graph::from_lif(make_minimal_lif());
  std::vector<std::string> visited;
  g->for_each_station(
    [&](const Station& s) { visited.push_back(s.station_id); });
  ASSERT_EQ(visited.size(), 1u);
  EXPECT_EQ(visited[0], "S1");
}

TEST(GraphTest, UnconnectedNodes_IdentifiesIsolated)
{
  LIF lif = make_minimal_lif();
  Node iso;
  iso.node_id = "N_iso";
  iso.map_id = "L1";
  iso.vehicle_type_node_properties.push_back(
    {"v1", std::nullopt, std::nullopt});
  lif.layouts[0].nodes.push_back(iso);
  auto g = Graph::from_lif(lif);

  auto unconnected = g->unconnected_nodes();
  ASSERT_EQ(unconnected.size(), 1u);
  EXPECT_EQ(unconnected[0], "N_iso");
}

TEST(GraphTest, UnconnectedNodes_StationReferencedNotListed)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_TRUE(g->unconnected_nodes().empty());
}

TEST(GraphTest, OperatorEq_SameLIF_True)
{
  auto a = Graph::from_lif(make_minimal_lif());
  auto b = Graph::from_lif(make_minimal_lif());
  EXPECT_EQ(*a, *b);
  EXPECT_FALSE(*a != *b);
}

TEST(GraphTest, OperatorEq_DifferentLIF_False)
{
  auto a = Graph::from_lif(make_minimal_lif());
  LIF lif_b = make_minimal_lif();
  lif_b.meta_information.lif_version = "0.12.0";
  auto b = Graph::from_lif(lif_b);
  EXPECT_FALSE(*a == *b);
  EXPECT_TRUE(*a != *b);
}

TEST(GraphTest, OperatorEq_SameNodesDifferentLayoutOrder_False)
{
  // Equality is structural (order-sensitive): same nodes across layouts in
  // different orders are NOT equal.
  LIF lif_a = make_multi_layout_lif();
  LIF lif_b = make_multi_layout_lif();
  std::swap(lif_b.layouts[0], lif_b.layouts[1]);
  EXPECT_FALSE(*Graph::from_lif(lif_a) == *Graph::from_lif(lif_b));
}

TEST(GraphTest, DumpDot_OutputContainsAllNodesAndEdges)
{
  auto g = Graph::from_lif(make_minimal_lif());
  std::ostringstream os;
  g->dump_dot(os);
  const std::string out = os.str();
  EXPECT_NE(out.find("digraph G"), std::string::npos);
  EXPECT_NE(out.find("\"N0\""), std::string::npos);
  EXPECT_NE(out.find("\"N1\""), std::string::npos);
  EXPECT_NE(out.find("\"N0\" -> \"N1\""), std::string::npos);
}

TEST(GraphTest, DumpDot_EdgeLabels_AreEdgeIds)
{
  auto g = Graph::from_lif(make_minimal_lif());
  std::ostringstream os;
  g->dump_dot(os);
  EXPECT_NE(os.str().find("label=\"E1\""), std::string::npos);
}

TEST(GraphTest, DumpDot_MultiEdge_BothEdgesRendered)
{
  LIF lif = make_minimal_lif();
  Edge e2;
  e2.edge_id = "E2";
  e2.start_node_id = "N0";
  e2.end_node_id = "N1";
  VehicleTypeEdgeProperty p2;
  p2.vehicle_type_id = "v2";
  p2.rotation_allowed = false;
  e2.vehicle_type_edge_properties.push_back(p2);
  lif.layouts[0].edges.push_back(e2);

  auto g = Graph::from_lif(lif);
  std::ostringstream os;
  g->dump_dot(os);
  EXPECT_NE(os.str().find("label=\"E1\""), std::string::npos);
  EXPECT_NE(os.str().find("label=\"E2\""), std::string::npos);
}

TEST(GraphTest, FromLoadedLif_SampleLayout_Loads)
{
#ifdef VDA5050_CORE__SAMPLE_LAYOUT_PATH
  auto r = load_from_file(VDA5050_CORE__SAMPLE_LAYOUT_PATH);
  ASSERT_TRUE(static_cast<bool>(r));
  auto g = Graph::from_lif(std::move(r.lif).value());
  EXPECT_FALSE(g->empty());
  EXPECT_GT(g->node_count(), 0u);
  EXPECT_EQ(g->lif().meta_information.lif_version, "0.11.0");
#else
  GTEST_SKIP() << "VDA5050_CORE__SAMPLE_LAYOUT_PATH not defined";
#endif
}

TEST(GraphTest, AddNode_Appends_AndIndexes)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Node n;
  n.node_id = "N_new";
  n.map_id = "L1";
  n.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  g->add_node(n, "L1");
  EXPECT_TRUE(g->has_node("N_new"));
  EXPECT_EQ(g->node_count(), 3u);
}

TEST(GraphTest, AddNode_DuplicateId_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Node n;
  n.node_id = "N0";  // duplicate
  n.map_id = "L1";
  EXPECT_THROW(g->add_node(n, "L1"), std::invalid_argument);
}

TEST(GraphTest, AddNode_UnknownLayout_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Node n;
  n.node_id = "N_new";
  EXPECT_THROW(g->add_node(n, "UNKNOWN_LAYOUT"), std::invalid_argument);
}

TEST(GraphTest, AddEdge_AppendsAndAdjacencyUpdated)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Edge e;
  e.edge_id = "E2";
  e.start_node_id = "N1";
  e.end_node_id = "N0";
  VehicleTypeEdgeProperty p;
  p.vehicle_type_id = "v1";
  p.rotation_allowed = false;
  e.vehicle_type_edge_properties.push_back(p);
  g->add_edge(e, "L1");
  EXPECT_TRUE(g->has_edge("E2"));
  const auto& out_n1 = g->outbound_edges("N1");
  EXPECT_TRUE(std::find(out_n1.begin(), out_n1.end(), "E2") != out_n1.end());
}

TEST(GraphTest, AddEdge_StartNotInLayout_Throws)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  Edge e;
  e.edge_id = "E_bad";
  e.start_node_id = "N0";
  e.end_node_id = "N5";
  EXPECT_THROW(g->add_edge(e, "LB"), std::invalid_argument);
}

TEST(GraphTest, AddEdge_EndUnknown_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Edge e;
  e.edge_id = "E_bad";
  e.start_node_id = "N0";
  e.end_node_id = "GHOST";
  EXPECT_THROW(g->add_edge(e, "L1"), std::invalid_argument);
}

TEST(GraphTest, AddEdge_UnknownLayout_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Edge e;
  e.edge_id = "E_bad";
  e.start_node_id = "N0";
  e.end_node_id = "N1";
  EXPECT_THROW(g->add_edge(e, "UNKNOWN_LAYOUT"), std::invalid_argument);
}

TEST(GraphTest, AddStation_UnknownLayout_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Station s;
  s.station_id = "S_new";
  s.interaction_node_ids = {"N0"};
  EXPECT_THROW(g->add_station(s, "UNKNOWN_LAYOUT"), std::invalid_argument);
}

TEST(GraphTest, DeleteNode_NoRefs_Succeeds)
{
  LIF lif = make_minimal_lif();
  Node iso;
  iso.node_id = "N_iso";
  iso.map_id = "L1";
  iso.vehicle_type_node_properties.push_back(
    {"v1", std::nullopt, std::nullopt});
  lif.layouts[0].nodes.push_back(iso);
  auto g = Graph::from_lif(lif);
  g->delete_node("N_iso");
  EXPECT_FALSE(g->has_node("N_iso"));
}

TEST(GraphTest, DeleteNode_ReferencedByEdge_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_THROW(g->delete_node("N0"), std::invalid_argument);
}

TEST(GraphTest, DeleteNode_ReferencedByStation_Throws)
{
  auto g = Graph::from_lif(make_station_only_lif());
  EXPECT_THROW(g->delete_node("N0"), std::invalid_argument);
}

TEST(GraphTest, DeleteEdge_Removes_AndAdjacencyUpdated)
{
  auto g = Graph::from_lif(make_minimal_lif());
  g->delete_edge("E1");
  EXPECT_FALSE(g->has_edge("E1"));
  EXPECT_TRUE(g->outbound_edges("N0").empty());
  EXPECT_TRUE(g->inbound_edges("N1").empty());
}

TEST(GraphTest, DeleteStation_Removes)
{
  auto g = Graph::from_lif(make_minimal_lif());
  g->delete_station("S1");
  EXPECT_FALSE(g->has_station("S1"));
  EXPECT_EQ(g->station_count(), 0u);
}

TEST(GraphTest, UpdateNodeId_RewritesNode_AndAllRefs)
{
  auto g = Graph::from_lif(make_minimal_lif());
  g->update_node_id("N0", "N_renamed");
  EXPECT_FALSE(g->has_node("N0"));
  EXPECT_TRUE(g->has_node("N_renamed"));
  EXPECT_EQ(g->get_edge("E1").start_node_id, "N_renamed");
  EXPECT_EQ(g->get_station("S1").interaction_node_ids[0], "N_renamed");
}

TEST(GraphTest, UpdateNodeId_DuplicateNewId_Throws)
{
  auto g = Graph::from_lif(make_minimal_lif());
  EXPECT_THROW(g->update_node_id("N0", "N1"), std::invalid_argument);
}

TEST(GraphTest, UpdateEdgeId_Simple)
{
  auto g = Graph::from_lif(make_minimal_lif());
  g->update_edge_id("E1", "E_renamed");
  EXPECT_FALSE(g->has_edge("E1"));
  EXPECT_TRUE(g->has_edge("E_renamed"));
}

TEST(GraphTest, Prune_RemovesIsolatedNodes)
{
  LIF lif = make_minimal_lif();
  Node iso;
  iso.node_id = "N_iso";
  iso.map_id = "L1";
  iso.vehicle_type_node_properties.push_back(
    {"v1", std::nullopt, std::nullopt});
  lif.layouts[0].nodes.push_back(iso);
  auto g = Graph::from_lif(lif);

  auto removed = g->prune();
  ASSERT_EQ(removed.size(), 1u);
  EXPECT_EQ(removed[0], "N_iso");
  EXPECT_FALSE(g->has_node("N_iso"));
}

TEST(GraphTest, Prune_KeepsStationReferencedNodes)
{
  auto g = Graph::from_lif(make_station_only_lif());
  EXPECT_TRUE(g->prune().empty());
  EXPECT_TRUE(g->has_node("N0"));
}

TEST(GraphTest, Mutation_AfterAddDeleteAdd_StateConsistent)
{
  auto g = Graph::from_lif(make_minimal_lif());
  Node n;
  n.node_id = "N_tmp";
  n.map_id = "L1";
  n.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  g->add_node(n, "L1");
  g->delete_node("N_tmp");
  g->add_node(n, "L1");
  EXPECT_TRUE(g->has_node("N_tmp"));
  const auto& out_n0 = g->outbound_edges("N0");
  ASSERT_EQ(out_n0.size(), 1u);
  EXPECT_EQ(out_n0[0], "E1");
}

TEST(GraphTest, DeleteNode_NonLast_SwappedNodeStillResolves)
{
  LIF lif = make_minimal_lif();
  for (const char* id : {"NA", "NB"})
  {
    Node n;
    n.node_id = id;
    n.map_id = "L1";
    n.vehicle_type_node_properties.push_back(
      {"v1", std::nullopt, std::nullopt});
    lif.layouts[0].nodes.push_back(n);
  }
  auto g = Graph::from_lif(lif);  // nodes: N0, N1, NA, NB
  // NA is not last, so NB is swapped into its slot; NB must still resolve.
  g->delete_node("NA");
  EXPECT_FALSE(g->has_node("NA"));
  ASSERT_TRUE(g->has_node("NB"));
  EXPECT_EQ(g->get_node("NB").node_id, "NB");
  EXPECT_EQ(g->node_count(), 3u);
}

TEST(GraphTest, DeleteLastElement_NodeEdgeStation_Consistent)
{
  auto g = Graph::from_lif(make_minimal_lif());  // N0, N1; E1; S1
  g->delete_station("S1");
  EXPECT_EQ(g->station_count(), 0u);
  g->delete_edge("E1");
  EXPECT_EQ(g->edge_count(), 0u);
  g->delete_node("N1");  // last element in the node vector
  g->delete_node("N0");  // sole remaining element
  EXPECT_EQ(g->node_count(), 0u);
  EXPECT_TRUE(g->empty());
}

TEST(GraphTest, SelfLoopEdge_RenameNode_RewritesBothEndpoints)
{
  LIF lif = make_minimal_lif();
  Edge loop;
  loop.edge_id = "E_loop";
  loop.start_node_id = "N1";
  loop.end_node_id = "N1";  // self-loop
  VehicleTypeEdgeProperty p;
  p.vehicle_type_id = "v1";
  p.rotation_allowed = false;
  loop.vehicle_type_edge_properties.push_back(p);
  lif.layouts[0].edges.push_back(loop);
  auto g = Graph::from_lif(lif);

  g->update_node_id("N1", "N1x");
  EXPECT_EQ(g->get_edge("E_loop").start_node_id, "N1x");
  EXPECT_EQ(g->get_edge("E_loop").end_node_id, "N1x");
  EXPECT_EQ(g->outbound_edges("N1x").size(), 1u);  // E_loop
  EXPECT_EQ(g->inbound_edges("N1x").size(), 2u);   // E1, E_loop

  g->delete_edge("E_loop");
  EXPECT_FALSE(g->has_edge("E_loop"));
  EXPECT_EQ(g->outbound_edges("N1x").size(), 0u);
  EXPECT_EQ(g->inbound_edges("N1x").size(), 1u);  // E1 remains
}

TEST(GraphTest, UpdateNodeId_CrossLayoutReferencedNode_RewritesEdge)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  // N5 lives in LB but is the end of E_xfer, whose start N1 is in LA.
  g->update_node_id("N5", "N5x");
  EXPECT_FALSE(g->has_node("N5"));
  EXPECT_TRUE(g->has_node("N5x"));
  EXPECT_EQ(g->get_edge("E_xfer").end_node_id, "N5x");
  EXPECT_EQ(g->inbound_edges("N5x").size(), 1u);
}

TEST(GraphTest, DeleteNode_CrossLayoutReferenced_ThrowsThenSucceeds)
{
  auto g = Graph::from_lif(make_multi_layout_lif());
  EXPECT_THROW(g->delete_node("N5"), std::invalid_argument);
  g->delete_edge("E_xfer");
  g->delete_node("N5");
  EXPECT_FALSE(g->has_node("N5"));
}

TEST(GraphTest, UpdateNodeId_IsolatedNode_RemainsUnconnected)
{
  LIF lif = make_minimal_lif();
  Node iso;
  iso.node_id = "N_iso";
  iso.map_id = "L1";
  iso.vehicle_type_node_properties.push_back(
    {"v1", std::nullopt, std::nullopt});
  lif.layouts[0].nodes.push_back(iso);
  auto g = Graph::from_lif(lif);

  g->update_node_id("N_iso", "N_iso2");
  auto unconnected = g->unconnected_nodes();
  ASSERT_EQ(unconnected.size(), 1u);
  EXPECT_EQ(unconnected[0], "N_iso2");
  EXPECT_TRUE(g->outbound_edges("N_iso2").empty());
}

TEST(GraphTest, UpdateStationId_ThenRenameReferencedNode_Consistent)
{
  auto g = Graph::from_lif(make_minimal_lif());  // S1 references N0
  g->update_station_id("S1", "S1x");
  EXPECT_TRUE(g->has_station("S1x"));
  // The reverse index must track the station rename so renaming N0 still
  // updates S1x's interaction node.
  g->update_node_id("N0", "N0x");
  ASSERT_FALSE(g->get_station("S1x").interaction_node_ids.empty());
  EXPECT_EQ(g->get_station("S1x").interaction_node_ids[0], "N0x");
}

}  // namespace vda5050_core::layout::test
