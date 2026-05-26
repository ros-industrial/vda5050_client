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

#include "vda5050_core/layout/lif.hpp"
#include "vda5050_core/master/validation/factsheet_alignment.hpp"

namespace vda5050_core::master::test {

namespace {

using vda5050_core::layout::Edge;
using vda5050_core::layout::Graph;
using vda5050_core::layout::Layout;
using vda5050_core::layout::LIF;
using vda5050_core::layout::Node;
using vda5050_core::layout::VehicleTypeEdgeProperty;

Node make_node(const std::string& id)
{
  Node n;
  n.node_id = id;
  n.map_id = "L1";
  n.node_position = {0.0, 0.0};
  n.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  return n;
}

VehicleTypeEdgeProperty make_prop(
  const std::string& vehicle_type_id, std::optional<double> max_speed)
{
  VehicleTypeEdgeProperty p;
  p.vehicle_type_id = vehicle_type_id;
  p.max_speed = max_speed;
  return p;
}

// Graph with nodes N0, N1 on map "L1" and one directed edge N0 --E1--> N1
// carrying the supplied per-vehicle-type properties.
Graph::ConstPtr make_graph(std::vector<VehicleTypeEdgeProperty> edge_props)
{
  LIF lif;
  Layout layout;
  layout.layout_id = "L1";
  layout.nodes.push_back(make_node("N0"));
  layout.nodes.push_back(make_node("N1"));
  Edge e;
  e.edge_id = "E1";
  e.start_node_id = "N0";
  e.end_node_id = "N1";
  e.vehicle_type_edge_properties = std::move(edge_props);
  layout.edges.push_back(std::move(e));
  lif.layouts.push_back(std::move(layout));
  return Graph::from_lif(std::move(lif));
}

vda5050_core::types::Factsheet make_factsheet(
  double speed_max, double speed_min = 0.0)
{
  vda5050_core::types::Factsheet fs;
  fs.physical_parameters.speed_max = speed_max;
  fs.physical_parameters.speed_min = speed_min;
  return fs;
}

}  // namespace

TEST(FactsheetAlignment, AlignedEdge_NoFindings)
{
  auto graph = make_graph({make_prop("v1", 1.0)});
  auto res = check_factsheet_alignment(*graph, make_factsheet(2.0));
  EXPECT_TRUE(res.findings.empty());
  EXPECT_FALSE(res.has_error());
}

TEST(FactsheetAlignment, EdgeSpeedExceedsCapability_Warns)
{
  auto graph = make_graph({make_prop("v1", 3.0)});
  auto res = check_factsheet_alignment(*graph, make_factsheet(2.0));
  ASSERT_EQ(res.findings.size(), 1u);
  EXPECT_EQ(res.findings.front().code, "SpeedExceedsCapability");
  EXPECT_EQ(res.findings.front().severity, AlignmentSeverity::WARNING);
  EXPECT_NE(res.findings.front().description.find("v1"), std::string::npos);
  EXPECT_FALSE(res.has_error());
}

TEST(FactsheetAlignment, EdgeSpeedBelowMinimum_Warns)
{
  auto graph = make_graph({make_prop("v1", 0.2)});
  auto res = check_factsheet_alignment(*graph, make_factsheet(2.0, 0.5));
  ASSERT_EQ(res.findings.size(), 1u);
  EXPECT_EQ(res.findings.front().code, "SpeedBelowMinimum");
}

TEST(FactsheetAlignment, MultiVehicleType_ChecksEveryLane)
{
  // Two lanes on the edge: "v1" within capability, "v2" exceeds it. Both are
  // checked; only the exceeding lane is flagged, naming its vehicle_type_id.
  auto graph = make_graph({make_prop("v1", 1.0), make_prop("v2", 5.0)});
  auto res = check_factsheet_alignment(*graph, make_factsheet(2.0));
  ASSERT_EQ(res.findings.size(), 1u);
  EXPECT_EQ(res.findings.front().code, "SpeedExceedsCapability");
  EXPECT_NE(res.findings.front().description.find("v2"), std::string::npos);
}

TEST(FactsheetAlignment, NoMaxSpeedOnLane_Skipped)
{
  auto graph = make_graph({make_prop("v1", std::nullopt)});
  auto res = check_factsheet_alignment(*graph, make_factsheet(2.0));
  EXPECT_TRUE(res.findings.empty());
}

TEST(FactsheetAlignment, UnknownAgvCapability_SkipsAndWarnsOnce)
{
  auto graph = make_graph({make_prop("v1", 5.0)});
  auto res = check_factsheet_alignment(*graph, make_factsheet(0.0));
  ASSERT_EQ(res.findings.size(), 1u);
  EXPECT_EQ(res.findings.front().code, "SpeedCapabilityUnknown");
}

}  // namespace vda5050_core::master::test
