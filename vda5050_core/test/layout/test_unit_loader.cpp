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

#include <unistd.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

#include "vda5050_core/json_utils/layout_serialization.hpp"
#include "vda5050_core/layout/layout_loader.hpp"

namespace vda5050_core::layout::test {

namespace {

nlohmann::json minimal_valid_json()
{
  return nlohmann::json::parse(R"({
    "metaInformation": {
      "projectIdentification": "p",
      "creator": "c",
      "exportTimestamp": "2026-01-01T00:00:00Z",
      "lifVersion": "0.11.0"
    },
    "layouts": [{
      "layoutId": "L1",
      "layoutVersion": "1",
      "nodes": [
        {"nodeId":"N0","mapId":"L1","nodePosition":{"x":0.0,"y":0.0},
         "vehicleTypeNodeProperties":[{"vehicleTypeId":"v1"}]},
        {"nodeId":"N1","mapId":"L1","nodePosition":{"x":5.0,"y":0.0},
         "vehicleTypeNodeProperties":[{"vehicleTypeId":"v1"}]}
      ],
      "edges": [
        {"edgeId":"E1","startNodeId":"N0","endNodeId":"N1",
         "vehicleTypeEdgeProperties":[{"vehicleTypeId":"v1","rotationAllowed":false}]}
      ],
      "stations": [
        {"stationId":"S1","interactionNodeIds":["N0"]}
      ]
    }]
  })");
}

bool has_error_of_type(const LayoutLoadResult& r, LayoutLoadErrorType t)
{
  if (r) return false;
  return std::any_of(r.errors.begin(), r.errors.end(), [&](const auto& e) {
    return e.type == t;
  });
}

}  // namespace

TEST(LayoutLoaderTest, LoadFromFile_NotFound_ReturnsFileNotFound)
{
  auto r = load_from_file("/nonexistent/path/to/layout.json");
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::FILE_NOT_FOUND));
}

TEST(LayoutLoaderTest, LoadFromJson_TopLevelNotObject_ReturnsParseError)
{
  auto r = load_from_json(nlohmann::json::array());
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::JSON_PARSE_ERROR));
}

TEST(LayoutLoaderTest, LoadFromFile_MalformedJson_ReturnsParseError)
{
  const std::string path = std::string(testing::TempDir()) +
                           "/vda5050_malformed_layout_" +
                           std::to_string(::getpid()) + ".json";
  {
    std::ofstream out(path);
    out << R"({ "metaInformation": { "projectIdentification": )";
  }
  auto r = load_from_file(path);
  std::remove(path.c_str());
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::JSON_PARSE_ERROR));
}

TEST(LayoutLoaderTest, LoadFromJson_Minimal_HappyPath)
{
  auto r = load_from_json(minimal_valid_json());
  ASSERT_TRUE(static_cast<bool>(r));
  ASSERT_EQ(r.lif.value().layouts.size(), 1u);
}

TEST(LayoutLoaderTest, LoadFromJson_LenientOnUnknownFields)
{
  auto j = minimal_valid_json();
  j["unknownTopLevel"] = "future-compat";
  j["layouts"][0]["unknownLayoutField"] = 42;
  EXPECT_TRUE(static_cast<bool>(load_from_json(j)));
}

TEST(LayoutLoaderTest, LoadFromJson_AbsentOptionalsStayNullopt)
{
  auto r = load_from_json(minimal_valid_json());
  ASSERT_TRUE(static_cast<bool>(r));
  const auto& vtep =
    r.lif.value().layouts[0].edges[0].vehicle_type_edge_properties[0];
  EXPECT_FALSE(vtep.orientation_type.has_value());
}

TEST(LayoutLoaderTest, LoadFromJson_MissingMetaInformation_ReturnsMissingField)
{
  auto j = minimal_valid_json();
  j.erase("metaInformation");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(
    has_error_of_type(r, LayoutLoadErrorType::MISSING_REQUIRED_FIELD));
}

TEST(LayoutLoaderTest, LoadFromJson_UnknownOrientationType_ReturnsInvalidValue)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]
   ["orientationType"] = "UPSIDE_DOWN";
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::INVALID_FIELD_VALUE));
}

TEST(LayoutLoaderTest, LoadFromJson_NonStringLifVersion_ReturnsInvalidValue)
{
  auto j = minimal_valid_json();
  j["metaInformation"]["lifVersion"] = 11;
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::INVALID_FIELD_VALUE));
}

TEST(LayoutLoaderTest, LoadFromJson_DuplicateNodeId_ReturnsDuplicateId)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["nodes"][1]["nodeId"] = "N0";  // collides with N0
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::DUPLICATE_ID));
}

TEST(LayoutLoaderTest, LoadFromJson_EmptyVehicleTypeNodeProperties)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["nodes"][0]["vehicleTypeNodeProperties"] =
    nlohmann::json::array();
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY));
}

TEST(LayoutLoaderTest, LoadFromJson_EmptyVehicleTypeEdgeProperties)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"] =
    nlohmann::json::array();
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY));
}

TEST(LayoutLoaderTest, LoadFromJson_EdgeEndNodeIdDangling_ReturnsDanglingRef)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["endNodeId"] = "GHOST";
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::DANGLING_NODE_REF));
}

TEST(LayoutLoaderTest, LoadFromJson_StationDanglingInteractionNodeId)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["stations"] = nlohmann::json::parse(R"([
    {"stationId":"S1","interactionNodeIds":["GHOST"]}
  ])");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::DANGLING_NODE_REF));
}

TEST(LayoutLoaderTest, LoadFromJson_StationEmptyInteractionNodeIds)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["stations"] = nlohmann::json::parse(R"([
    {"stationId":"S1","interactionNodeIds":[]}
  ])");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY));
}

TEST(LayoutLoaderTest, LoadFromJson_TrajectorySizeMismatch)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]["trajectory"] =
    nlohmann::json::parse(R"({
      "knotVector": [0.0, 0.5, 1.0],
      "controlPoints": [{"x":0.0,"y":0.0},{"x":5.0,"y":0.0}]
    })");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(
    has_error_of_type(r, LayoutLoadErrorType::TRAJECTORY_SIZE_MISMATCH));
}

TEST(LayoutLoaderTest, LoadFromJson_EmptyLayouts_ReturnsEmptyArray)
{
  auto j = minimal_valid_json();
  j["layouts"] = nlohmann::json::array();
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY));
}

TEST(LayoutLoaderTest, LoadFromJson_TrajectoryDegreeZero_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]["trajectory"] =
    nlohmann::json::parse(R"({
      "degree": 0,
      "knotVector": [0.0, 0.5, 1.0],
      "controlPoints": [{"x":0.0,"y":0.0},{"x":5.0,"y":0.0}]
    })");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(LayoutLoaderTest, LoadFromJson_KnotVectorOutOfRange_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]["trajectory"] =
    nlohmann::json::parse(R"({
      "knotVector": [0.0, 0.5, 1.5, 2.0],
      "controlPoints": [{"x":0.0,"y":0.0},{"x":5.0,"y":0.0}],
      "degree": 1
    })");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(LayoutLoaderTest, LoadFromJson_KnotVectorNotMonotonic_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]["trajectory"] =
    nlohmann::json::parse(R"({
      "knotVector": [0.0, 0.7, 0.3, 1.0],
      "controlPoints": [{"x":0.0,"y":0.0},{"x":5.0,"y":0.0}],
      "degree": 1
    })");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(
  LayoutLoaderTest, LoadFromJson_ControlPointWeightNegative_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]["trajectory"] =
    nlohmann::json::parse(R"({
      "knotVector": [0.0, 0.33, 0.66, 1.0],
      "controlPoints": [
        {"x":0.0,"y":0.0,"weight":-1.0},
        {"x":5.0,"y":0.0,"weight":1.0}
      ],
      "degree": 1
    })");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(LayoutLoaderTest, LoadFromJson_StationHeightNegative_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["stations"] = nlohmann::json::parse(R"([
    {"stationId":"S1","interactionNodeIds":["N0"],"stationHeight":-0.5}
  ])");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(LayoutLoaderTest, LoadFromJson_StationThetaOutOfRange_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["stations"] = nlohmann::json::parse(R"([
    {
      "stationId":"S1",
      "interactionNodeIds":["N0"],
      "stationPosition":{"x":0.0,"y":0.0,"theta":5.0}
    }
  ])");
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(LayoutLoaderTest, LoadFromJson_NodeThetaOutOfRange_ReturnsOutOfRange)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["nodes"][0]["vehicleTypeNodeProperties"][0]["theta"] = 4.0;
  auto r = load_from_json(j);
  EXPECT_FALSE(static_cast<bool>(r));
  EXPECT_TRUE(has_error_of_type(r, LayoutLoadErrorType::OUT_OF_RANGE));
}

TEST(LayoutLoaderTest, Trajectory_HappyPath_RoundTrip)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]["trajectory"] =
    nlohmann::json::parse(R"({
      "knotVector": [0.0, 0.33, 0.66, 1.0],
      "controlPoints": [
        {"x": 0.0, "y": 0.0, "weight": 1.0},
        {"x": 5.0, "y": 0.0, "weight": 1.0}
      ],
      "degree": 1
    })");
  auto r = load_from_json(j);
  ASSERT_TRUE(static_cast<bool>(r));
  auto r2 = load_from_json(nlohmann::json(r.lif.value()));
  ASSERT_TRUE(static_cast<bool>(r2));
  EXPECT_EQ(r.lif.value(), r2.lif.value());
}

TEST(LayoutLoaderTest, LoadRestriction_RoundTrip)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0]
   ["loadRestriction"] = nlohmann::json::parse(R"({
      "unloaded": true,
      "loaded": false,
      "loadSetNames": ["pallet_eur", "pallet_us"]
    })");
  auto r = load_from_json(j);
  ASSERT_TRUE(static_cast<bool>(r));
  auto r2 = load_from_json(nlohmann::json(r.lif.value()));
  ASSERT_TRUE(static_cast<bool>(r2));
  EXPECT_EQ(r.lif.value(), r2.lif.value());
}

TEST(LayoutLoaderTest, StationPosition_RoundTrip)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["stations"] = nlohmann::json::parse(R"([
    {
      "stationId": "S1",
      "interactionNodeIds": ["N0"],
      "stationName": "Pickup A",
      "stationHeight": 1.2,
      "stationPosition": { "x": 0.0, "y": 0.0, "theta": 1.5708 }
    }
  ])");
  auto r = load_from_json(j);
  ASSERT_TRUE(static_cast<bool>(r));
  auto r2 = load_from_json(nlohmann::json(r.lif.value()));
  ASSERT_TRUE(static_cast<bool>(r2));
  EXPECT_EQ(r.lif.value(), r2.lif.value());
}

TEST(LayoutLoaderTest, Action_RoundTrip)
{
  auto j = minimal_valid_json();
  j["layouts"][0]["nodes"][0]["vehicleTypeNodeProperties"][0]["actions"] =
    nlohmann::json::parse(R"([
      {
        "actionType": "pick",
        "blockingType": "HARD",
        "actionDescription": "Pick a pallet",
        "requirementType": "CONDITIONAL",
        "actionParameters": [
          {"key": "loadId", "value": "L1"},
          {"key": "lhd",    "value": "fork"}
        ]
      }
    ])");
  auto r = load_from_json(j);
  ASSERT_TRUE(static_cast<bool>(r));
  auto r2 = load_from_json(nlohmann::json(r.lif.value()));
  ASSERT_TRUE(static_cast<bool>(r2));
  EXPECT_EQ(r.lif.value(), r2.lif.value());
}

TEST(LayoutLoaderTest, Enums_NonDefaultValues_RoundTrip)
{
  auto j = minimal_valid_json();
  auto& vtep = j["layouts"][0]["edges"][0]["vehicleTypeEdgeProperties"][0];
  vtep["orientationType"] = "GLOBAL";
  vtep["rotationAtStartNodeAllowed"] = "NONE";
  vtep["rotationAtEndNodeAllowed"] = "CW";
  j["layouts"][0]["nodes"][0]["vehicleTypeNodeProperties"][0]["actions"] =
    nlohmann::json::parse(R"([
      {"actionType":"x","blockingType":"SOFT","requirementType":"REQUIRED"}
    ])");
  auto r = load_from_json(j);
  ASSERT_TRUE(static_cast<bool>(r));
  auto r2 = load_from_json(nlohmann::json(r.lif.value()));
  ASSERT_TRUE(static_cast<bool>(r2));
  EXPECT_EQ(r.lif.value(), r2.lif.value());
}

TEST(LayoutLoaderTest, LoadFromFile_RoundTripSampleData)
{
#ifdef VDA5050_CORE__SAMPLE_LAYOUT_PATH
  auto r = load_from_file(VDA5050_CORE__SAMPLE_LAYOUT_PATH);
  ASSERT_TRUE(static_cast<bool>(r))
    << "Sample layout at " << VDA5050_CORE__SAMPLE_LAYOUT_PATH
    << " did not load";
  auto r2 = load_from_json(nlohmann::json(r.lif.value()));
  ASSERT_TRUE(static_cast<bool>(r2));
  EXPECT_EQ(r.lif.value(), r2.lif.value());
#else
  GTEST_SKIP() << "VDA5050_CORE__SAMPLE_LAYOUT_PATH not defined";
#endif
}

}  // namespace vda5050_core::layout::test
