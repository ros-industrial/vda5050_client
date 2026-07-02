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

#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "vda5050_core/layout/validate_layout.hpp"

namespace vda5050_core::layout {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kDefaultTrajectoryDegree = 1;
constexpr double kKnotMin = 0.0;
constexpr double kKnotMax = 1.0;

void add(
  std::vector<LayoutLoadError>& errs, LayoutLoadErrorType t, std::string desc)
{
  errs.push_back({t, std::move(desc)});
}

bool in_closed_range(double v, double lo, double hi)
{
  return v >= lo && v <= hi;
}

void validate_trajectory(
  std::vector<LayoutLoadError>& errors, const std::string& edge_id,
  const Trajectory& t)
{
  const int degree = t.degree.value_or(kDefaultTrajectoryDegree);
  if (degree < 1)
  {
    add(
      errors, LayoutLoadErrorType::OUT_OF_RANGE,
      "Edge '" + edge_id + "' trajectory.degree (" + std::to_string(degree) +
        ") must be >= 1.");
    // Bail: the size/knot checks below all derive from degree and would just
    // emit derivative errors on top of the root cause.
    return;
  }
  if (t.control_points.size() < static_cast<std::size_t>(degree) + 1u)
  {
    add(
      errors, LayoutLoadErrorType::OUT_OF_RANGE,
      "Edge '" + edge_id + "' trajectory has fewer control points (" +
        std::to_string(t.control_points.size()) + ") than degree + 1.");
  }

  const std::size_t expected_knots =
    t.control_points.size() + static_cast<std::size_t>(degree) + 1u;
  if (t.knot_vector.size() != expected_knots)
  {
    add(
      errors, LayoutLoadErrorType::TRAJECTORY_SIZE_MISMATCH,
      "Edge '" + edge_id + "' trajectory: knotVector.size (" +
        std::to_string(t.knot_vector.size()) +
        ") != controlPoints.size + degree + 1 (" +
        std::to_string(expected_knots) + ").");
  }

  for (std::size_t k = 0; k < t.knot_vector.size(); ++k)
  {
    const double kv = t.knot_vector[k];
    if (!in_closed_range(kv, kKnotMin, kKnotMax))
    {
      add(
        errors, LayoutLoadErrorType::OUT_OF_RANGE,
        "Edge '" + edge_id + "' trajectory.knotVector[" + std::to_string(k) +
          "] = " + std::to_string(kv) + " is outside [0.0, 1.0].");
    }
    if (k > 0 && t.knot_vector[k] < t.knot_vector[k - 1])
    {
      add(
        errors, LayoutLoadErrorType::OUT_OF_RANGE,
        "Edge '" + edge_id +
          "' trajectory.knotVector is not non-decreasing"
          " at index " +
          std::to_string(k) + ".");
    }
  }

  for (std::size_t c = 0; c < t.control_points.size(); ++c)
  {
    const auto& cp = t.control_points[c];
    if (cp.weight && *cp.weight < 0.0)
    {
      add(
        errors, LayoutLoadErrorType::OUT_OF_RANGE,
        "Edge '" + edge_id + "' trajectory.controlPoints[" + std::to_string(c) +
          "].weight (" + std::to_string(*cp.weight) + ") must be >= 0.");
    }
  }
}

}  // namespace

std::vector<LayoutLoadError> validate_layout(const LIF& lif)
{
  std::vector<LayoutLoadError> errors;

  if (lif.layouts.empty())
  {
    add(
      errors, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY,
      "LIF.layouts must not be empty.");
    return errors;
  }

  std::unordered_set<std::string> seen_layout_ids;
  std::unordered_set<std::string> seen_node_ids;
  std::unordered_set<std::string> seen_edge_ids;
  std::unordered_set<std::string> seen_station_ids;

  for (const auto& layout : lif.layouts)
  {
    if (!seen_layout_ids.insert(layout.layout_id).second)
    {
      add(
        errors, LayoutLoadErrorType::DUPLICATE_ID,
        "Duplicate layoutId '" + layout.layout_id + "'.");
    }

    std::unordered_set<std::string> nodes_in_this_layout;
    for (const auto& node : layout.nodes)
    {
      if (!seen_node_ids.insert(node.node_id).second)
      {
        add(
          errors, LayoutLoadErrorType::DUPLICATE_ID,
          "Duplicate nodeId '" + node.node_id + "' (layout '" +
            layout.layout_id + "').");
      }
      nodes_in_this_layout.insert(node.node_id);

      if (node.vehicle_type_node_properties.empty())
      {
        add(
          errors, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY,
          "Node '" + node.node_id +
            "' vehicleTypeNodeProperties must not be empty.");
      }

      for (const auto& vtnp : node.vehicle_type_node_properties)
      {
        if (vtnp.theta && !in_closed_range(*vtnp.theta, -kPi, kPi))
        {
          add(
            errors, LayoutLoadErrorType::OUT_OF_RANGE,
            "Node '" + node.node_id + "' vehicleTypeNodeProperty(" +
              vtnp.vehicle_type_id + ").theta (" + std::to_string(*vtnp.theta) +
              ") is outside [-pi, pi].");
        }
      }
    }

    for (const auto& edge : layout.edges)
    {
      if (!seen_edge_ids.insert(edge.edge_id).second)
      {
        add(
          errors, LayoutLoadErrorType::DUPLICATE_ID,
          "Duplicate edgeId '" + edge.edge_id + "' (layout '" +
            layout.layout_id + "').");
      }
      if (edge.vehicle_type_edge_properties.empty())
      {
        add(
          errors, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY,
          "Edge '" + edge.edge_id +
            "' vehicleTypeEdgeProperties must not be empty.");
      }
      for (const auto& vtep : edge.vehicle_type_edge_properties)
      {
        if (vtep.trajectory.has_value())
        {
          validate_trajectory(errors, edge.edge_id, vtep.trajectory.value());
        }
      }

      if (
        nodes_in_this_layout.find(edge.start_node_id) ==
        nodes_in_this_layout.end())
      {
        add(
          errors, LayoutLoadErrorType::DANGLING_NODE_REF,
          "Edge '" + edge.edge_id + "' startNodeId '" + edge.start_node_id +
            "' is not in its own layout '" + layout.layout_id + "'.");
      }
    }

    for (const auto& station : layout.stations)
    {
      if (!seen_station_ids.insert(station.station_id).second)
      {
        add(
          errors, LayoutLoadErrorType::DUPLICATE_ID,
          "Duplicate stationId '" + station.station_id + "' (layout '" +
            layout.layout_id + "').");
      }
      if (station.interaction_node_ids.empty())
      {
        add(
          errors, LayoutLoadErrorType::EMPTY_REQUIRED_ARRAY,
          "Station '" + station.station_id +
            "' interactionNodeIds must not be empty.");
      }
      if (station.station_height && *station.station_height < 0.0)
      {
        add(
          errors, LayoutLoadErrorType::OUT_OF_RANGE,
          "Station '" + station.station_id + "' stationHeight (" +
            std::to_string(*station.station_height) + ") must be >= 0.");
      }
      if (
        station.station_position && station.station_position->theta &&
        !in_closed_range(*station.station_position->theta, -kPi, kPi))
      {
        add(
          errors, LayoutLoadErrorType::OUT_OF_RANGE,
          "Station '" + station.station_id + "' stationPosition.theta (" +
            std::to_string(*station.station_position->theta) +
            ") is outside [-pi, pi].");
      }
    }
  }

  // Cross-layout endNodeId resolution + station interactionNodeIds resolution.
  for (const auto& layout : lif.layouts)
  {
    for (const auto& edge : layout.edges)
    {
      if (seen_node_ids.find(edge.end_node_id) == seen_node_ids.end())
      {
        add(
          errors, LayoutLoadErrorType::DANGLING_NODE_REF,
          "Edge '" + edge.edge_id + "' endNodeId '" + edge.end_node_id +
            "' does not resolve to any node in the LIF.");
      }
    }
    for (const auto& station : layout.stations)
    {
      for (const auto& nid : station.interaction_node_ids)
      {
        if (seen_node_ids.find(nid) == seen_node_ids.end())
        {
          add(
            errors, LayoutLoadErrorType::DANGLING_NODE_REF,
            "Station '" + station.station_id + "' interactionNodeId '" + nid +
              "' does not resolve to any node in the LIF.");
        }
      }
    }
  }

  return errors;
}

}  // namespace vda5050_core::layout
