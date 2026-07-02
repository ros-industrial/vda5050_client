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

#include "vda5050_core/master/validation/factsheet_alignment.hpp"

#include <algorithm>
#include <sstream>
#include <string>

#include "vda5050_core/layout/edge.hpp"

namespace vda5050_core::master {

bool FactsheetAlignmentResult::has_error() const
{
  return std::any_of(findings.begin(), findings.end(), [](const auto& f) {
    return f.severity == AlignmentSeverity::ERROR;
  });
}

FactsheetAlignmentResult check_factsheet_alignment(
  const vda5050_core::layout::Graph& graph,
  const vda5050_core::types::Factsheet& factsheet)
{
  FactsheetAlignmentResult result;

  const double agv_speed_max = factsheet.physical_parameters.speed_max;
  const double agv_speed_min = factsheet.physical_parameters.speed_min;

  // No usable speed capability reported (default-constructed or factsheet not
  // yet received) — skip rather than flag every edge.
  if (agv_speed_max <= 0.0)
  {
    result.findings.push_back(
      {AlignmentSeverity::WARNING, "SpeedCapabilityUnknown",
       "AGV factsheet reports no usable physical_parameters.speed_max; "
       "edge speed alignment was not checked."});
    return result;
  }

  graph.for_each_edge_ordered([&](const vda5050_core::layout::Edge& edge) {
    for (const auto& prop : edge.vehicle_type_edge_properties)
    {
      if (!prop.max_speed.has_value()) continue;
      const double edge_max = prop.max_speed.value();
      if (edge_max > agv_speed_max)
      {
        std::ostringstream oss;
        oss << "Edge '" << edge.edge_id << "' (vehicle_type '"
            << prop.vehicle_type_id << "') max_speed=" << edge_max
            << " m/s exceeds AGV factsheet physical_parameters.speed_max="
            << agv_speed_max << " m/s.";
        result.findings.push_back(
          {AlignmentSeverity::WARNING, "SpeedExceedsCapability", oss.str()});
      }
      if (agv_speed_min > 0.0 && edge_max < agv_speed_min)
      {
        std::ostringstream oss;
        oss << "Edge '" << edge.edge_id << "' (vehicle_type '"
            << prop.vehicle_type_id << "') max_speed=" << edge_max
            << " m/s is below AGV factsheet physical_parameters.speed_min="
            << agv_speed_min << " m/s.";
        result.findings.push_back(
          {AlignmentSeverity::WARNING, "SpeedBelowMinimum", oss.str()});
      }
    }
  });

  return result;
}

}  // namespace vda5050_core::master
