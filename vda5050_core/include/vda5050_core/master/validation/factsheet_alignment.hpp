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

#ifndef VDA5050_CORE__MASTER__VALIDATION__FACTSHEET_ALIGNMENT_HPP_
#define VDA5050_CORE__MASTER__VALIDATION__FACTSHEET_ALIGNMENT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/types/factsheet.hpp"

namespace vda5050_core::master {

// Cross-checks a loaded layout against an AGV's factsheet so the integrator
// learns, before sending orders, whether the AGV can physically execute what
// the layout commands.
//
// Speed rule: "SpeedExceedsCapability" / "SpeedBelowMinimum" — for every edge's
// per-vehicle-type max_speed, compare against the factsheet's reported
// physical speed envelope (physical_parameters.speed_max / speed_min). The
// v2.0.0 factsheet carries no vehicle_type_id, so the check cannot resolve
// which vehicle-type lane applies to this AGV; it reports every lane and names
// the vehicle_type_id in the finding, leaving relevance to the integrator.
// Findings are advisory (WARNING) and never block onboarding.
//
// Deferred (no v2.0.0 factsheet field to check against): footprint-fits-edge,
// localization tolerance, mapId / mapVersion match.

/// \brief Severity of an alignment finding.
enum class AlignmentSeverity : std::uint8_t
{
  WARNING = 0,
  ERROR = 1,
};

/// \brief One mismatch from check_factsheet_alignment. `code` is a stable
/// identifier; `description` is human-readable and embeds the offending
/// values.
struct AlignmentFinding
{
  AlignmentSeverity severity;  ///< advisory vs blocking
  std::string code;            ///< stable finding identifier
  std::string description;     ///< human-readable detail with values
};

/// \brief Aggregated result of one factsheet-against-layout alignment check.
/// An empty `findings` vector means the AGV is fully aligned.
struct FactsheetAlignmentResult
{
  std::vector<AlignmentFinding> findings;  ///< one entry per mismatch

  /// \brief True iff any finding has severity ERROR.
  bool has_error() const;
};

/// \brief Run alignment checks for `factsheet` against `graph`. Stateless;
/// safe to call concurrently.
///
/// \param graph      the master's loaded layout
/// \param factsheet  the AGV's reported factsheet
/// \return one finding per detected mismatch; empty means full alignment
FactsheetAlignmentResult check_factsheet_alignment(
  const vda5050_core::layout::Graph& graph,
  const vda5050_core::types::Factsheet& factsheet);

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__VALIDATION__FACTSHEET_ALIGNMENT_HPP_
