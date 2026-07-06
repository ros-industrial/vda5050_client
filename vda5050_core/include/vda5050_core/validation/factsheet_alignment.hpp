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

#ifndef VDA5050_CORE__VALIDATION__FACTSHEET_ALIGNMENT_HPP_
#define VDA5050_CORE__VALIDATION__FACTSHEET_ALIGNMENT_HPP_

#include "vda5050_core/errors/validation_result.hpp"
#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/types/factsheet.hpp"

namespace vda5050_core {

namespace validation {

/// \brief Advisory check: edge speeds in `graph` against the AGV factsheet's
/// speed envelope. Stateless; safe to call concurrently.
///
/// \param graph      the master's loaded layout
/// \param factsheet  the AGV's reported factsheet
/// \return WARNING-level entries — one per detected mismatch, or a single
///         entry noting the check was skipped when the factsheet reports no
///         usable speed. No entries means full alignment
vda5050_core::errors::ValidationResult check_factsheet_alignment(
  const vda5050_core::layout::Graph& graph,
  const vda5050_core::types::Factsheet& factsheet);

}  // namespace validation
}  // namespace vda5050_core

#endif  // VDA5050_CORE__VALIDATION__FACTSHEET_ALIGNMENT_HPP_
