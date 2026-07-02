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

#ifndef VDA5050_CORE__LAYOUT__VALIDATE_LAYOUT_HPP_
#define VDA5050_CORE__LAYOUT__VALIDATE_LAYOUT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "vda5050_core/layout/lif.hpp"

namespace vda5050_core::layout {

enum class LayoutLoadErrorType : std::uint8_t
{
  FILE_NOT_FOUND,
  FILE_READ_FAILED,
  JSON_PARSE_ERROR,
  MISSING_REQUIRED_FIELD,
  INVALID_FIELD_VALUE,
  DUPLICATE_ID,
  DANGLING_NODE_REF,
  EMPTY_REQUIRED_ARRAY,
  TRAJECTORY_SIZE_MISMATCH,
  OUT_OF_RANGE,
};

struct LayoutLoadError
{
  LayoutLoadErrorType type;
  std::string description;
};

/// \brief Check LIF structural invariants.
///
/// \param lif Layout to validate.
/// \return Errors found; empty if the layout is valid.
std::vector<LayoutLoadError> validate_layout(const LIF& lif);

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__VALIDATE_LAYOUT_HPP_
