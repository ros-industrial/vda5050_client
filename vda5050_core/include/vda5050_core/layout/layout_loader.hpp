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

#ifndef VDA5050_CORE__LAYOUT__LAYOUT_LOADER_HPP_
#define VDA5050_CORE__LAYOUT__LAYOUT_LOADER_HPP_

#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_core/layout/lif.hpp"
#include "vda5050_core/layout/validate_layout.hpp"

namespace vda5050_core::layout {

/// \brief Result of a layout load: a valid LIF, or the load errors.
///
/// The loader populates exactly one side — on success `lif` holds the LIF and
/// `errors` is empty; on failure `lif` is empty and `errors` is non-empty.
/// Check via `operator bool` (or `lif`) first, and access the LIF through
/// `lif.value()` so a missed check throws rather than being undefined.
struct LayoutLoadResult
{
  std::optional<LIF> lif;
  std::vector<LayoutLoadError> errors;

  /// \brief True iff a LIF was loaded (so lif.value() is safe to use).
  explicit operator bool() const noexcept
  {
    return lif.has_value() && errors.empty();
  }
};

/// \brief Load a single LIF, validating it; the result holds the LIF or errors.
///
/// Node, edge, station, and layout IDs are required globally unique across the
/// loaded LIF. Multi-vendor reconciliation (where different vendor LIFs may
/// legitimately reuse IDs and must be remapped by the master) is out of scope;
/// loading several vendor LIFs as one will yield DUPLICATE_ID errors by design.
///
/// \param path Path to the LIF JSON file (load_from_json takes a parsed value).
/// \return Result holding the validated LIF, or the errors found.
LayoutLoadResult load_from_file(const std::string& path);
LayoutLoadResult load_from_json(const nlohmann::json& json);

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__LAYOUT_LOADER_HPP_
