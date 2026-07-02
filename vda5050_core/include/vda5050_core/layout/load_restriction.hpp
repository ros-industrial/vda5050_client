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

#ifndef VDA5050_CORE__LAYOUT__LOAD_RESTRICTION_HPP_
#define VDA5050_CORE__LAYOUT__LOAD_RESTRICTION_HPP_

#include <optional>
#include <string>
#include <vector>

namespace vda5050_core::layout {

/// \brief Load-traversability rule on an edge (per vehicle-type bucket).
///
/// When load_set_names is absent/empty, the edge is usable for all load sets
/// the vehicle declares in its factsheet.
struct LoadRestriction
{
  bool unloaded = true;  ///< edge usable when vehicle carries no load
  bool loaded = true;    ///< edge usable when vehicle carries a load
  std::optional<std::vector<std::string>> load_set_names;

  inline bool operator==(const LoadRestriction& other) const
  {
    return this->unloaded == other.unloaded && this->loaded == other.loaded &&
           this->load_set_names == other.load_set_names;
  }
  inline bool operator!=(const LoadRestriction& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__LOAD_RESTRICTION_HPP_
