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

#ifndef VDA5050_CORE__LAYOUT__LIF_HPP_
#define VDA5050_CORE__LAYOUT__LIF_HPP_

#include <string>
#include <vector>

#include "vda5050_core/layout/layout.hpp"

namespace vda5050_core::layout {

struct MetaInformation
{
  std::string project_identification;
  std::string creator;
  std::string export_timestamp;  ///< ISO 8601 UTC
  std::string lif_version;       ///< e.g. "0.11.0"

  inline bool operator==(const MetaInformation& other) const
  {
    return this->project_identification == other.project_identification &&
           this->creator == other.creator &&
           this->export_timestamp == other.export_timestamp &&
           this->lif_version == other.lif_version;
  }
  inline bool operator!=(const MetaInformation& other) const
  {
    return !(this->operator==(other));
  }
};

struct LIF
{
  MetaInformation meta_information;
  std::vector<Layout> layouts;

  inline bool operator==(const LIF& other) const
  {
    return this->meta_information == other.meta_information &&
           this->layouts == other.layouts;
  }
  inline bool operator!=(const LIF& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__LIF_HPP_
