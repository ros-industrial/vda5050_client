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

#ifndef VDA5050_CORE__LAYOUT__LAYOUT_ACTION_HPP_
#define VDA5050_CORE__LAYOUT__LAYOUT_ACTION_HPP_

#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/types/blocking_type.hpp"

namespace vda5050_core::layout {

enum class RequirementType
{
  REQUIRED,
  CONDITIONAL,
  OPTIONAL,
};

struct ActionParameter
{
  std::string key;
  std::string value;

  inline bool operator==(const ActionParameter& other) const
  {
    return this->key == other.key && this->value == other.value;
  }
  inline bool operator!=(const ActionParameter& other) const
  {
    return !(this->operator==(other));
  }
};

/// \brief LIF action template (no `actionId`; the master generates ids when
/// integrating it into an order).
///
/// Forked from VDA5050 Action because of the missing `actionId` and the
/// LIF-only `requirementType`.
struct Action
{
  std::string action_type;
  vda5050_core::types::BlockingType blocking_type;
  std::optional<std::string> action_description;
  std::optional<RequirementType> requirement_type;
  std::optional<std::vector<ActionParameter>> action_parameters;

  inline bool operator==(const Action& other) const
  {
    return this->action_type == other.action_type &&
           this->blocking_type == other.blocking_type &&
           this->action_description == other.action_description &&
           this->requirement_type == other.requirement_type &&
           this->action_parameters == other.action_parameters;
  }
  inline bool operator!=(const Action& other) const
  {
    return !(this->operator==(other));
  }
};

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__LAYOUT_ACTION_HPP_
