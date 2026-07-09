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

#include "vda5050_core/validation/predefined_action_types.hpp"

#include <set>
#include <string>

namespace vda5050_core::validation {

bool is_capability_exempt_action_type(const std::string& action_type)
{
  // Charging is capability-dependent, so it's NOT here (normal check applies).
  static const std::set<std::string> kExempt = {
    "stateRequest", "factsheetRequest", "cancelOrder", "initPosition",
    "startPause",   "stopPause",        "logReport"};
  return kExempt.count(action_type) != 0;
}

bool is_motion_exempt_action_type(const std::string& action_type)
{
  static const std::set<std::string> kExempt = {
    "cancelOrder", "startPause", "stopPause"};
  return kExempt.count(action_type) != 0;
}

bool is_position_init_action_type(const std::string& action_type)
{
  return action_type == "initPosition";
}

}  // namespace vda5050_core::validation
