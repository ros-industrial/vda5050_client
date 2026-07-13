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

#ifndef VDA5050_CORE__MASTER__ACTIONS__ACTION_FACTORY_HPP_
#define VDA5050_CORE__MASTER__ACTIONS__ACTION_FACTORY_HPP_

#include <string>
#include <vector>

#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"
#include "vda5050_core/types/blocking_type.hpp"

namespace vda5050_core {
namespace master {

/// \brief Builds Action structs (custom + predefined state/factsheet/init
///        requests).
///
/// Set the wrapping InstantActions header (version/manufacturer/serial) before
/// assign/publish; validation checks it.
class ActionFactory
{
public:
  /// \brief Build an Action. Caller owns action_id (see generate_action_id()).
  static vda5050_core::types::Action build_custom(
    const std::string& action_type, const std::string& action_id,
    vda5050_core::types::BlockingType blocking_type =
      vda5050_core::types::BlockingType::NONE,
    const std::string& description = "",
    const std::vector<vda5050_core::types::ActionParameter>& parameters = {});

  /// \brief Generate a UUIDv4 action_id; use when there's no id to correlate.
  static std::string generate_action_id();

  /// \brief Build a stateRequest instantAction; the AGV replies with a State.
  static vda5050_core::types::Action build_state_request(
    const std::string& action_id, const std::string& description = "");

  /// \brief Build a factsheetRequest instantAction; AGV replies on factsheet.
  static vda5050_core::types::Action build_factsheet_request(
    const std::string& action_id, const std::string& description = "");

  /// \brief Build an initPosition instantAction that sets the AGV's pose.
  static vda5050_core::types::Action build_init_position(
    const std::string& action_id, double x, double y, double theta,
    const std::string& map_id, const std::string& last_node_id,
    const std::string& description = "");
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ACTIONS__ACTION_FACTORY_HPP_
