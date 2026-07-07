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

/// \brief Builds Action structs (arbitrary custom actions plus the predefined
///        stateRequest / factsheetRequest).
///
/// Never sets the header — ProtocolAdapter owns it and overwrites it on
/// publish. The predefined builders take no parameters and default to NONE
/// blocking; the caller-supplied action_id correlates the AGV's response in
/// on_state's action_states.
class ActionFactory
{
public:
  /// \brief Build an instantAction. Caller owns action_id (see
  ///        generate_action_id() if none). blocking_type defaults to NONE;
  ///        action_parameters is set only when `parameters` is non-empty.
  static vda5050_core::types::Action build_custom(
    const std::string& action_type, const std::string& action_id,
    vda5050_core::types::BlockingType blocking_type =
      vda5050_core::types::BlockingType::NONE,
    const std::string& description = "",
    const std::vector<vda5050_core::types::ActionParameter>& parameters = {});

  /// \brief Generate a UUIDv4 hex action_id (RFC 4122, 8-4-4-4-12 lowercase).
  ///        Use only when the caller has no FMS-side id to correlate.
  static std::string generate_action_id();

  /// \brief Build a stateRequest instantAction (no params, NONE-blocking); the
  ///        AGV replies with a fresh State, observed via on_state.
  /// \param action_id     Caller-supplied unique id (UUID recommended).
  /// \param description   Optional human-readable annotation.
  static vda5050_core::types::Action build_state_request(
    const std::string& action_id, const std::string& description = "");

  /// \brief Build a factsheetRequest instantAction (no params, NONE-blocking);
  ///        the AGV replies on the retained factsheet topic (on_factsheet).
  /// \param action_id     Caller-supplied unique id (UUID recommended).
  /// \param description   Optional human-readable annotation.
  static vda5050_core::types::Action build_factsheet_request(
    const std::string& action_id, const std::string& description = "");
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ACTIONS__ACTION_FACTORY_HPP_
