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

namespace vda5050_core::master {

// =============================================================================
// ActionFactory (Tasks #20, #38).
// =============================================================================
//
// Helper to construct vda5050_core::types::Action structs ergonomically.
//
// - `build_custom`: generic builder for arbitrary action_types
// - `generate_action_id`: UUIDv4 helper for callers without their own ID
// - `build_state_request`: the predefined stateRequest action
// - `build_factsheet_request`: the predefined factsheetRequest action
//
// Header building (header_id sequence, timestamp, version, manufacturer,
// serial_number) is owned by ProtocolAdapter; the factory MUST NOT touch it.
// Callers wrap the produced Action(s) in a vda5050_core::types::InstantActions
// with default-initialized header — the adapter overwrites it on publish.
//
// **stateRequest / factsheetRequest workflow** (VM-VDA-6-8-2-6 [BACKLOG]
// "stateRequest failed workflow", VM-VDA-6-8-2-8 [BACKLOG]
// "factsheetRequest finished workflow"):
//
// Per the spec, neither action
// takes parameters. The factory builders below produce a default-blocking
// (NONE) action with the canonical action_type spelling and the
// caller-supplied action_id. The action_id is what FMS uses to correlate
// the response.
//
// **FMS observation pattern** (master gives the hooks; FMS owns tracking):
//
//     // 1. FMS sends the request via assign_instant_actions:
//     auto req = ActionFactory::build_state_request("req-42");
//     master.assign_instant_actions(mfg, serial, wrap({req}));
//
//     // 2. FMS subclass overrides on_state(...) to inspect actionStates:
//     void MyMaster::on_state(const std::string& agv_id,
//                             const vda5050_core::types::State& state) {
//       for (const auto& as : state.action_states) {
//         if (as.action_id != "req-42") continue;
//         if (as.action_status == ActionStatus::FINISHED) { ... }
//         if (as.action_status == ActionStatus::FAILED)   { ... }
//       }
//     }
//
//     // 3. For factsheetRequest specifically, the AGV publishes on the
//     //    retained `factsheet` topic; on_factsheet(...) fires too.
//     //    Either callback can be the trigger.
//
// **Spec ambiguity on stateRequest "failed" workflow**
// shows only FINISHED for stateRequest ("the state has been communicated").
// FAILED is technically a legal action_status enum value but spec doesn't
// describe when AGV would emit it for stateRequest. FMS should handle
// FAILED (log + retry) and treat absent-action_id-in-state
// as "still pending".

class ActionFactory
{
public:
  /// \brief Build an instantAction with the given fields.
  ///
  /// Caller owns action_id — pass an FMS-correlated identifier (e.g. a
  /// job-tracker UUID). For the rare case where the caller has no
  /// preferred ID, see generate_action_id().
  ///
  /// blocking_type defaults to NONE per the spec (least
  /// restrictive). The spec doesn't mandate a default; NONE is safe for
  /// stateRequest / factsheetRequest / logReport (all inherently NONE).
  ///
  /// The returned Action has action_parameters populated only when
  /// `parameters` is non-empty; otherwise the optional is left
  /// std::nullopt to keep wire payloads compact.
  static vda5050_core::types::Action build_custom(
    const std::string& action_type, const std::string& action_id,
    vda5050_core::types::BlockingType blocking_type =
      vda5050_core::types::BlockingType::NONE,
    const std::string& description = "",
    const std::vector<vda5050_core::types::ActionParameter>& parameters = {});

  /// \brief Generate a UUIDv4 hex string suitable for action_id.
  ///
  /// Use ONLY when the caller has no FMS-side identifier to correlate.
  /// Most FMS deployments should pass their own job-tracker IDs to
  /// build_custom() instead.
  ///
  /// Format: 8-4-4-4-12 lowercase hex, total 36 chars including dashes,
  /// matching RFC 4122 §3 textual UUID layout. Variant + version bits set
  /// per RFC 4122 §4.4 (random UUID).
  static std::string generate_action_id();

  /// \brief Build a stateRequest instantAction.
  ///
  /// Per the spec, stateRequest takes no parameters and is
  /// inherently NONE-blocking (a query). After the AGV processes it, a
  /// fresh State message is published; FMS observes via on_state.
  ///
  /// \param action_id     Caller-supplied unique id (UUID recommended).
  /// \param description   Optional human-readable annotation.
  static vda5050_core::types::Action build_state_request(
    const std::string& action_id, const std::string& description = "");

  /// \brief Build a factsheetRequest instantAction.
  ///
  /// Per the spec, factsheetRequest takes no parameters and is
  /// inherently NONE-blocking. After the AGV processes it, a Factsheet
  /// message is published on the retained `factsheet` topic; FMS observes
  /// via on_factsheet (and FINISHED action_status arriving in the next
  /// state on action_states[]).
  ///
  /// \param action_id     Caller-supplied unique id (UUID recommended).
  /// \param description   Optional human-readable annotation.
  static vda5050_core::types::Action build_factsheet_request(
    const std::string& action_id, const std::string& description = "");
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__ACTIONS__ACTION_FACTORY_HPP_
