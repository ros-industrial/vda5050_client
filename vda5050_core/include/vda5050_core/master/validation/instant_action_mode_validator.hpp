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

#ifndef VDA5050_CORE__MASTER__VALIDATION__INSTANT_ACTION_MODE_VALIDATOR_HPP_
#define VDA5050_CORE__MASTER__VALIDATION__INSTANT_ACTION_MODE_VALIDATOR_HPP_

#include <string>

#include "vda5050_core/master/validation/pre_send_validator.hpp"
#include "vda5050_core/order_utils/validation_result.hpp"
#include "vda5050_core/types/instant_actions.hpp"

namespace vda5050_core::master {

/// \brief Operating-mode gate: in non-AUTOMATIC modes, only the predefined
/// instant-scope actions (is_mode_exempt_action_type) may be sent.
vda5050_core::order_utils::ValidationResult validate_instant_action_mode(
  const PreSendContext& ctx,
  const vda5050_core::types::InstantActions& actions);

/// \brief True if action_type is a predefined instant-scope action exempt from
/// the operating-mode gate (e.g. stateRequest, cancelOrder, initPosition).
bool is_mode_exempt_action_type(const std::string& action_type);

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__VALIDATION__INSTANT_ACTION_MODE_VALIDATOR_HPP_
