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

#ifndef VDA5050_CORE__MASTER__VALIDATION__SCHEMA_VALIDATOR_HPP_
#define VDA5050_CORE__MASTER__VALIDATION__SCHEMA_VALIDATOR_HPP_

#include "vda5050_core/order_utils/validation_result.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

namespace vda5050_core::master {

/// \brief Structural field checks on an Order (version, non-empty ids).
vda5050_core::order_utils::ValidationResult validate_order_schema(
  const vda5050_core::types::Order& order);

/// \brief Structural field checks on an InstantActions message.
vda5050_core::order_utils::ValidationResult validate_instant_actions_schema(
  const vda5050_core::types::InstantActions& actions);

/// \brief Structural field checks on a State message.
vda5050_core::order_utils::ValidationResult validate_state_schema(
  const vda5050_core::types::State& state);

/// \brief Structural field checks on a Connection message.
vda5050_core::order_utils::ValidationResult validate_connection_schema(
  const vda5050_core::types::Connection& connection);

/// \brief Structural field checks on a Factsheet message.
vda5050_core::order_utils::ValidationResult validate_factsheet_schema(
  const vda5050_core::types::Factsheet& factsheet);

/// \brief Structural field checks on a Visualization message.
vda5050_core::order_utils::ValidationResult validate_visualization_schema(
  const vda5050_core::types::Visualization& visualization);

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__VALIDATION__SCHEMA_VALIDATOR_HPP_
