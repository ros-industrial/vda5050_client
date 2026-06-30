/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
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

#ifndef VDA5050_CORE__ORDER_UTILS__ORDER_GRAPH_VALIDATOR_HPP_
#define VDA5050_CORE__ORDER_UTILS__ORDER_GRAPH_VALIDATOR_HPP_

#include "vda5050_core/errors/validation_result.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core {

namespace order_utils {

/// \brief Checks that the nodes and edges in a VDA5050 Order form a valid
/// graph according to the VDA5050 specification sheet.
///
/// \param order The order to be checked.
///
/// \return A result struct of type errors::ValidationResult
errors::ValidationResult is_valid_graph(
  const vda5050_core::types::Order& order);

/// \brief Checks if order update is valid for order stitching
///
/// \param base_order The base order.
/// \param next_order the update order.
///
/// \return A result struct of type errors::ValidationResult
errors::ValidationResult is_valid_update(
  const vda5050_core::types::Order& base_order,
  const vda5050_core::types::Order& next_order);

}  // namespace order_utils
}  // namespace vda5050_core

#endif  // VDA5050_CORE__ORDER_UTILS__ORDER_GRAPH_VALIDATOR_HPP_
