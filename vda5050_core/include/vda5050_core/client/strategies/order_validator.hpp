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

#ifndef VDA5050_CORE__CLIENT__STRATEGIES__ORDER_VALIDATOR_HPP_
#define VDA5050_CORE__CLIENT__STRATEGIES__ORDER_VALIDATOR_HPP_

#include <memory>
#include <vector>

#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core {

namespace client {

/// \brief Outcome of the VDA5050 order-acceptance decision.
enum class AcceptanceOutcome
{
  /// \brief Order is valid and should be applied to the execution state.
  ACCEPTED,
  /// \brief Order is invalid; report `errors` and keep the previous order.
  REJECTED,
  /// \brief Duplicate resend (same orderId + orderUpdateId); leave state unchanged.
  IGNORED
};

/// \brief Result of order-acceptance validation.
struct AcceptanceResult
{
  AcceptanceOutcome outcome = AcceptanceOutcome::REJECTED;

  /// \brief Errors to report when `outcome == REJECTED`.
  std::vector<types::Error> errors;

  bool accepted() const
  {
    return outcome == AcceptanceOutcome::ACCEPTED;
  }
  bool rejected() const
  {
    return outcome == AcceptanceOutcome::REJECTED;
  }
  bool ignored() const
  {
    return outcome == AcceptanceOutcome::IGNORED;
  }
};

/// \brief Gatekeeper that runs the VDA5050 order-acceptance decision flow
/// before an order enters the execution plane.
///
/// Structural validity reuses `validation::is_valid_graph`.
/// TODO(eileentyz): Add position- and capability-dependent checks once AGV
/// position and factsheet/capability data are available.
class OrderValidator
{
public:
  /// \brief Evaluate an incoming order against graph validity and execution state.
  ///
  /// \param incoming_order Order payload received from master control.
  /// \param context Execution context exposing the order-execution resource.
  /// \return Acceptance decision; on rejection `errors` holds the VDA5050 errors.
  // TODO(eileentyz): Add protocol limit validation once factsheet/config data
  // is available.
  AcceptanceResult validate_order(
    const types::Order& incoming_order,
    const std::shared_ptr<execution::ContextInterface>& context) const;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__STRATEGIES__ORDER_VALIDATOR_HPP_
