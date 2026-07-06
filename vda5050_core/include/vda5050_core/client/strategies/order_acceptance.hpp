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

#ifndef VDA5050_CORE__CLIENT__STRATEGIES__ORDER_ACCEPTANCE_HPP_
#define VDA5050_CORE__CLIENT__STRATEGIES__ORDER_ACCEPTANCE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "vda5050_core/client/strategies/order_validator.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"

namespace vda5050_core {

namespace client {

/// \brief Strategy that runs the VDA5050 order-acceptance flow.
///
/// On each step it reads the latest inbound `OrderUpdate` from the context,
/// runs the `OrderValidator` decision, and applies the corresponding side effects:
///   - Accepted -> applies the order to the execution state.
///   - Rejected -> records the validation errors.
///   - Ignored  -> leaves the current execution state unchanged.
///
/// The validator owns the decision; this strategy owns the side effects,
/// so the decision logic stays independently testable.
class OrderAcceptance : public execution::StrategyInterface
{
public:
  OrderAcceptance();

  /// \brief Construct with a pre-configured validator
  explicit OrderAcceptance(OrderValidator validator);

  void init(std::shared_ptr<execution::ContextInterface> context) override;

  void step(std::shared_ptr<execution::ContextInterface> context) override;

private:
  OrderValidator validator_;

  std::string last_order_id_;
  uint32_t last_order_update_id_ = 0;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__STRATEGIES__ORDER_ACCEPTANCE_HPP_
