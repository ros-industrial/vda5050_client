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

#ifndef VDA5050_CORE__CLIENT__STRATEGIES__STATE_REPORTING_HPP_
#define VDA5050_CORE__CLIENT__STRATEGIES__STATE_REPORTING_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <optional>

#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {

namespace client {

/// \brief Hook invoked with the current VDA5050 State whenever it changes.
///
/// This is where a transport layer (e.g. MQTT) plugs in: the core assembles and
/// hands out the State, it does not publish.
using StateReporter = std::function<void(const types::State&)>;

/// \brief Reports the AGV's VDA5050 State and finalizes order completion.
///
/// The State is already assembled in `OrderExecutionResource` by the acceptance,
/// traversal, and action strategies. On each step this strategy:
/// - clears `executing_order` once the order is complete (route fully traversed
///   and every action terminal, i.e. FINISHED/FAILED);
/// - reports the current State through a `StateReporter` hook when it changes
///   or when the heartbeat interval expires.
///
/// Out of scope: instantAction state handling and real transport. Fields owned
/// by components that may not exist yet (driving, paused, AGV position, velocity,
/// loads, battery, safety) are reported straight from State at their current
/// values.
class StateReporting : public execution::StrategyInterface
{
public:
  /// \brief Construct with the maximum interval between State reports.
  explicit StateReporting(
    std::chrono::milliseconds heartbeat_interval = std::chrono::seconds(30));

  /// \brief Resolve the execution resource that carries the State.
  void init(std::shared_ptr<execution::ContextInterface> context) override;

  /// \brief Finalize order completion, then report the State if it changed.
  void step(std::shared_ptr<execution::ContextInterface> context) override;

  /// \brief Register the hook that receives State updates (no-op if empty).
  void set_reporter(StateReporter reporter);

private:
  std::shared_ptr<OrderExecutionResource> execution_;
  StateReporter reporter_;

  /// \brief Last State handed to the reporter; used to report only on change.
  std::optional<types::State> last_reported_;

  std::chrono::milliseconds heartbeat_interval_;
  std::optional<std::chrono::steady_clock::time_point> last_report_time_;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__STRATEGIES__STATE_REPORTING_HPP_
