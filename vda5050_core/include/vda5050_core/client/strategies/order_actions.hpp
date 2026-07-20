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

#ifndef VDA5050_CORE__CLIENT__STRATEGIES__ORDER_ACTIONS_HPP_
#define VDA5050_CORE__CLIENT__STRATEGIES__ORDER_ACTIONS_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/client/events/edge_entered.hpp"
#include "vda5050_core/client/events/edge_left.hpp"
#include "vda5050_core/client/events/node_traversed.hpp"
#include "vda5050_core/client/resources/order_execution.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/engine.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core {

namespace client {

/// \brief Triggers and tracks an order's node and edge actions.
///
/// Subscribes to traversal events emitted by `OrderTraversal`
/// (`NodeTraversedEvent`, `EdgeEnteredEvent`, `EdgeLeftEvent`) and updates the
/// matching `actionState`s in the `OrderExecutionResource`.
///
/// The full `Action` objects are read from the accepted order. This strategy
/// decides when an action may start, marks it RUNNING, emits an
/// `ExecuteActionEvent`, and records status updates reported through
/// `ActionExecution`.
///
/// Blocking behavior is applied when actions are started:
/// - HARD runs exclusively and does not start while the AGV is driving;
/// - SOFT may run alongside other non-HARD actions but does not start while the
///   AGV is driving;
/// - NONE can run concurrently unless a HARD action is active.
///
/// This strategy only schedules order actions. It does not perform
/// robot-specific actions, handle instant actions, or directly control
/// navigation.
class OrderActions : public execution::StrategyInterface,
                     public std::enable_shared_from_this<OrderActions>
{
public:
  /// \brief Create an OrderActions owned by a shared_ptr.
  static std::shared_ptr<OrderActions> make(
    std::shared_ptr<execution::Engine> source);

  /// \brief Resolve the execution resource and subscribe to the source engine.
  void init(std::shared_ptr<execution::ContextInterface> context) override;

  /// \brief Retry actions deferred by blockingType once per strategy step.
  ///
  /// Actions are triggered by the source engine's callbacks; this only re-pumps
  /// the pending queue so a deferral that has since cleared (driving stopped, a
  /// blocker finished) proceeds without waiting for a new traversal event.
  void step(std::shared_ptr<execution::ContextInterface> context) override;

private:
  /// \brief Private; use make() to obtain a shared_ptr instance.
  explicit OrderActions(std::shared_ptr<execution::Engine> source);

  /// \brief Run a node's actions when it is traversed.
  void on_node_traversed(const NodeTraversedEvent& event);

  /// \brief Start an edge's actions when it is entered.
  void on_edge_entered(const EdgeEnteredEvent& event);

  /// \brief Stop an edge's still-running actions when it is left.
  void on_edge_left(const EdgeLeftEvent& event);

  /// \brief Queue freshly triggered actions (de-duplicated by action_id).
  void enqueue(const std::vector<types::Action>& actions);

  /// \brief Start every pending action whose blockingType constraints are met,
  /// repeating until no further action can start (a completion may unblock more).
  void pump();

  /// \brief Claim an action and emit an ExecuteActionEvent.
  void start_action(const types::Order& order, const types::Action& action);

  /// \brief Record an action status update reported through ActionExecution.
  void update_action_status(
    const std::string& action_id, types::ActionStatus status,
    std::optional<std::string> result_description);

  /// \brief Finish any of these actions that are still running (edge left).
  void stop_actions(const std::vector<types::Action>& actions);

  std::shared_ptr<execution::Engine> source_;
  std::shared_ptr<OrderExecutionResource> execution_;

  /// \brief Triggered actions still waiting for their blockingType turn.
  std::vector<types::Action> pending_;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__STRATEGIES__ORDER_ACTIONS_HPP_
