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

#include "vda5050_core/client/strategies/state_reporting.hpp"

#include <memory>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/types/action_status.hpp"

namespace vda5050_core {

namespace client {

namespace {

// The route is fully traversed when no nodes or edges remain to visit.
bool route_done(const types::State& state)
{
  return state.node_states.empty() && state.edge_states.empty();
}

// Every action has reached a terminal status (FINISHED or FAILED).
bool all_actions_terminal(const types::State& state)
{
  for (const auto& action_state : state.action_states)
  {
    if (
      action_state.action_status != types::ActionStatus::FINISHED &&
      action_state.action_status != types::ActionStatus::FAILED)
    {
      return false;
    }
  }
  return true;
}

// An order is complete once its route is done and all its actions are terminal.
bool order_complete(const types::State& state)
{
  return route_done(state) && all_actions_terminal(state);
}

}  // namespace

StateReporting::StateReporting(std::chrono::milliseconds heartbeat_interval)
: heartbeat_interval_(heartbeat_interval)
{
}

void StateReporting::init(std::shared_ptr<execution::ContextInterface> context)
{
  if (!context)
  {
    VDA5050_WARN_STREAM("StateReporting: context is null");
    return;
  }

  // The execution resource carries the State this strategy reports.
  execution_ = context->get_resource<OrderExecutionResource>();
  if (!execution_)
  {
    VDA5050_WARN_STREAM("StateReporting: OrderExecutionResource not available");
  }
}

void StateReporting::set_reporter(StateReporter reporter)
{
  if (!reporter) return;
  reporter_ = std::move(reporter);
}

void StateReporting::step(std::shared_ptr<execution::ContextInterface> /*ctx*/)
{
  if (!execution_) return;

  types::State state = execution_->get_state();

  // Finalize completion: once a running order is fully traversed with all its
  // actions terminal, it is no longer executing. Only an order that is currently
  // executing is cleared, so this is idempotent on subsequent steps. executing
  // is internal bookkeeping (not a State field), so clearing it does not by
  // itself trigger a report.
  if (execution_->is_executing_order() && order_complete(state))
  {
    execution_->set_executing_order(false);
  }

  // Report on meaningful state changes and periodically as a heartbeat.
  if (!reporter_) return;
  const auto now = std::chrono::steady_clock::now();
  const bool changed = !last_reported_ || *last_reported_ != state;
  const bool heartbeat_due =
    last_report_time_ && now - *last_report_time_ >= heartbeat_interval_;
  if (changed || heartbeat_due)
  {
    reporter_(state);
    last_reported_ = std::move(state);
    last_report_time_ = now;
  }
}

}  // namespace client
}  // namespace vda5050_core
