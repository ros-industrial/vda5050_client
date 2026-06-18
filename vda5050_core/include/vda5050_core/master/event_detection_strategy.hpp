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

#ifndef VDA5050_CORE__MASTER__EVENT_DETECTION_STRATEGY_HPP_
#define VDA5050_CORE__MASTER__EVENT_DETECTION_STRATEGY_HPP_

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/engine.hpp"
#include "vda5050_core/execution/event_queue.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"
#include "vda5050_core/master/connection/connection_event_detector.hpp"
#include "vda5050_core/master/state/state_event_detector.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {
namespace master {

// Inbound updates: the raw AGV messages the master receives.
struct StateUpdate : execution::Initialize<StateUpdate, execution::UpdateBase>
{
  types::State state;
  explicit StateUpdate(types::State s) : state(std::move(s)) {}
};

struct ConnectionUpdate
: execution::Initialize<ConnectionUpdate, execution::UpdateBase>
{
  types::Connection connection;
  explicit ConnectionUpdate(types::Connection c) : connection(std::move(c)) {}
};

// Outbound events: discrete transitions emitted to internal fleet logic.
struct NodeReachedEvent
: execution::Initialize<NodeReachedEvent, execution::EventBase>
{
  event::ReachedNode node;
  explicit NodeReachedEvent(event::ReachedNode n) : node(std::move(n)) {}
};

struct ErrorsChangedEvent
: execution::Initialize<ErrorsChangedEvent, execution::EventBase>
{
  std::vector<types::Error> appeared;
  std::vector<types::Error> resolved;
  ErrorsChangedEvent(std::vector<types::Error> a, std::vector<types::Error> r)
  : appeared(std::move(a)), resolved(std::move(r))
  {
  }
};

struct StateFlagChangedEvent
: execution::Initialize<StateFlagChangedEvent, execution::EventBase>
{
  enum class Flag
  {
    NEW_BASE_REQUESTED,
    MODE,
    PAUSED,
    DRIVING,
    LOADS,
  };
  Flag flag;
  explicit StateFlagChangedEvent(Flag f) : flag(f) {}
};

struct ConnectionChangedEvent
: execution::Initialize<ConnectionChangedEvent, execution::EventBase>
{
  ConnectionEventKind kind;
  explicit ConnectionChangedEvent(ConnectionEventKind k) : kind(k) {}
};

/// \brief Folds the per-transition detectors into the execution engine.
///
/// On each inbound State / Connection update it diffs against the previous
/// snapshot using the stateless detector functions and emits a typed event per
/// transition. Downstream fleet logic consumes them via engine()->on<...>().
class EventDetectionStrategy : public execution::StrategyInterface
{
public:
  void init(std::shared_ptr<execution::ContextInterface> context) override
  {
    context->provider()->on<StateUpdate>(
      [this](std::shared_ptr<StateUpdate> update) {
        engine()->notify(update);
      });
    context->provider()->on<ConnectionUpdate>(
      [this](std::shared_ptr<ConnectionUpdate> update) {
        engine()->notify(update);
      });
  }

  void step(std::shared_ptr<execution::ContextInterface> context) override
  {
    if (auto update = context->get_update<StateUpdate>())
    {
      detect_state_events(update->state);
    }
    if (auto update = context->get_update<ConnectionUpdate>())
    {
      detect_connection_event(update->connection);
    }
  }

private:
  void detect_state_events(const types::State& curr)
  {
    if (have_prev_state_)
    {
      if (auto reached = event::newly_reached_node(prev_state_, curr))
      {
        emit_now<NodeReachedEvent>(*reached);
      }

      auto appeared = event::errors_appeared(prev_state_, curr);
      auto resolved = event::errors_resolved(prev_state_, curr);
      if (!appeared.empty() || !resolved.empty())
      {
        emit_now<ErrorsChangedEvent>(std::move(appeared), std::move(resolved));
      }

      emit_flag(
        event::new_base_requested(prev_state_, curr),
        StateFlagChangedEvent::Flag::NEW_BASE_REQUESTED);
      emit_flag(
        event::mode_changed(prev_state_, curr),
        StateFlagChangedEvent::Flag::MODE);
      emit_flag(
        event::paused_changed(prev_state_, curr),
        StateFlagChangedEvent::Flag::PAUSED);
      emit_flag(
        event::driving_changed(prev_state_, curr),
        StateFlagChangedEvent::Flag::DRIVING);
      emit_flag(
        event::loads_changed(prev_state_, curr),
        StateFlagChangedEvent::Flag::LOADS);
    }
    prev_state_ = curr;
    have_prev_state_ = true;
  }

  void detect_connection_event(const types::Connection& curr)
  {
    const ConnectionEventKind kind =
      detect_connection_transition(prev_connection_, curr);
    if (kind != ConnectionEventKind::NONE)
    {
      emit_now<ConnectionChangedEvent>(kind);
    }
    prev_connection_ = curr;
  }

  void emit_flag(bool changed, StateFlagChangedEvent::Flag flag)
  {
    if (changed) emit_now<StateFlagChangedEvent>(flag);
  }

  // emit() only enqueues, and the engine dispatches one event per step(); emit
  // and step together so each transition reaches its handler in this tick.
  template <typename EventT, typename... Args>
  void emit_now(Args&&... args)
  {
    engine()->emit<EventT>(
      execution::Priority::NORMAL, std::forward<Args>(args)...);
    engine()->step();
  }

  types::State prev_state_{};
  bool have_prev_state_ = false;
  std::optional<types::Connection> prev_connection_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__EVENT_DETECTION_STRATEGY_HPP_
