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

#include "vda5050_core/master/event_detector.hpp"

#include <utility>

namespace vda5050_core {
namespace master {

//=============================================================================
EventDetector::EventDetector(
  std::string agv_id, std::shared_ptr<execution::Provider> provider)
: agv_id_(std::move(agv_id)), provider_(std::move(provider))
{
}

//=============================================================================
void EventDetector::on_state(const types::State& state)
{
  std::optional<event::ReachedNode> reached;
  std::vector<types::Error> appeared;
  std::vector<types::Error> resolved;
  bool mode_changed = false;
  bool paused_changed = false;
  bool driving_changed = false;
  bool new_base_requested = false;
  bool loads_changed = false;

  // Diff against the last seen state and take the new snapshot under the lock;
  // publish outside it so a listener that re-enters the provider can't deadlock.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (have_prev_state_)
    {
      reached = event::newly_reached_node(prev_state_, state);
      appeared = event::errors_appeared(prev_state_, state);
      resolved = event::errors_resolved(prev_state_, state);
      mode_changed = event::mode_changed(prev_state_, state);
      paused_changed = event::paused_changed(prev_state_, state);
      driving_changed = event::driving_changed(prev_state_, state);
      new_base_requested = event::new_base_requested(prev_state_, state);
      loads_changed = event::loads_changed(prev_state_, state);
    }
    prev_state_ = state;
    have_prev_state_ = true;
  }

  if (reached)
  {
    provider_->push<NodeReachedUpdate>(agv_id_, *reached);
  }
  if (!appeared.empty() || !resolved.empty())
  {
    provider_->push<ErrorsChangedUpdate>(
      agv_id_, std::move(appeared), std::move(resolved));
  }
  if (mode_changed)
  {
    provider_->push<OperatingModeChangedUpdate>(agv_id_, state.operating_mode);
  }
  if (paused_changed)
  {
    provider_->push<PausedChangedUpdate>(agv_id_, state.paused.value_or(false));
  }
  if (driving_changed)
  {
    provider_->push<DrivingChangedUpdate>(agv_id_, state.driving);
  }
  if (new_base_requested)
  {
    provider_->push<NewBaseRequestUpdate>(agv_id_);
  }
  if (loads_changed)
  {
    provider_->push<LoadsChangedUpdate>(
      agv_id_, state.loads.value_or(std::vector<types::Load>{}));
  }
}

//=============================================================================
void EventDetector::on_connection(const types::Connection& connection)
{
  ConnectionEventKind kind;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    kind = detect_connection_transition(prev_connection_, connection);
    prev_connection_ = connection;
  }

  if (kind != ConnectionEventKind::NONE)
  {
    provider_->push<ConnectionChangedUpdate>(agv_id_, kind);
  }
}

}  // namespace master
}  // namespace vda5050_core
