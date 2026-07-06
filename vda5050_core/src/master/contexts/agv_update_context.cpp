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

#include "vda5050_core/master/contexts/agv_update_context.hpp"

#include <memory>
#include <mutex>
#include <typeindex>
#include <utility>
#include <vector>

namespace vda5050_core {
namespace master {

AGVUpdateContext::AGVUpdateContext(std::string agv_id)
: agv_id_(std::move(agv_id))
{
}

void AGVUpdateContext::init() {}

void AGVUpdateContext::on_state(const types::State& state)
{
  if (prev_state_)
  {
    const auto& prev = *prev_state_;

    if (auto reached = update::newly_reached_node(prev, state))
    {
      publish<NodeReachedUpdate>(agv_id_, *reached);
    }

    auto appeared = update::errors_appeared(prev, state);
    auto resolved = update::errors_resolved(prev, state);
    if (!appeared.empty() || !resolved.empty())
    {
      publish<ErrorsChangedUpdate>(
        agv_id_, std::move(appeared), std::move(resolved));
    }

    if (update::new_base_requested(prev, state))
    {
      publish<NewBaseRequestUpdate>(agv_id_);
    }

    if (update::mode_changed(prev, state))
    {
      publish<OperatingModeChangedUpdate>(
        agv_id_, state.operating_mode, prev.operating_mode);
    }

    if (update::paused_changed(prev, state))
    {
      publish<PausedChangedUpdate>(agv_id_, state.paused.value_or(false));
    }

    if (update::driving_changed(prev, state))
    {
      publish<DrivingChangedUpdate>(agv_id_, state.driving);
    }

    if (update::loads_changed(prev, state))
    {
      publish<LoadsChangedUpdate>(
        agv_id_, state.loads.value_or(std::vector<types::Load>{}));
    }
  }

  prev_state_ = state;
}

void AGVUpdateContext::on_connection(const types::Connection& connection)
{
  auto kind = detect_connection_transition(prev_connection_, connection);
  if (kind != ConnectionTransition::NONE)
  {
    publish<ConnectionChangedUpdate>(agv_id_, kind);
  }
  prev_connection_ = connection;
}

std::shared_ptr<execution::UpdateBase> AGVUpdateContext::get_update_raw(
  std::type_index type) const
{
  std::lock_guard<std::mutex> lock(storage_mutex_);
  auto it = updates_.find(type);
  return it != updates_.end() ? it->second : nullptr;
}

std::shared_ptr<execution::ResourceBase> AGVUpdateContext::get_resource_raw(
  std::type_index /*type*/) const
{
  return nullptr;
}

void AGVUpdateContext::cache_update(
  const std::shared_ptr<execution::UpdateBase>& update)
{
  std::lock_guard<std::mutex> lock(storage_mutex_);
  updates_[update->get_type()] = update;
}

}  // namespace master
}  // namespace vda5050_core
