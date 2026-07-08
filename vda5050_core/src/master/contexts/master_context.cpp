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

#include "vda5050_core/master/contexts/master_context.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <typeindex>
#include <utility>
#include <vector>

namespace vda5050_core {
namespace master {

void MasterContext::init() {}

void MasterContext::on_state(
  const std::string& agv_id, const types::State& state)
{
  std::vector<std::shared_ptr<execution::UpdateBase>> updates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = prev_states_.find(agv_id);
    if (it != prev_states_.end())
    {
      const auto& prev = it->second;

      if (auto reached = update::newly_reached_node(prev, state))
      {
        updates.push_back(
          std::make_shared<NodeReachedUpdate>(agv_id, *reached));
      }

      auto appeared = update::errors_appeared(prev, state);
      auto resolved = update::errors_resolved(prev, state);
      if (!appeared.empty() || !resolved.empty())
      {
        updates.push_back(std::make_shared<ErrorsChangedUpdate>(
          agv_id, std::move(appeared), std::move(resolved)));
      }

      if (update::new_base_requested(prev, state))
      {
        updates.push_back(std::make_shared<NewBaseRequestUpdate>(agv_id));
      }

      if (update::mode_changed(prev, state))
      {
        updates.push_back(std::make_shared<OperatingModeChangedUpdate>(
          agv_id, state.operating_mode, prev.operating_mode));
      }

      if (update::paused_changed(prev, state))
      {
        updates.push_back(std::make_shared<PausedChangedUpdate>(
          agv_id, state.paused.value_or(false)));
      }

      if (update::driving_changed(prev, state))
      {
        updates.push_back(
          std::make_shared<DrivingChangedUpdate>(agv_id, state.driving));
      }

      if (update::loads_changed(prev, state))
      {
        updates.push_back(std::make_shared<LoadsChangedUpdate>(
          agv_id, state.loads.value_or(std::vector<types::Load>{})));
      }
    }
    prev_states_[agv_id] = state;
  }

  for (auto& u : updates) provider()->push_shared(std::move(u));
}

void MasterContext::on_connection(
  const std::string& agv_id, const types::Connection& connection)
{
  std::shared_ptr<execution::UpdateBase> update;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = prev_connections_.find(agv_id);
    const std::optional<types::Connection> prev =
      it != prev_connections_.end()
        ? std::optional<types::Connection>(it->second)
        : std::nullopt;
    auto kind = detect_connection_transition(prev, connection);
    if (kind != ConnectionTransition::NONE)
    {
      update = std::make_shared<ConnectionChangedUpdate>(agv_id, kind);
    }
    prev_connections_[agv_id] = connection;
  }

  if (update) provider()->push_shared(std::move(update));
}

std::shared_ptr<execution::UpdateBase> MasterContext::get_update_raw(
  std::type_index /*type*/) const
{
  return nullptr;
}

std::shared_ptr<execution::ResourceBase> MasterContext::get_resource_raw(
  std::type_index /*type*/) const
{
  return nullptr;
}

}  // namespace master
}  // namespace vda5050_core
