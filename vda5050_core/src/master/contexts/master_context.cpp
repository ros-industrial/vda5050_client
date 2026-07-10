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

#include <algorithm>
#include <memory>
#include <mutex>
#include <typeindex>
#include <utility>
#include <vector>

#include "vda5050_core/master/updates/agv_updates.hpp"

namespace vda5050_core {
namespace master {

namespace {

bool same_error(const types::Error& a, const types::Error& b)
{
  // Identity is type + references; description is prose, not part of identity.
  return a.error_type == b.error_type &&
         a.error_references == b.error_references;
}

// Errors in `from` absent from `against`.
std::vector<types::Error> errors_diff(
  const std::vector<types::Error>& from,
  const std::vector<types::Error>& against)
{
  std::vector<types::Error> diff;
  for (const auto& e : from)
  {
    auto it = std::find_if(
      against.begin(), against.end(),
      [&e](const types::Error& a) { return same_error(e, a); });
    if (it == against.end()) diff.push_back(e);
  }
  return diff;
}

bool newly_reached_node(const types::State& prev, const types::State& curr)
{
  if (curr.last_node_id.empty()) return false;
  return curr.last_node_id != prev.last_node_id ||
         curr.last_node_sequence_id != prev.last_node_sequence_id;
}

std::vector<types::Error> errors_appeared(
  const types::State& prev, const types::State& curr)
{
  return errors_diff(curr.errors, prev.errors);
}

std::vector<types::Error> errors_resolved(
  const types::State& prev, const types::State& curr)
{
  return errors_diff(prev.errors, curr.errors);
}

bool new_base_requested(const types::State& prev, const types::State& curr)
{
  return curr.new_base_request.value_or(false) &&
         !prev.new_base_request.value_or(false);
}

bool mode_changed(const types::State& prev, const types::State& curr)
{
  return prev.operating_mode != curr.operating_mode;
}

bool paused_changed(const types::State& prev, const types::State& curr)
{
  return prev.paused.value_or(false) != curr.paused.value_or(false);
}

bool driving_changed(const types::State& prev, const types::State& curr)
{
  return prev.driving != curr.driving;
}

bool loads_changed(const types::State& prev, const types::State& curr)
{
  return prev.loads != curr.loads;
}

ConnectionTransition detect_connection_transition(
  const types::Connection* prev, const types::Connection& curr)
{
  if (prev && prev->connection_state == curr.connection_state)
  {
    return ConnectionTransition::NONE;
  }

  switch (curr.connection_state)
  {
    case types::ConnectionState::ONLINE:
      return ConnectionTransition::CONNECTED;
    case types::ConnectionState::OFFLINE:
      return ConnectionTransition::OFFLINE;
    case types::ConnectionState::CONNECTIONBROKEN:
      return ConnectionTransition::CONNECTIONBROKEN;
  }

  return ConnectionTransition::NONE;
}

}  // namespace

void MasterContext::init() {}

void MasterContext::on_state(
  const std::string& agv_id, const types::State& state)
{
  std::vector<std::shared_ptr<execution::UpdateBase>> updates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = prev_states_.find(agv_id);
    if (
      it != prev_states_.end() &&
      state.header.header_id < it->second.header.header_id)
    {
      return;  // out-of-order / stale State: drop
    }
    if (it != prev_states_.end())
    {
      const auto& prev = it->second;

      if (newly_reached_node(prev, state))
      {
        updates.push_back(std::make_shared<NodeReachedUpdate>(
          agv_id, state.last_node_id, state.last_node_sequence_id));
      }

      auto appeared = errors_appeared(prev, state);
      auto resolved = errors_resolved(prev, state);
      if (!appeared.empty() || !resolved.empty())
      {
        updates.push_back(std::make_shared<ErrorsChangedUpdate>(
          agv_id, std::move(appeared), std::move(resolved)));
      }

      if (new_base_requested(prev, state))
      {
        updates.push_back(std::make_shared<NewBaseRequestUpdate>(agv_id));
      }

      if (mode_changed(prev, state))
      {
        updates.push_back(std::make_shared<OperatingModeChangedUpdate>(
          agv_id, state.operating_mode, prev.operating_mode));
      }

      if (paused_changed(prev, state))
      {
        updates.push_back(std::make_shared<PausedChangedUpdate>(
          agv_id, state.paused.value_or(false)));
      }

      if (driving_changed(prev, state))
      {
        updates.push_back(
          std::make_shared<DrivingChangedUpdate>(agv_id, state.driving));
      }

      if (loads_changed(prev, state))
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
    const auto* prev = it != prev_connections_.end() ? &it->second : nullptr;
    auto kind = detect_connection_transition(prev, connection);
    if (kind != ConnectionTransition::NONE)
    {
      update = std::make_shared<ConnectionChangedUpdate>(agv_id, kind);
    }
    if (kind == ConnectionTransition::CONNECTED)
    {
      prev_states_.erase(agv_id);  // reconnect: drop stale baseline to reseed
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
