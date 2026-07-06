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

#include <algorithm>

#include "vda5050_core/client/adapter/state_manager.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
std::shared_ptr<StateManager> StateManager::make()
{
  auto manager = std::shared_ptr<StateManager>(new StateManager());
  return manager;
}

//=============================================================================
void StateManager::set_position(
  double x, double y, double theta, const std::string& map_id)
{
  types::AGVPosition agv_position;
  agv_position.x = x;
  agv_position.y = y;
  agv_position.theta = theta;
  agv_position.map_id = map_id;
  agv_position.position_initialized = position_initialized_;

  set_agv_position(agv_position);
}

//=============================================================================
void StateManager::set_velocity(const types::Velocity& velocity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.velocity = velocity;
}

//=============================================================================
void StateManager::set_driving(bool driving)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.driving = driving;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_paused(bool paused)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.paused = paused;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_new_base_request(bool request)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.new_base_request = request;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_distance_since_last_node(double distance)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.distance_since_last_node = distance;
}

//=============================================================================
void StateManager::set_battery_state(const types::BatteryState& battery)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.battery_state = battery;
}

//=============================================================================
void StateManager::set_operating_mode(types::OperatingMode mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.operating_mode = mode;
}

//=============================================================================
void StateManager::set_safety_state(const types::SafetyState& safety_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.safety_state = safety_state;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::add_action_state(const types::ActionState& action_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = std::find_if(
    state_.action_states.begin(), state_.action_states.end(),
    [&action_state](auto action) {
      return action_state.action_id == action.action_id;
    });
  if (it != state_.action_states.end())
  {
    *it = action_state;
  }
  else
  {
    state_.action_states.push_back(action_state);
  }
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_action_states(
  const std::vector<types::ActionState>& action_states)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.action_states = action_states;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::clear_action_states()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.action_states.clear();
}

//=============================================================================
void StateManager::add_error(const types::Error& error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.errors.push_back(error);
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_errors(const std::vector<types::Error>& errors)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.errors = errors;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::clear_errors()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.errors.clear();
}

//=============================================================================
void StateManager::add_load(const types::Load& load)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!state_.loads.has_value())
  {
    state_.loads = {};
  }
  state_.loads->push_back(load);
}

//=============================================================================
void StateManager::set_loads(const std::vector<types::Load>& loads)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.loads = loads;
}

//=============================================================================
void StateManager::clear_loads()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.loads = {};
}

//=============================================================================
void StateManager::remove_loads()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.loads = std::nullopt;
}

//=============================================================================
void StateManager::add_information(const types::Info& information)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.information.has_value())
  {
    state_.information = {};
  }
  state_.information->push_back(information);
}

//=============================================================================
void StateManager::set_information(const std::vector<types::Info>& infomation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.information = infomation;
}

//=============================================================================
void StateManager::remove_information()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.information = std::nullopt;
}

//=============================================================================
types::State StateManager::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

//=============================================================================
void StateManager::mark_publish_requested()
{
  publish_requested_ = true;
}

//=============================================================================
bool StateManager::consume_publish_requested()
{
  return publish_requested_.exchange(false);
}

//=============================================================================
StateManager::StateManager()
: position_initialized_(false), publish_requested_(false)
{
  // Nothing to do here ...
}

//=============================================================================
void StateManager::set_order(const types::Order& order)
{
  std::lock_guard<std::mutex> lock(mutex_);

  state_.order_id = order.order_id;
  state_.order_update_id = order.order_update_id;
  state_.zone_set_id = order.zone_set_id;

  state_.node_states.clear();
  state_.edge_states.clear();

  for (const auto& node : order.nodes)
  {
    types::NodeState ns;
    ns.node_id = node.node_id;
    ns.sequence_id = node.sequence_id;
    ns.released = node.released;
    ns.node_description = node.node_description;
    ns.node_position = node.node_position;

    state_.node_states.push_back(std::move(ns));
  }

  for (const auto& edge : order.edges)
  {
    types::EdgeState es;
    es.edge_id = edge.edge_id;
    es.sequence_id = es.sequence_id;
    es.released = edge.released;
    es.edge_description = edge.edge_description;
    es.trajectory = edge.trajectory;

    state_.edge_states.push_back(std::move(es));
  }
}

//=============================================================================
void StateManager::node_reached(const types::Node& node)
{
  std::lock_guard<std::mutex> lock(mutex_);

  state_.last_node_id = node.node_id;
  state_.last_node_sequence_id = node.sequence_id;

  auto it = std::find_if(
    state_.node_states.begin(), state_.node_states.end(),
    [&](const auto& node_state) {
      return node_state.node_id == node.node_id &&
             node_state.sequence_id == node.sequence_id;
    });

  if (it != state_.node_states.end())
  {
    state_.node_states.erase(it);
  }
}

//=============================================================================
void StateManager::edge_traversed(const types::Edge& edge)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = std::find_if(
    state_.edge_states.begin(), state_.edge_states.end(),
    [&](const auto& edge_state) {
      return edge_state.edge_id == edge.edge_id &&
             edge_state.sequence_id == edge.sequence_id;
    });

  if (it != state_.edge_states.end())
  {
    state_.edge_states.erase(it);
  }
}

//=============================================================================
void StateManager::clear_order()
{
  std::lock_guard<std::mutex> lock(mutex_);

  state_.node_states.clear();
  state_.edge_states.clear();

  state_.order_id.clear();
  state_.order_update_id = 0;
  state_.zone_set_id.reset();
}

//=============================================================================
void StateManager::set_agv_position(const types::AGVPosition& position)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.agv_position = position;

  if (position.position_initialized)
  {
    position_initialized_ = true;
    map_id_ = position.map_id;
  }
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
