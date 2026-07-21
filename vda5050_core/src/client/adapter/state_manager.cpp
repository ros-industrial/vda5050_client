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
  double x, double y, double theta, const std::string& world_map_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!state_.agv_position.has_value())
  {
    state_.agv_position = types::AGVPosition{};
  }

  state_.agv_position->map_id = world_map_id;

  auto it = transformation_.find(world_map_id);
  if (it != transformation_.end())
  {
    auto world_pose = it->second.to_world_pose({x, y, theta});
    state_.agv_position->x = world_pose.x;
    state_.agv_position->y = world_pose.y;
    state_.agv_position->theta = world_pose.theta;
  }
  else
  {
    state_.agv_position->x = x;
    state_.agv_position->y = y;
    state_.agv_position->theta = theta;
  }
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
  publish_requested_ = true;
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
    state_.loads = std::vector<types::Load>{};
  }
  state_.loads->push_back(load);
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_loads(const std::vector<types::Load>& loads)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.loads = loads;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::clear_loads()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.loads = std::vector<types::Load>{};
  publish_requested_ = true;
}

//=============================================================================
void StateManager::remove_loads()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.loads = std::nullopt;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::add_information(const types::Info& information)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_.information.has_value())
  {
    state_.information = std::vector<types::Info>{};
  }
  state_.information->push_back(information);
  publish_requested_ = true;
}

//=============================================================================
void StateManager::set_information(const std::vector<types::Info>& infomation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.information = infomation;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::remove_information()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.information = std::nullopt;
  publish_requested_ = true;
}

//=============================================================================
void StateManager::initialize_position(
  double x, double y, double theta, const std::string& world_map_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  types::AGVPosition position;
  position.x = x;
  position.y = y;
  position.theta = theta;
  position.map_id = world_map_id;
  position.position_initialized = true;

  state_.agv_position = std::move(position);
}

//=============================================================================
void StateManager::set_transformation(
  const Transformation& transformation, const std::string& world_map_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  transformation_.insert_or_assign(world_map_id, transformation);
}

//=============================================================================
std::optional<Transformation> StateManager::transformation(
  const std::string& world_map_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = transformation_.find(world_map_id);
  if (it != transformation_.end()) return it->second;
  return std::nullopt;
}

//=============================================================================
types::State StateManager::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

//=============================================================================
bool StateManager::position_initialized() const
{
  if (state_.agv_position.has_value())
    return state_.agv_position->position_initialized;
  return false;
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
StateManager::StateManager() : publish_requested_(false)
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

  if (order.order_update_id == 0)
  {
    types::NodeState ns;
    ns.node_id = order.nodes.front().node_id;
    ns.sequence_id = order.nodes.front().sequence_id;
    ns.released = order.nodes.front().released;
    ns.node_description = order.nodes.front().node_description;
    ns.node_position = order.nodes.front().node_position;
    state_.node_states.push_back(std::move(ns));

    if (order.nodes.size() > 1)
    {
      types::EdgeState es;
      es.edge_id = order.edges.front().edge_id;
      es.sequence_id = order.edges.front().sequence_id;
      es.released = order.edges.front().released;
      es.edge_description = order.edges.front().edge_description;
      es.trajectory = order.edges.front().trajectory;

      state_.edge_states.push_back(std::move(es));
    }
  }

  for (size_t i = 1; i < order.nodes.size(); i++)
  {
    auto node = order.nodes[i];
    types::NodeState ns;
    ns.node_id = node.node_id;
    ns.sequence_id = node.sequence_id;
    ns.released = node.released;
    ns.node_description = node.node_description;
    ns.node_position = node.node_position;

    state_.node_states.push_back(std::move(ns));
  }

  for (size_t i = 1; i < order.edges.size(); i++)
  {
    auto edge = order.edges[i];
    types::EdgeState es;
    es.edge_id = edge.edge_id;
    es.sequence_id = edge.sequence_id;
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
void StateManager::set_last_node(
  const std::string& node_id, uint32_t sequence_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  state_.last_node_id = node_id;
  state_.last_node_sequence_id = sequence_id;
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
