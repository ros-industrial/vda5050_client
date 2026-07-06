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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__STATE_MANAGER_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__STATE_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/info.hpp"
#include "vda5050_core/types/load.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

class StateManager : public std::enable_shared_from_this<StateManager>
{
public:
  static std::shared_ptr<StateManager> make();

  void set_position(
    double x, double y, double theta, const std::string& map_id);

  void set_velocity(const types::Velocity& velocity);

  void set_driving(bool driving);

  void set_paused(bool paused);

  void set_new_base_request(bool request);

  void set_distance_since_last_node(double distance);

  void set_battery_state(const types::BatteryState& battery);

  void set_operating_mode(types::OperatingMode mode);

  void set_safety_state(const types::SafetyState& safety_state);

  void add_action_state(const types::ActionState& action_state);

  void set_action_states(const std::vector<types::ActionState>& action_states);

  void clear_action_states();

  void add_error(const types::Error& error);

  void set_errors(const std::vector<types::Error>& errors);

  void clear_errors();

  void add_load(const types::Load& load);

  void set_loads(const std::vector<types::Load>& loads);

  void clear_loads();

  void remove_loads();

  void add_information(const types::Info& information);

  void set_information(const std::vector<types::Info>& infomation);

  void remove_information();

  types::State state() const;

  void mark_publish_requested();

  bool consume_publish_requested();

private:
  friend class Adapter;

  StateManager();

  void set_order(const types::Order& order);

  void node_reached(const types::Node& node);

  void edge_traversed(const types::Edge& edge);

  void clear_order();

  void set_agv_position(const types::AGVPosition& position);

  mutable std::mutex mutex_;
  types::State state_;

  bool position_initialized_;
  std::string map_id_;

  std::atomic_bool publish_requested_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__STATE_MANAGER_HPP_
