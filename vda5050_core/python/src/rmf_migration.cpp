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

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/agv_position.hpp"

#include "rmf_migration.hpp"

namespace vda5050_core {

namespace python {

namespace rmf_migration {

//=============================================================================
RobotState::RobotState(
  std::string map, std::array<double, 3> position, double battery_soc)
: map_(std::move(map)), position_(position), battery_soc_(battery_soc)
{
  // Nothing to do here ...
}

//=============================================================================
const std::string& RobotState::map() const
{
  return map_;
}

//=============================================================================
void RobotState::set_map(std::string value)
{
  map_ = std::move(value);
}

//=============================================================================
std::array<double, 3> RobotState::position() const
{
  return position_;
}

//=============================================================================
void RobotState::set_position(std::array<double, 3> value)
{
  position_ = value;
}

//=============================================================================
double RobotState::battery_state_of_charge() const
{
  return battery_soc_;
}

//=============================================================================
void RobotState::set_battery_state_of_charge(double value)
{
  battery_soc_ = value;
}

//=============================================================================
types::AGVPosition RobotState::to_agv_position(
  std::string map, const std::array<double, 3>& position)
{
  types::AGVPosition pos;
  pos.x = position[0];
  pos.y = position[1];
  pos.theta = position[2];
  pos.map_id = std::move(map);

  return pos;
}

//=============================================================================
types::BatteryState RobotState::to_battery_state(double soc)
{
  types::BatteryState b;
  b.battery_charge = soc;
  return b;
}

//=============================================================================
RobotConfiguration::RobotConfiguration(
  std::string manufacturer, std::string serial_number,
  std::string interface_name, std::string version)
: manufacturer(std::move(manufacturer)),
  serial_number(std::move(serial_number)),
  interface_name(std::move(interface_name)),
  version(std::move(version))
{
  // Nothing to do here ...
}

//=============================================================================
const std::string& Destination::map() const
{
  return map_;
}

//=============================================================================
std::array<double, 3> Destination::position() const
{
  return position_;
}

//=============================================================================
std::array<double, 2> Destination::xy() const
{
  std::array<double, 2> destination;
  std::copy_n(position_.begin(), 2, destination.begin());
  return destination;
}

//=============================================================================
double Destination::yaw() const
{
  return position_[2];
}

//=============================================================================
uint32_t Destination::graph_index() const
{
  return graph_index_;
}

//=============================================================================
std::string Destination::name() const
{
  return name_;
}

//=============================================================================
std::optional<double> Destination::speed_limit() const
{
  return speed_limit_;
}

//=============================================================================
Destination::Destination(
  std::string map, std::array<double, 3> position, uint32_t graph_index,
  std::string name, std::optional<double> speed_limit)
: map_(std::move(map)),
  position_(position),
  graph_index_(graph_index),
  name_(name),
  speed_limit_(speed_limit)
{
  // Nothing to do here
}

//=============================================================================
ActivityIdentifier::ActivityIdentifier(
  std::optional<std::string> order_id, std::optional<std::string> action_id)
: order_id_(std::move(order_id)), action_id_(std::move(action_id))
{
  // Nothing to do here ...
}

//=============================================================================
const std::optional<std::string>& ActivityIdentifier::order_id() const
{
  return order_id_;
}

//=============================================================================
const std::optional<std::string>& ActivityIdentifier::action_id() const
{
  return action_id_;
}

//=============================================================================
bool ActivityIdentifier::operator==(const ActivityIdentifier& other) const
{
  return order_id_ == other.order_id_ && action_id_ == other.action_id_;
}

//=============================================================================
bool ActivityIdentifier::operator!=(const ActivityIdentifier& other) const
{
  return !(this->operator==(other));
}

//=============================================================================
void CommandExecution::finished()
{
  if (execution_)
  {
    execution_->finished();
  }
}

//=============================================================================
void CommandExecution::failed(const std::string& reason)
{
  if (execution_)
  {
    execution_->failed(reason);
  }
}

//=============================================================================
bool CommandExecution::okay() const
{
  if (!execution_)
  {
    return false;
  }
  return execution_->okay();
}

//=============================================================================
bool CommandExecution::is_finished() const
{
  if (!execution_)
  {
    return true;
  }

  return execution_->is_finished();
}

//=============================================================================
const ActivityIdentifier& CommandExecution::identifier() const
{
  return identifier_;
}

//=============================================================================
CommandExecution::CommandExecution(
  std::shared_ptr<client::adapter::Execution> execution,
  ActivityIdentifier identifier)
: execution_(std::move(execution)), identifier_(std::move(identifier))
{
  // Nothing to do here ...
}

//=============================================================================
RobotCallbacks::RobotCallbacks(
  NavigationRequest navigate, StopRequest stop, ActionExecutor action_executor)
: navigate_(std::move(navigate)),
  stop_(std::move(stop)),
  action_executor_(std::move(action_executor)),
  localize_(nullptr)
{
  // Nothing to do her ...
}

//=============================================================================
NavigationRequest RobotCallbacks::navigate() const
{
  return navigate_;
}

//=============================================================================
StopRequest RobotCallbacks::stop() const
{
  return stop_;
}

//=============================================================================
ActionExecutor RobotCallbacks::action_executor() const
{
  return action_executor_;
}

//=============================================================================
RobotCallbacks& RobotCallbacks::with_localization(
  LocalizationRequest localization)
{
  localize_ = std::move(localization);
  return *this;
}

//=============================================================================
LocalizationRequest RobotCallbacks::localize() const
{
  return localize_;
}

//=============================================================================
void RobotUpdateHandle::update(
  RobotState state, ActivityIdentifier /*identifier*/)
{
  adapter_->state_manager()->set_position(
    state.position()[0], state.position()[1], state.position()[2], state.map());
  adapter_->state_manager()->set_battery_state(
    RobotState::to_battery_state(state.battery_state_of_charge()));
}

//=============================================================================
std::shared_ptr<client::adapter::StateManager> RobotUpdateHandle::more()
{
  return adapter_->state_manager();
}

//=============================================================================
RobotUpdateHandle::RobotUpdateHandle(
  std::shared_ptr<client::adapter::Adapter> adapter)
: adapter_(std::move(adapter))
{
  // Nothing to do here ...
}

//=============================================================================
FleetConfiguration::FleetConfiguration(
  std::string fleet_name, std::string broker_uri, std::string client_id_prefix,
  int update_interval)
: fleet_name_(std::move(fleet_name)),
  broker_uri_(std::move(broker_uri)),
  client_id_prefix_(std::move(client_id_prefix)),
  update_interval_(update_interval)
{
  // Nothing to do here ...
}

//=============================================================================
std::optional<FleetConfiguration> FleetConfiguration::from_config_files(
  const std::string& /*config_file*/)
{
  // TODO(sauk2): Define a config and read off it. Another option is to
  // define a conversion from RMF config to VDA5050 factsheet
  return std::nullopt;
}

//=============================================================================
const std::string& FleetConfiguration::fleet_name() const
{
  return fleet_name_;
}

//=============================================================================
void FleetConfiguration::set_fleet_name(std::string value)
{
  fleet_name_ = std::move(value);
}

//=============================================================================
const std::unordered_map<std::string, RobotConfiguration>&
FleetConfiguration::known_robot_configurations() const
{
  return known_robot_configurations_;
}

//=============================================================================
std::vector<std::string> FleetConfiguration::known_robots() const
{
  std::vector<std::string> result;
  result.reserve(known_robot_configurations_.size());

  for (const auto& [name, _] : known_robot_configurations_)
  {
    result.push_back(name);
  }
  return result;
}

//=============================================================================
void FleetConfiguration::add_known_robot_configuration(
  std::string robot_name, RobotConfiguration config)
{
  known_robot_configurations_.insert_or_assign(
    std::move(robot_name), std::move(config));
}

//=============================================================================
std::optional<RobotConfiguration>
FleetConfiguration::get_known_robot_configuration(const std::string& name) const
{
  auto it = known_robot_configurations_.find(name);
  if (it != known_robot_configurations_.end()) return it->second;
  return std::nullopt;
}

//=============================================================================
const std::string& FleetConfiguration::broker_uri() const
{
  return broker_uri_;
}

//=============================================================================
void FleetConfiguration::set_broker_uri(std::string value)
{
  broker_uri_ = std::move(value);
}

//=============================================================================
const std::string& FleetConfiguration::client_id_prefix() const
{
  return client_id_prefix_;
}

//=============================================================================
void FleetConfiguration::set_client_id_prefix(std::string value)
{
  client_id_prefix_ = std::move(value);
}

//=============================================================================
int FleetConfiguration::update_interval() const
{
  return update_interval_;
}

//=============================================================================
void FleetConfiguration::set_update_interval(int value)
{
  update_interval_ = value;
}

//=============================================================================
std::shared_ptr<RobotUpdateHandle> FleetUpdateHandle::add_robot(
  std::string name, RobotState initial_state, RobotConfiguration configuration,
  RobotCallbacks callbacks)
{
  auto mqtt_client = transport::create_default_client_unique(
    configuration_.broker_uri(), configuration_.client_id_prefix());
  auto protocol_adapter = execution::ProtocolAdapter::make(
    std::move(mqtt_client), configuration.interface_name, configuration.version,
    configuration.manufacturer, configuration.serial_number);

  auto adapter = client::adapter::Adapter::make(protocol_adapter);
  adapter->state_manager()->set_position(
    initial_state.position()[0], initial_state.position()[1],
    initial_state.position()[2], initial_state.map());
  adapter->state_manager()->set_battery_state(
    RobotState::to_battery_state(initial_state.battery_state_of_charge()));

  adapter->on_navigate(
    [callbacks](
      client::adapter::NodeRequest node_request,
      std::optional<client::adapter::EdgeRequest> /*edge_request*/,
      std::shared_ptr<client::adapter::OrderExecution> execution) {
      const auto& node_position = node_request.node_position().value();
      auto destination = Destination(
        node_request.node_position().value().map_id,
        {node_position.x, node_position.y, node_position.theta.value_or(0.0)},
        node_request.sequence_id(), node_request.node_id());
      auto command =
        CommandExecution(execution, ActivityIdentifier(execution->order_id()));

      callbacks.navigate()(std::move(destination), std::move(command));
    });

  adapter->on_action(
    [callbacks](client::adapter::ActionRequest request, auto execution) {
      auto command = CommandExecution(
        execution, ActivityIdentifier(request.order_id(), request.action_id()));

      callbacks.action_executor()(
        request.action_type(), request.action_id(), std::move(command));
    });

  robots_.insert_or_assign(name, adapter);

  return std::shared_ptr<RobotUpdateHandle>(new RobotUpdateHandle(adapter));
}

//=============================================================================
FleetUpdateHandle::FleetUpdateHandle(FleetConfiguration configuration)
: configuration_(std::move(configuration))
{
  // Nothing to do here ...
}

//=============================================================================
std::shared_ptr<Adapter> Adapter::make()
{
  auto adapter = std::shared_ptr<Adapter>(new Adapter());
  return adapter;
}

//=============================================================================
std::shared_ptr<FleetUpdateHandle> Adapter::add_vda5050_fleet(
  FleetConfiguration configuration)
{
  auto fleet = std::shared_ptr<FleetUpdateHandle>(
    new FleetUpdateHandle(std::move(configuration)));
  fleets_.push_back(fleet);
  return fleet;
}

//=============================================================================
void Adapter::start()
{
  for (auto& fleet : fleets_)
  {
    for (auto& [name, adapter] : fleet->robots_)
    {
      adapter->start();
    }
  }
}

//=============================================================================
void Adapter::stop()
{
  for (auto& fleet : fleets_)
  {
    for (auto& [name, adapter] : fleet->robots_)
    {
      adapter->stop();
    }
  }
}

//=============================================================================
Adapter::Adapter()
{
  // Nothing to do here ...
}

}  // namespace rmf_migration
}  // namespace python
}  // namespace vda5050_core
