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

#ifndef RMF_MIGRATION_HPP_
#define RMF_MIGRATION_HPP_

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_core/client/adapter/adapter.hpp"

namespace vda5050_core {

namespace python {

namespace rmf_migration {

class RobotState
{
public:
  RobotState(
    std::string map, std::array<double, 3> position, double battery_soc);

  const std::string& map() const;

  void set_map(std::string value);

  std::array<double, 3> position() const;

  void set_position(std::array<double, 3> value);

  double battery_state_of_charge() const;

  void set_battery_state_of_charge(double value);

private:
  friend class FleetUpdateHandle;
  friend class RobotUpdateHandle;

  static types::AGVPosition to_agv_position(
    std::string map, const std::array<double, 3>& position);

  static types::BatteryState to_battery_state(double battery);

  std::string map_;
  std::array<double, 3> position_;
  double battery_soc_;
};

class RobotConfiguration
{
public:
  RobotConfiguration(
    std::string manufacturer, std::string serial_number,
    std::string interface_name = "uagv", std::string version = "2.0.0");

  std::string manufacturer;

  std::string serial_number;

  std::string interface_name;

  std::string version;

  std::optional<types::Factsheet> factsheet;
};

class Destination
{
public:
  const std::string& map() const;

  std::array<double, 3> position() const;

  std::array<double, 2> xy() const;

  double yaw() const;

  uint32_t graph_index() const;

  std::string name() const;

  std::optional<double> speed_limit() const;

private:
  friend class FleetUpdateHandle;

  Destination(
    std::string map, std::array<double, 3> position, uint32_t graph_index,
    std::string name, std::optional<double> speed_limit = std::nullopt);

  std::string map_;
  std::array<double, 3> position_;
  uint32_t graph_index_;
  std::string name_;
  std::optional<double> speed_limit_;
};

class ActivityIdentifier
{
public:
  ActivityIdentifier(
    std::optional<std::string> order_id = std::nullopt,
    std::optional<std::string> action_id = std::nullopt);

  const std::optional<std::string>& order_id() const;

  const std::optional<std::string>& action_id() const;

  bool operator==(const ActivityIdentifier& other) const;

  bool operator!=(const ActivityIdentifier& other) const;

private:
  std::optional<std::string> order_id_;
  std::optional<std::string> action_id_;
};

class CommandExecution
{
public:
  void finished();

  bool okay() const;

  bool is_finished() const;

  void failed(const std::string& reason);

  const ActivityIdentifier& identifier() const;

private:
  friend class FleetUpdateHandle;

  CommandExecution(
    std::shared_ptr<client::adapter::Execution> execution,
    ActivityIdentifier identifier);

  std::shared_ptr<client::adapter::Execution> execution_;
  ActivityIdentifier identifier_;
};

using NavigationRequest = std::function<void(Destination, CommandExecution)>;

using StopRequest = std::function<void()>;

using ActionExecutor =
  std::function<void(std::string, std::string, CommandExecution)>;

using LocalizationRequest = std::function<void(Destination, CommandExecution)>;

class RobotCallbacks
{
public:
  RobotCallbacks(
    NavigationRequest navigate, StopRequest stop,
    ActionExecutor action_executor);

  NavigationRequest navigate() const;

  StopRequest stop() const;

  ActionExecutor action_executor() const;

  RobotCallbacks& with_localization(LocalizationRequest localization);

  LocalizationRequest localize() const;

private:
  NavigationRequest navigate_;
  StopRequest stop_;
  ActionExecutor action_executor_;
  LocalizationRequest localize_;
};

class RobotUpdateHandle
{
public:
  void update(RobotState state, ActivityIdentifier identifier);

  std::shared_ptr<client::adapter::StateManager> more();

private:
  friend class FleetUpdateHandle;

  explicit RobotUpdateHandle(std::shared_ptr<client::adapter::Adapter> adapter);

  std::shared_ptr<client::adapter::Adapter> adapter_;
};

class FleetConfiguration
{
public:
  FleetConfiguration(
    std::string fleet_name, std::string broker_uri,
    std::string client_id_prefix, int update_interval = 30);

  static std::optional<FleetConfiguration> from_config_files(
    const std::string& config_file);

  const std::string& fleet_name() const;

  void set_fleet_name(std::string value);

  const std::unordered_map<std::string, RobotConfiguration>&
  known_robot_configurations() const;

  std::vector<std::string> known_robots() const;

  void add_known_robot_configuration(
    std::string robot_name, RobotConfiguration config);

  std::optional<RobotConfiguration> get_known_robot_configuration(
    const std::string& name) const;

  const std::string& broker_uri() const;

  void set_broker_uri(std::string value);

  const std::string& client_id_prefix() const;

  void set_client_id_prefix(std::string value);

  int update_interval() const;

  void set_update_interval(int value);

private:
  std::string fleet_name_;
  std::string broker_uri_;
  std::string client_id_prefix_;
  std::unordered_map<std::string, RobotConfiguration>
    known_robot_configurations_;
  int update_interval_;
};

class FleetUpdateHandle
{
public:
  std::shared_ptr<RobotUpdateHandle> add_robot(
    std::string name, RobotState initial_state,
    RobotConfiguration configuration, RobotCallbacks callbacks);

private:
  friend class Adapter;

  explicit FleetUpdateHandle(FleetConfiguration configuration);

  FleetConfiguration configuration_;

  std::unordered_map<std::string, std::shared_ptr<client::adapter::Adapter>>
    robots_;
};

class Adapter
{
public:
  static std::shared_ptr<Adapter> make();

  std::shared_ptr<FleetUpdateHandle> add_vda5050_fleet(
    FleetConfiguration configuration);

  void start();

  void stop();

private:
  Adapter();

  std::vector<std::shared_ptr<FleetUpdateHandle>> fleets_;
};

}  // namespace rmf_migration
}  // namespace python
}  // namespace vda5050_core

#endif  // RMF_MIGRATION_HPP_
