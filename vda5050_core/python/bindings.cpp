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

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "vda5050_core/client/adapter/state_manager.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/error_level.hpp"
#include "vda5050_core/types/error_reference.hpp"
#include "vda5050_core/types/info.hpp"
#include "vda5050_core/types/info_level.hpp"
#include "vda5050_core/types/info_reference.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/velocity.hpp"

#include "rmf_migration.hpp"

namespace py = pybind11;

using vda5050_core::python::rmf_migration::ActionExecutor;
using vda5050_core::python::rmf_migration::ActivityIdentifier;
using vda5050_core::python::rmf_migration::Adapter;
using vda5050_core::python::rmf_migration::CommandExecution;
using vda5050_core::python::rmf_migration::Destination;
using vda5050_core::python::rmf_migration::FleetConfiguration;
using vda5050_core::python::rmf_migration::FleetUpdateHandle;
using vda5050_core::python::rmf_migration::LocalizationRequest;
using vda5050_core::python::rmf_migration::NavigationRequest;
using vda5050_core::python::rmf_migration::RobotCallbacks;
using vda5050_core::python::rmf_migration::RobotConfiguration;
using vda5050_core::python::rmf_migration::RobotState;
using vda5050_core::python::rmf_migration::RobotUpdateHandle;
using vda5050_core::python::rmf_migration::StopRequest;

using vda5050_core::client::adapter::StateManager;
using vda5050_core::types::ActionState;
using vda5050_core::types::ActionStatus;
using vda5050_core::types::AGVPosition;
using vda5050_core::types::BatteryState;
using vda5050_core::types::Error;
using vda5050_core::types::ErrorLevel;
using vda5050_core::types::ErrorReference;
using vda5050_core::types::EStop;
using vda5050_core::types::Info;
using vda5050_core::types::InfoLevel;
using vda5050_core::types::InfoReference;
using vda5050_core::types::OperatingMode;
using vda5050_core::types::SafetyState;
using vda5050_core::types::Velocity;

PYBIND11_MODULE(vda5050_core_python, m)
{
  m.doc() = "VDA5050 Core Python bindings";

  auto m_client = m.def_submodule("client", "Native VDA5050 client API");

  py::enum_<ActionStatus>(m_client, "ActionStatus")
    .value("WAITING", ActionStatus::WAITING)
    .value("INITIALIZING", ActionStatus::INITIALIZING)
    .value("RUNNING", ActionStatus::RUNNING)
    .value("PAUSED", ActionStatus::PAUSED)
    .value("FINISHED", ActionStatus::FINISHED)
    .value("FAILED", ActionStatus::FAILED);

  py::enum_<EStop>(m_client, "EStop")
    .value("AUTOACK", EStop::AUTOACK)
    .value("MANUAL", EStop::MANUAL)
    .value("REMOTE", EStop::REMOTE)
    .value("NONE", EStop::NONE);

  py::enum_<ErrorLevel>(m_client, "ErrorLevel")
    .value("WARNING", ErrorLevel::WARNING)
    .value("FATAL", ErrorLevel::FATAL);

  py::enum_<InfoLevel>(m_client, "InfoLevel")
    .value("DEBUG", InfoLevel::DEBUG)
    .value("INFO", InfoLevel::INFO);

  py::enum_<OperatingMode>(m_client, "OperatingMode")
    .value("AUTOMATIC", OperatingMode::AUTOMATIC)
    .value("SEMIAUTOMATIC", OperatingMode::SEMIAUTOMATIC)
    .value("MANUAL", OperatingMode::MANUAL)
    .value("SERVICE", OperatingMode::SERVICE)
    .value("TEACHIN", OperatingMode::TEACHIN);

  py::class_<ActionState>(m_client, "ActionState")
    .def(py::init<>())
    .def_readwrite("action_id", &ActionState::action_id)
    .def_readwrite("action_type", &ActionState::action_type)
    .def_readwrite("action_description", &ActionState::action_description)
    .def_readwrite("action_status", &ActionState::action_status)
    .def_readwrite("result_description", &ActionState::result_description)
    .def("__eq__", &ActionState::operator==)
    .def("__ne__", &ActionState::operator!=);

  py::class_<BatteryState>(m_client, "BatteryState")
    .def(py::init<>())
    .def_readwrite("battery_charge", &BatteryState::battery_charge)
    .def_readwrite("battery_voltage", &BatteryState::battery_voltage)
    .def_readwrite("battery_health", &BatteryState::battery_health)
    .def_readwrite("charging", &BatteryState::charging)
    .def_readwrite("reach", &BatteryState::reach)
    .def("__eq__", &BatteryState::operator==)
    .def("__ne__", &BatteryState::operator!=);

  py::class_<AGVPosition>(m_client, "AGVPosition")
    .def(py::init<>())
    .def_readwrite("x", &AGVPosition::x)
    .def_readwrite("y", &AGVPosition::y)
    .def_readwrite("theta", &AGVPosition::theta)
    .def_readwrite("position_initialized", &AGVPosition::position_initialized)
    .def_readwrite("map_id", &AGVPosition::map_id)
    .def_readwrite("map_description", &AGVPosition::map_description)
    .def_readwrite("localization_score", &AGVPosition::localization_score)
    .def_readwrite("deviation_range", &AGVPosition::deviation_range)
    .def("__eq__", &AGVPosition::operator==)
    .def("__ne__", &AGVPosition::operator!=);

  py::class_<Velocity>(m_client, "Velocity")
    .def(py::init<>())
    .def_readwrite("vx", &Velocity::vx)
    .def_readwrite("vy", &Velocity::vy)
    .def_readwrite("omega", &Velocity::omega)
    .def("__eq__", &Velocity::operator==)
    .def("__ne__", &Velocity::operator!=);

  py::class_<SafetyState>(m_client, "SafetyState")
    .def(py::init<>())
    .def_readwrite("e_stop", &SafetyState::e_stop)
    .def_readwrite("field_violation", &SafetyState::field_violation)
    .def("__eq__", &SafetyState::operator==)
    .def("__ne__", &SafetyState::operator!=);

  py::class_<ErrorReference>(m_client, "ErrorReference")
    .def(py::init<>())
    .def_readwrite("reference_key", &ErrorReference::reference_key)
    .def_readwrite("reference_value", &ErrorReference::reference_value)
    .def("__eq__", &ErrorReference::operator==)
    .def("__ne__", &ErrorReference::operator!=);

  py::class_<Error>(m_client, "Error")
    .def(py::init<>())
    .def_readwrite("error_type", &Error::error_type)
    .def_readwrite("error_references", &Error::error_references)
    .def_readwrite("error_description", &Error::error_description)
    .def_readwrite("error_level", &Error::error_level)
    .def("__eq__", &Error::operator==)
    .def("__ne__", &Error::operator!=);

  py::class_<InfoReference>(m_client, "InfoReference")
    .def(py::init<>())
    .def_readwrite("reference_key", &InfoReference::reference_key)
    .def_readwrite("reference_value", &InfoReference::reference_value)
    .def("__eq__", &InfoReference::operator==)
    .def("__ne__", &InfoReference::operator!=);

  py::class_<Info>(m_client, "Info")
    .def(py::init<>())
    .def_readwrite("info_type", &Info::info_type)
    .def_readwrite("info_references", &Info::info_references)
    .def_readwrite("info_description", &Info::info_description)
    .def_readwrite("info_level", &Info::info_level)
    .def("__eq__", &Info::operator==)
    .def("__ne__", &Info::operator!=);

  py::class_<StateManager, std::shared_ptr<StateManager>>(
    m_client, "StateManager")
    .def("set_position", &StateManager::set_position)
    .def("set_velocity", &StateManager::set_velocity)
    .def("set_driving", &StateManager::set_driving)
    .def("set_paused", &StateManager::set_paused)
    .def("set_new_base_request", &StateManager::set_new_base_request)
    .def(
      "set_distance_since_last_node",
      &StateManager::set_distance_since_last_node)
    .def("set_battery_state", &StateManager::set_battery_state)
    .def("set_operating_mode", &StateManager::set_operating_mode)
    .def("set_safety_state", &StateManager::set_safety_state)
    .def("add_action_state", &StateManager::add_action_state)
    .def("set_action_states", &StateManager::set_action_states)
    .def("clear_action_states", &StateManager::clear_action_states)
    .def("add_error", &StateManager::add_error)
    .def("set_errors", &StateManager::set_errors)
    .def("clear_errors", &StateManager::clear_errors)
    .def("add_information", &StateManager::add_information)
    .def("set_information", &StateManager::set_information)
    .def("remove_information", &StateManager::remove_information);

  auto m_rmf_migration =
    m.def_submodule("rmf_migration", "Open-RMF style migration API");

  py::class_<ActivityIdentifier>(m_rmf_migration, "ActivityIdentifier")
    .def(
      py::init<std::optional<std::string>, std::optional<std::string>>(),
      py::arg("order_id") = std::nullopt, py::arg("action_id") = std::nullopt)
    .def_property_readonly("order_id", &ActivityIdentifier::order_id)
    .def_property_readonly("action_id", &ActivityIdentifier::action_id)
    .def("__eq__", &ActivityIdentifier::operator==)
    .def("__ne__", &ActivityIdentifier::operator!=);

  py::class_<RobotState>(m_rmf_migration, "RobotState")
    .def(
      py::init<std::string, std::array<double, 3>, double>(), py::arg("map"),
      py::arg("position"), py::arg("battery_soc"))
    .def_property("map", &RobotState::map, &RobotState::set_map)
    .def_property("position", &RobotState::position, &RobotState::set_position)
    .def_property(
      "battery_state_of_charge", &RobotState::battery_state_of_charge,
      &RobotState::set_battery_state_of_charge);

  py::class_<RobotConfiguration>(m_rmf_migration, "RobotConfiguration")
    .def(
      py::init<std::string, std::string, std::string, std::string>(),
      py::arg("manufacturer"), py::arg("serial_number"),
      py::arg("interface_name") = "uagv", py::arg("version") = "2.0.0")
    .def_readwrite("manufacturer", &RobotConfiguration::manufacturer)
    .def_readwrite("serial_number", &RobotConfiguration::serial_number)
    .def_readwrite("interface_name", &RobotConfiguration::interface_name)
    .def_readwrite("version", &RobotConfiguration::version)
    .def_readwrite("factsheet", &RobotConfiguration::factsheet);

  py::class_<Destination>(m_rmf_migration, "Destination")
    .def_property_readonly("map", &Destination::map)
    .def_property_readonly("position", &Destination::position)
    .def_property_readonly("xy", &Destination::xy)
    .def_property_readonly("yaw", &Destination::yaw)
    .def_property_readonly("graph_index", &Destination::graph_index)
    .def_property_readonly("name", &Destination::name)
    .def_property_readonly("speed_limit", &Destination::speed_limit);

  py::class_<CommandExecution>(m_rmf_migration, "CommandExecution")
    .def("finished", &CommandExecution::finished)
    .def("failed", &CommandExecution::failed)
    .def("okay", &CommandExecution::okay)
    .def("is_finished", &CommandExecution::is_finished)
    .def_property_readonly("identifier", &CommandExecution::identifier);

  py::class_<RobotCallbacks>(m_rmf_migration, "RobotCallbacks")
    .def(
      py::init<NavigationRequest, StopRequest, ActionExecutor>(),
      py::arg("navigate"), py::arg("stop"), py::arg("action_executor"))
    .def_property_readonly("navigate", &RobotCallbacks::navigate)
    .def_property_readonly("stop", &RobotCallbacks::stop)
    .def_property_readonly("action_executor", &RobotCallbacks::action_executor)
    .def_property(
      "localize", &RobotCallbacks::localize,
      &RobotCallbacks::with_localization);

  py::class_<RobotUpdateHandle, std::shared_ptr<RobotUpdateHandle>>(
    m_rmf_migration, "RobotUpdateHandle")
    .def("update", &RobotUpdateHandle::update)
    .def("more", [](RobotUpdateHandle& self) { return self.more(); });

  py::class_<FleetConfiguration>(m_rmf_migration, "FleetConfiguration")
    .def(
      py::init<std::string, std::string, std::string, int>(),
      py::arg("fleet_name"), py::arg("broker_uri"), py::arg("client_id_prefix"),
      py::arg("update_interval") = 30)
    .def_property(
      "fleet_name", &FleetConfiguration::fleet_name,
      &FleetConfiguration::set_fleet_name)
    .def_property(
      "broker_uri", &FleetConfiguration::broker_uri,
      &FleetConfiguration::set_broker_uri)
    .def_property(
      "client_id_prefix", &FleetConfiguration::client_id_prefix,
      &FleetConfiguration::set_client_id_prefix)
    .def_property(
      "update_interval", &FleetConfiguration::update_interval,
      &FleetConfiguration::set_update_interval)
    .def_property_readonly("known_robots", &FleetConfiguration::known_robots)
    .def(
      "add_known_robot_configuration",
      &FleetConfiguration::add_known_robot_configuration)
    .def(
      "get_known_robot_configuration",
      &FleetConfiguration::get_known_robot_configuration);

  py::class_<FleetUpdateHandle, std::shared_ptr<FleetUpdateHandle>>(
    m_rmf_migration, "FleetUpdateHandle")
    .def("add_robot", &FleetUpdateHandle::add_robot);

  py::class_<Adapter, std::shared_ptr<Adapter>>(m_rmf_migration, "Adapter")
    .def_static("make", &Adapter::make)
    .def(
      "add_vda5050_fleet", &Adapter::add_vda5050_fleet,
      py::arg("configuration"))
    .def("start", &Adapter::start)
    .def("stop", &Adapter::stop);
}
