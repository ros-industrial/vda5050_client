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
#include <pybind11_json/pybind11_json.hpp>

#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"
#include "vda5050_core/types/action_parameter_factsheet.hpp"
#include "vda5050_core/types/action_scope.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_action.hpp"
#include "vda5050_core/types/agv_class.hpp"
#include "vda5050_core/types/agv_geometry.hpp"
#include "vda5050_core/types/agv_kinematic.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/bounding_box_reference.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/control_point.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/edge_state.hpp"
#include "vda5050_core/types/envelope2d.hpp"
#include "vda5050_core/types/envelope3d.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/error_level.hpp"
#include "vda5050_core/types/error_reference.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/info.hpp"
#include "vda5050_core/types/info_level.hpp"
#include "vda5050_core/types/info_reference.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/load.hpp"
#include "vda5050_core/types/load_dimensions.hpp"
#include "vda5050_core/types/load_set.hpp"
#include "vda5050_core/types/load_specification.hpp"
#include "vda5050_core/types/max_array_lens.hpp"
#include "vda5050_core/types/max_string_lens.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/node_position.hpp"
#include "vda5050_core/types/node_state.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/optional_parameter.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/orientation_type.hpp"
#include "vda5050_core/types/physical_parameters.hpp"
#include "vda5050_core/types/polygon_point.hpp"
#include "vda5050_core/types/position.hpp"
#include "vda5050_core/types/protocol_features.hpp"
#include "vda5050_core/types/protocol_limits.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/support.hpp"
#include "vda5050_core/types/timing.hpp"
#include "vda5050_core/types/trajectory.hpp"
#include "vda5050_core/types/type_specification.hpp"
#include "vda5050_core/types/value_data_type.hpp"
#include "vda5050_core/types/velocity.hpp"
#include "vda5050_core/types/visualization.hpp"
#include "vda5050_core/types/wheel_definition.hpp"
#include "vda5050_core/types/wheel_definition_type.hpp"

#include "vda5050_core_py/types.hpp"

namespace vda5050_core_py {

using vda5050_core::types::Action;
using vda5050_core::types::ActionParameter;
using vda5050_core::types::ActionParameterFactsheet;
using vda5050_core::types::ActionScope;
using vda5050_core::types::ActionState;
using vda5050_core::types::ActionStatus;
using vda5050_core::types::AGVAction;
using vda5050_core::types::AGVClass;
using vda5050_core::types::AGVGeometry;
using vda5050_core::types::AGVKinematic;
using vda5050_core::types::AGVPosition;
using vda5050_core::types::BatteryState;
using vda5050_core::types::BlockingType;
using vda5050_core::types::BoundingBoxReference;
using vda5050_core::types::Connection;
using vda5050_core::types::ConnectionState;
using vda5050_core::types::ControlPoint;
using vda5050_core::types::Edge;
using vda5050_core::types::EdgeState;
using vda5050_core::types::Envelope2d;
using vda5050_core::types::Envelope3d;
using vda5050_core::types::Error;
using vda5050_core::types::ErrorLevel;
using vda5050_core::types::ErrorReference;
using vda5050_core::types::EStop;
using vda5050_core::types::Factsheet;
using vda5050_core::types::Header;
using vda5050_core::types::Info;
using vda5050_core::types::InfoLevel;
using vda5050_core::types::InfoReference;
using vda5050_core::types::InstantActions;
using vda5050_core::types::Load;
using vda5050_core::types::LoadDimensions;
using vda5050_core::types::LoadSet;
using vda5050_core::types::LoadSpecification;
using vda5050_core::types::MaxArrayLens;
using vda5050_core::types::MaxStringLens;
using vda5050_core::types::Node;
using vda5050_core::types::NodePosition;
using vda5050_core::types::NodeState;
using vda5050_core::types::OperatingMode;
using vda5050_core::types::OptionalParameter;
using vda5050_core::types::Order;
using vda5050_core::types::OrientationType;
using vda5050_core::types::PhysicalParameters;
using vda5050_core::types::PolygonPoint;
using vda5050_core::types::Position;
using vda5050_core::types::ProtocolFeatures;
using vda5050_core::types::ProtocolLimits;
using vda5050_core::types::SafetyState;
using vda5050_core::types::State;
using vda5050_core::types::Support;
using vda5050_core::types::Timing;
using vda5050_core::types::Trajectory;
using vda5050_core::types::TypeSpecification;
using vda5050_core::types::ValueDataType;
using vda5050_core::types::Velocity;
using vda5050_core::types::Visualization;
using vda5050_core::types::WheelDefinition;
using vda5050_core::types::WheelDefinitionType;

namespace {

// ── Enums ─────────────────────────────────────────────────────────────────────

void bind_types_action_status(py::module_& m)
{
  py::enum_<ActionStatus>(m, "ActionStatus")
    .value("WAITING", ActionStatus::WAITING)
    .value("INITIALIZING", ActionStatus::INITIALIZING)
    .value("RUNNING", ActionStatus::RUNNING)
    .value("PAUSED", ActionStatus::PAUSED)
    .value("FINISHED", ActionStatus::FINISHED)
    .value("FAILED", ActionStatus::FAILED);
}

void bind_types_e_stop(py::module_& m)
{
  py::enum_<EStop>(m, "EStop")
    .value("AUTOACK", EStop::AUTOACK)
    .value("MANUAL", EStop::MANUAL)
    .value("REMOTE", EStop::REMOTE)
    .value("NONE", EStop::NONE);
}

void bind_types_error_level(py::module_& m)
{
  py::enum_<ErrorLevel>(m, "ErrorLevel")
    .value("WARNING", ErrorLevel::WARNING)
    .value("FATAL", ErrorLevel::FATAL);
}

void bind_types_info_level(py::module_& m)
{
  py::enum_<InfoLevel>(m, "InfoLevel")
    .value("DEBUG", InfoLevel::DEBUG)
    .value("INFO", InfoLevel::INFO);
}

void bind_types_operating_mode(py::module_& m)
{
  py::enum_<OperatingMode>(m, "OperatingMode")
    .value("AUTOMATIC", OperatingMode::AUTOMATIC)
    .value("SEMIAUTOMATIC", OperatingMode::SEMIAUTOMATIC)
    .value("MANUAL", OperatingMode::MANUAL)
    .value("SERVICE", OperatingMode::SERVICE)
    .value("TEACHIN", OperatingMode::TEACHIN);
}

void bind_types_blocking_type(py::module_& m)
{
  py::enum_<BlockingType>(m, "BlockingType")
    .value("NONE", BlockingType::NONE)
    .value("SOFT", BlockingType::SOFT)
    .value("HARD", BlockingType::HARD);
}

void bind_types_connection_state(py::module_& m)
{
  py::enum_<ConnectionState>(m, "ConnectionState")
    .value("ONLINE", ConnectionState::ONLINE)
    .value("OFFLINE", ConnectionState::OFFLINE)
    .value("CONNECTIONBROKEN", ConnectionState::CONNECTIONBROKEN);
}

void bind_types_orientation_type(py::module_& m)
{
  py::enum_<OrientationType>(m, "OrientationType")
    .value("GLOBAL", OrientationType::GLOBAL)
    .value("TANGENTIAL", OrientationType::TANGENTIAL);
}

void bind_types_value_data_type(py::module_& m)
{
  py::enum_<ValueDataType>(m, "ValueDataType")
    .value("BOOL", ValueDataType::BOOL)
    .value("NUMBER", ValueDataType::NUMBER)
    .value("INTEGER", ValueDataType::INTEGER)
    .value("FLOAT", ValueDataType::FLOAT)
    .value("STRING", ValueDataType::STRING)
    .value("OBJECT", ValueDataType::OBJECT)
    .value("ARRAY", ValueDataType::ARRAY);
}

void bind_types_action_scope(py::module_& m)
{
  py::enum_<ActionScope>(m, "ActionScope")
    .value("INSTANT", ActionScope::INSTANT)
    .value("NODE", ActionScope::NODE)
    .value("EDGE", ActionScope::EDGE);
}

void bind_types_support(py::module_& m)
{
  py::enum_<Support>(m, "Support")
    .value("SUPPORTED", Support::SUPPORTED)
    .value("REQUIRED", Support::REQUIRED);
}

void bind_types_agv_kinematic(py::module_& m)
{
  py::enum_<AGVKinematic>(m, "AGVKinematic")
    .value("DIFF", AGVKinematic::DIFF)
    .value("OMNI", AGVKinematic::OMNI)
    .value("THREEWHEEL", AGVKinematic::THREEWHEEL);
}

void bind_types_agv_class(py::module_& m)
{
  py::enum_<AGVClass>(m, "AGVClass")
    .value("FORKLIFT", AGVClass::FORKLIFT)
    .value("CONVEYOR", AGVClass::CONVEYOR)
    .value("TUGGER", AGVClass::TUGGER)
    .value("CARRIER", AGVClass::CARRIER);
}

void bind_types_wheel_definition_type(py::module_& m)
{
  py::enum_<WheelDefinitionType>(m, "WheelDefinitionType")
    .value("DRIVE", WheelDefinitionType::DRIVE)
    .value("CASTER", WheelDefinitionType::CASTER)
    .value("FIXED", WheelDefinitionType::FIXED)
    .value("MECANUM", WheelDefinitionType::MECANUM);
}

void bind_types_action_state(py::module_& m)
{
  py::class_<ActionState>(m, "ActionState")
    .def(py::init<>())
    .def_readwrite("action_id", &ActionState::action_id)
    .def_readwrite("action_type", &ActionState::action_type)
    .def_readwrite("action_description", &ActionState::action_description)
    .def_readwrite("action_status", &ActionState::action_status)
    .def_readwrite("result_description", &ActionState::result_description)
    .def("__eq__", &ActionState::operator==)
    .def("__ne__", &ActionState::operator!=)
    .def("json", [](const ActionState& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> ActionState { return j; });
}

void bind_types_battery_state(py::module_& m)
{
  py::class_<BatteryState>(m, "BatteryState")
    .def(py::init<>())
    .def_readwrite("battery_charge", &BatteryState::battery_charge)
    .def_readwrite("battery_voltage", &BatteryState::battery_voltage)
    .def_readwrite("battery_health", &BatteryState::battery_health)
    .def_readwrite("charging", &BatteryState::charging)
    .def_readwrite("reach", &BatteryState::reach)
    .def("__eq__", &BatteryState::operator==)
    .def("__ne__", &BatteryState::operator!=)
    .def(
      "json", [](const BatteryState& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> BatteryState { return j; });
}

void bind_types_agv_position(py::module_& m)
{
  py::class_<AGVPosition>(m, "AGVPosition")
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
    .def("__ne__", &AGVPosition::operator!=)
    .def("json", [](const AGVPosition& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> AGVPosition { return j; });
}

void bind_types_velocity(py::module_& m)
{
  py::class_<Velocity>(m, "Velocity")
    .def(py::init<>())
    .def_readwrite("vx", &Velocity::vx)
    .def_readwrite("vy", &Velocity::vy)
    .def_readwrite("omega", &Velocity::omega)
    .def("__eq__", &Velocity::operator==)
    .def("__ne__", &Velocity::operator!=)
    .def("json", [](const Velocity& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Velocity { return j; });
}

void bind_types_safety_state(py::module_& m)
{
  py::class_<SafetyState>(m, "SafetyState")
    .def(py::init<>())
    .def_readwrite("e_stop", &SafetyState::e_stop)
    .def_readwrite("field_violation", &SafetyState::field_violation)
    .def("__eq__", &SafetyState::operator==)
    .def("__ne__", &SafetyState::operator!=)
    .def("json", [](const SafetyState& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> SafetyState { return j; });
}

void bind_types_error_reference(py::module_& m)
{
  py::class_<ErrorReference>(m, "ErrorReference")
    .def(py::init<>())
    .def_readwrite("reference_key", &ErrorReference::reference_key)
    .def_readwrite("reference_value", &ErrorReference::reference_value)
    .def("__eq__", &ErrorReference::operator==)
    .def("__ne__", &ErrorReference::operator!=)
    .def(
      "json", [](const ErrorReference& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> ErrorReference { return j; });
}

void bind_types_error(py::module_& m)
{
  py::class_<Error>(m, "Error")
    .def(py::init<>())
    .def_readwrite("error_type", &Error::error_type)
    .def_readwrite("error_references", &Error::error_references)
    .def_readwrite("error_description", &Error::error_description)
    .def_readwrite("error_level", &Error::error_level)
    .def("__eq__", &Error::operator==)
    .def("__ne__", &Error::operator!=)
    .def("json", [](const Error& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Error { return j; });
}

void bind_types_info_reference(py::module_& m)
{
  py::class_<InfoReference>(m, "InfoReference")
    .def(py::init<>())
    .def_readwrite("reference_key", &InfoReference::reference_key)
    .def_readwrite("reference_value", &InfoReference::reference_value)
    .def("__eq__", &InfoReference::operator==)
    .def("__ne__", &InfoReference::operator!=)
    .def(
      "json", [](const InfoReference& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> InfoReference { return j; });
}

void bind_types_info(py::module_& m)
{
  py::class_<Info>(m, "Info")
    .def(py::init<>())
    .def_readwrite("info_type", &Info::info_type)
    .def_readwrite("info_references", &Info::info_references)
    .def_readwrite("info_description", &Info::info_description)
    .def_readwrite("info_level", &Info::info_level)
    .def("__eq__", &Info::operator==)
    .def("__ne__", &Info::operator!=)
    .def("json", [](const Info& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> Info { return j; });
}

void bind_types_action_parameter(py::module_& m)
{
  py::class_<ActionParameter>(m, "ActionParameter")
    .def(py::init<>())
    .def_readwrite("key", &ActionParameter::key)
    .def_readwrite("value", &ActionParameter::value)
    .def("__eq__", &ActionParameter::operator==)
    .def("__ne__", &ActionParameter::operator!=)
    .def(
      "json",
      [](const ActionParameter& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> ActionParameter {
      return j;
    });
}

void bind_types_action_parameter_factsheet(py::module_& m)
{
  py::class_<ActionParameterFactsheet>(m, "ActionParameterFactsheet")
    .def(py::init<>())
    .def_readwrite("key", &ActionParameterFactsheet::key)
    .def_readwrite(
      "value_data_type", &ActionParameterFactsheet::value_data_type)
    .def_readwrite("description", &ActionParameterFactsheet::description)
    .def_readwrite("is_optional", &ActionParameterFactsheet::is_optional)
    .def("__eq__", &ActionParameterFactsheet::operator==)
    .def("__ne__", &ActionParameterFactsheet::operator!=)
    .def(
      "json",
      [](const ActionParameterFactsheet& self) -> nlohmann::json {
        return self;
      })
    .def_static(
      "from_json",
      [](const nlohmann::json& j) -> ActionParameterFactsheet { return j; });
}

void bind_types_bounding_box_reference(py::module_& m)
{
  py::class_<BoundingBoxReference>(m, "BoundingBoxReference")
    .def(py::init<>())
    .def_readwrite("x", &BoundingBoxReference::x)
    .def_readwrite("y", &BoundingBoxReference::y)
    .def_readwrite("z", &BoundingBoxReference::z)
    .def_readwrite("theta", &BoundingBoxReference::theta)
    .def("__eq__", &BoundingBoxReference::operator==)
    .def("__ne__", &BoundingBoxReference::operator!=)
    .def(
      "json",
      [](const BoundingBoxReference& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json",
      [](const nlohmann::json& j) -> BoundingBoxReference { return j; });
}

void bind_types_control_point(py::module_& m)
{
  py::class_<ControlPoint>(m, "ControlPoint")
    .def(py::init<>())
    .def_readwrite("x", &ControlPoint::x)
    .def_readwrite("y", &ControlPoint::y)
    .def_readwrite("weight", &ControlPoint::weight)
    .def("__eq__", &ControlPoint::operator==)
    .def("__ne__", &ControlPoint::operator!=)
    .def(
      "json", [](const ControlPoint& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> ControlPoint { return j; });
}

void bind_types_polygon_point(py::module_& m)
{
  py::class_<PolygonPoint>(m, "PolygonPoint")
    .def(py::init<>())
    .def_readwrite("x", &PolygonPoint::x)
    .def_readwrite("y", &PolygonPoint::y)
    .def("__eq__", &PolygonPoint::operator==)
    .def("__ne__", &PolygonPoint::operator!=)
    .def(
      "json", [](const PolygonPoint& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> PolygonPoint { return j; });
}

void bind_types_position(py::module_& m)
{
  py::class_<Position>(m, "Position")
    .def(py::init<>())
    .def_readwrite("x", &Position::x)
    .def_readwrite("y", &Position::y)
    .def_readwrite("theta", &Position::theta)
    .def("__eq__", &Position::operator==)
    .def("__ne__", &Position::operator!=)
    .def("json", [](const Position& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Position { return j; });
}

void bind_types_load_dimensions(py::module_& m)
{
  py::class_<LoadDimensions>(m, "LoadDimensions")
    .def(py::init<>())
    .def_readwrite("length", &LoadDimensions::length)
    .def_readwrite("width", &LoadDimensions::width)
    .def_readwrite("height", &LoadDimensions::height)
    .def("__eq__", &LoadDimensions::operator==)
    .def("__ne__", &LoadDimensions::operator!=)
    .def(
      "json", [](const LoadDimensions& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> LoadDimensions { return j; });
}

void bind_types_max_string_lens(py::module_& m)
{
  py::class_<MaxStringLens>(m, "MaxStringLens")
    .def(py::init<>())
    .def_readwrite("msg_len", &MaxStringLens::msg_len)
    .def_readwrite("topic_serial_len", &MaxStringLens::topic_serial_len)
    .def_readwrite("topic_elem_len", &MaxStringLens::topic_elem_len)
    .def_readwrite("id_len", &MaxStringLens::id_len)
    .def_readwrite("enum_len", &MaxStringLens::enum_len)
    .def_readwrite("load_id_len", &MaxStringLens::load_id_len)
    .def_readwrite("id_numerical_only", &MaxStringLens::id_numerical_only)
    .def("__eq__", &MaxStringLens::operator==)
    .def("__ne__", &MaxStringLens::operator!=)
    .def(
      "json", [](const MaxStringLens& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> MaxStringLens { return j; });
}

void bind_types_max_array_lens(py::module_& m)
{
  py::class_<MaxArrayLens>(m, "MaxArrayLens")
    .def(py::init<>())
    .def_readwrite("order_nodes", &MaxArrayLens::order_nodes)
    .def_readwrite("order_edges", &MaxArrayLens::order_edges)
    .def_readwrite("node_actions", &MaxArrayLens::node_actions)
    .def_readwrite("edge_actions", &MaxArrayLens::edge_actions)
    .def_readwrite(
      "actions_actions_parameters", &MaxArrayLens::actions_actions_parameters)
    .def_readwrite("instant_actions", &MaxArrayLens::instant_actions)
    .def_readwrite(
      "trajectory_knot_vector", &MaxArrayLens::trajectory_knot_vector)
    .def_readwrite(
      "trajectory_control_points", &MaxArrayLens::trajectory_control_points)
    .def_readwrite("state_node_states", &MaxArrayLens::state_node_states)
    .def_readwrite("state_edge_states", &MaxArrayLens::state_edge_states)
    .def_readwrite("state_loads", &MaxArrayLens::state_loads)
    .def_readwrite("state_action_states", &MaxArrayLens::state_action_states)
    .def_readwrite("state_errors", &MaxArrayLens::state_errors)
    .def_readwrite("state_information", &MaxArrayLens::state_information)
    .def_readwrite(
      "error_error_references", &MaxArrayLens::error_error_references)
    .def_readwrite(
      "information_info_references", &MaxArrayLens::information_info_references)
    .def("__eq__", &MaxArrayLens::operator==)
    .def("__ne__", &MaxArrayLens::operator!=)
    .def(
      "json", [](const MaxArrayLens& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> MaxArrayLens { return j; });
}

void bind_types_timing(py::module_& m)
{
  py::class_<Timing>(m, "Timing")
    .def(py::init<>())
    .def_readwrite("min_order_interval", &Timing::min_order_interval)
    .def_readwrite("min_state_interval", &Timing::min_state_interval)
    .def_readwrite("default_state_interval", &Timing::default_state_interval)
    .def_readwrite("visualization_interval", &Timing::visualization_interval)
    .def("__eq__", &Timing::operator==)
    .def("__ne__", &Timing::operator!=)
    .def("json", [](const Timing& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Timing { return j; });
}

void bind_types_physical_parameters(py::module_& m)
{
  py::class_<PhysicalParameters>(m, "PhysicalParameters")
    .def(py::init<>())
    .def_readwrite("speed_min", &PhysicalParameters::speed_min)
    .def_readwrite("speed_max", &PhysicalParameters::speed_max)
    .def_readwrite("acceleration_max", &PhysicalParameters::acceleration_max)
    .def_readwrite("deceleration_max", &PhysicalParameters::deceleration_max)
    .def_readwrite("height_min", &PhysicalParameters::height_min)
    .def_readwrite("height_max", &PhysicalParameters::height_max)
    .def_readwrite("width", &PhysicalParameters::width)
    .def_readwrite("length", &PhysicalParameters::length)
    .def_readwrite("angular_speed_min", &PhysicalParameters::angular_speed_min)
    .def_readwrite("angular_speed_max", &PhysicalParameters::angular_speed_max)
    .def("__eq__", &PhysicalParameters::operator==)
    .def("__ne__", &PhysicalParameters::operator!=)
    .def(
      "json",
      [](const PhysicalParameters& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> PhysicalParameters {
      return j;
    });
}

void bind_types_header(py::module_& m)
{
  py::class_<Header>(m, "Header")
    .def(py::init<>())
    .def_readwrite("header_id", &Header::header_id)
    .def_property(
      "timestamp",
      [](const Header& self) -> double {
        return std::chrono::duration<double>(self.timestamp.time_since_epoch())
          .count();
      },
      [](Header& self, double t) {
        self.timestamp = std::chrono::time_point<std::chrono::system_clock>(
          std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(t)));
      })
    .def_readwrite("version", &Header::version)
    .def_readwrite("manufacturer", &Header::manufacturer)
    .def_readwrite("serial_number", &Header::serial_number)
    .def("__eq__", &Header::operator==)
    .def("__ne__", &Header::operator!=)
    .def("json", [](const Header& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Header { return j; });
}

void bind_types_action(py::module_& m)
{
  py::class_<Action>(m, "Action")
    .def(py::init<>())
    .def_readwrite("action_type", &Action::action_type)
    .def_readwrite("action_id", &Action::action_id)
    .def_readwrite("blocking_type", &Action::blocking_type)
    .def_readwrite("action_description", &Action::action_description)
    .def_readwrite("action_parameters", &Action::action_parameters)
    .def("__eq__", &Action::operator==)
    .def("__ne__", &Action::operator!=)
    .def("json", [](const Action& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Action { return j; });
}

void bind_types_agv_action(py::module_& m)
{
  py::class_<AGVAction>(m, "AGVAction")
    .def(py::init<>())
    .def_readwrite("action_type", &AGVAction::action_type)
    .def_readwrite("action_scopes", &AGVAction::action_scopes)
    .def_readwrite("action_parameters", &AGVAction::action_parameters)
    .def_readwrite("result_description", &AGVAction::result_description)
    .def_readwrite("action_description", &AGVAction::action_description)
    .def_readwrite("blocking_types", &AGVAction::blocking_types)
    .def("__eq__", &AGVAction::operator==)
    .def("__ne__", &AGVAction::operator!=)
    .def("json", [](const AGVAction& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> AGVAction { return j; });
}

void bind_types_optional_parameter(py::module_& m)
{
  py::class_<OptionalParameter>(m, "OptionalParameter")
    .def(py::init<>())
    .def_readwrite("parameter", &OptionalParameter::parameter)
    .def_readwrite("support", &OptionalParameter::support)
    .def_readwrite("description", &OptionalParameter::description)
    .def("__eq__", &OptionalParameter::operator==)
    .def("__ne__", &OptionalParameter::operator!=)
    .def(
      "json",
      [](const OptionalParameter& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> OptionalParameter {
      return j;
    });
}

void bind_types_trajectory(py::module_& m)
{
  py::class_<Trajectory>(m, "Trajectory")
    .def(py::init<>())
    .def_readwrite("degree", &Trajectory::degree)
    .def_readwrite("knot_vector", &Trajectory::knot_vector)
    .def_readwrite("control_points", &Trajectory::control_points)
    .def("__eq__", &Trajectory::operator==)
    .def("__ne__", &Trajectory::operator!=)
    .def("json", [](const Trajectory& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Trajectory { return j; });
}

void bind_types_envelope2d(py::module_& m)
{
  py::class_<Envelope2d>(m, "Envelope2d")
    .def(py::init<>())
    .def_readwrite("set", &Envelope2d::set)
    .def_readwrite("polygon_points", &Envelope2d::polygon_points)
    .def_readwrite("description", &Envelope2d::description)
    .def("__eq__", &Envelope2d::operator==)
    .def("__ne__", &Envelope2d::operator!=)
    .def("json", [](const Envelope2d& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Envelope2d { return j; });
}

void bind_types_envelope3d(py::module_& m)
{
  py::class_<Envelope3d>(m, "Envelope3d")
    .def(py::init<>())
    .def_readwrite("set", &Envelope3d::set)
    .def_readwrite("format", &Envelope3d::format)
    .def_readwrite("data", &Envelope3d::data)
    .def_readwrite("url", &Envelope3d::url)
    .def_readwrite("description", &Envelope3d::description)
    .def("__eq__", &Envelope3d::operator==)
    .def("__ne__", &Envelope3d::operator!=)
    .def("json", [](const Envelope3d& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Envelope3d { return j; });
}

void bind_types_node_position(py::module_& m)
{
  py::class_<NodePosition>(m, "NodePosition")
    .def(py::init<>())
    .def_readwrite("x", &NodePosition::x)
    .def_readwrite("y", &NodePosition::y)
    .def_readwrite("theta", &NodePosition::theta)
    .def_readwrite(
      "allowed_deviation_x_y", &NodePosition::allowed_deviation_x_y)
    .def_readwrite(
      "allowed_deviation_theta", &NodePosition::allowed_deviation_theta)
    .def_readwrite("map_id", &NodePosition::map_id)
    .def_readwrite("map_description", &NodePosition::map_description)
    .def("__eq__", &NodePosition::operator==)
    .def("__ne__", &NodePosition::operator!=)
    .def(
      "json", [](const NodePosition& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> NodePosition { return j; });
}

void bind_types_load(py::module_& m)
{
  py::class_<Load>(m, "Load")
    .def(py::init<>())
    .def_readwrite("load_id", &Load::load_id)
    .def_readwrite("load_type", &Load::load_type)
    .def_readwrite("load_position", &Load::load_position)
    .def_readwrite("bounding_box_reference", &Load::bounding_box_reference)
    .def_readwrite("load_dimensions", &Load::load_dimensions)
    .def_readwrite("weight", &Load::weight)
    .def("__eq__", &Load::operator==)
    .def("__ne__", &Load::operator!=)
    .def("json", [](const Load& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> Load { return j; });
}

void bind_types_load_set(py::module_& m)
{
  py::class_<LoadSet>(m, "LoadSet")
    .def(py::init<>())
    .def_readwrite("set_name", &LoadSet::set_name)
    .def_readwrite("load_type", &LoadSet::load_type)
    .def_readwrite("load_positions", &LoadSet::load_positions)
    .def_readwrite("bounding_box_reference", &LoadSet::bounding_box_reference)
    .def_readwrite("load_dimensions", &LoadSet::load_dimensions)
    .def_readwrite("max_weight", &LoadSet::max_weight)
    .def_readwrite(
      "min_load_handling_height", &LoadSet::min_load_handling_height)
    .def_readwrite(
      "max_load_handling_height", &LoadSet::max_load_handling_height)
    .def_readwrite("min_load_handling_depth", &LoadSet::min_load_handling_depth)
    .def_readwrite("max_load_handling_depth", &LoadSet::max_load_handling_depth)
    .def_readwrite("min_load_handling_tilt", &LoadSet::min_load_handling_tilt)
    .def_readwrite("max_load_handling_tilt", &LoadSet::max_load_handling_tilt)
    .def_readwrite("agv_speed_limit", &LoadSet::agv_speed_limit)
    .def_readwrite("agv_acceleration_limit", &LoadSet::agv_acceleration_limit)
    .def_readwrite("agv_deceleration_limit", &LoadSet::agv_deceleration_limit)
    .def_readwrite("pick_time", &LoadSet::pick_time)
    .def_readwrite("drop_time", &LoadSet::drop_time)
    .def_readwrite("description", &LoadSet::description)
    .def("__eq__", &LoadSet::operator==)
    .def("__ne__", &LoadSet::operator!=)
    .def("json", [](const LoadSet& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> LoadSet { return j; });
}

void bind_types_load_specification(py::module_& m)
{
  py::class_<LoadSpecification>(m, "LoadSpecification")
    .def(py::init<>())
    .def_readwrite("load_positions", &LoadSpecification::load_positions)
    .def_readwrite("load_sets", &LoadSpecification::load_sets)
    .def("__eq__", &LoadSpecification::operator==)
    .def("__ne__", &LoadSpecification::operator!=)
    .def(
      "json",
      [](const LoadSpecification& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> LoadSpecification {
      return j;
    });
}

void bind_types_wheel_definition(py::module_& m)
{
  py::class_<WheelDefinition>(m, "WheelDefinition")
    .def(py::init<>())
    .def_readwrite("type", &WheelDefinition::type)
    .def_readwrite("is_active_driven", &WheelDefinition::is_active_driven)
    .def_readwrite("is_active_steered", &WheelDefinition::is_active_steered)
    .def_readwrite("position", &WheelDefinition::position)
    .def_readwrite("diameter", &WheelDefinition::diameter)
    .def_readwrite("width", &WheelDefinition::width)
    .def_readwrite("center_displacement", &WheelDefinition::center_displacement)
    .def_readwrite("constraints", &WheelDefinition::constraints)
    .def("__eq__", &WheelDefinition::operator==)
    .def("__ne__", &WheelDefinition::operator!=)
    .def(
      "json",
      [](const WheelDefinition& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> WheelDefinition {
      return j;
    });
}

void bind_types_agv_geometry(py::module_& m)
{
  py::class_<AGVGeometry>(m, "AGVGeometry")
    .def(py::init<>())
    .def_readwrite("wheel_definitions", &AGVGeometry::wheel_definitions)
    .def_readwrite("envelopes2d", &AGVGeometry::envelopes2d)
    .def_readwrite("envelopes3d", &AGVGeometry::envelopes3d)
    .def("__eq__", &AGVGeometry::operator==)
    .def("__ne__", &AGVGeometry::operator!=)
    .def("json", [](const AGVGeometry& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> AGVGeometry { return j; });
}

void bind_types_edge_state(py::module_& m)
{
  py::class_<EdgeState>(m, "EdgeState")
    .def(py::init<>())
    .def_readwrite("edge_id", &EdgeState::edge_id)
    .def_readwrite("sequence_id", &EdgeState::sequence_id)
    .def_readwrite("edge_description", &EdgeState::edge_description)
    .def_readwrite("released", &EdgeState::released)
    .def_readwrite("trajectory", &EdgeState::trajectory)
    .def("__eq__", &EdgeState::operator==)
    .def("__ne__", &EdgeState::operator!=)
    .def("json", [](const EdgeState& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> EdgeState { return j; });
}

void bind_types_node_state(py::module_& m)
{
  py::class_<NodeState>(m, "NodeState")
    .def(py::init<>())
    .def_readwrite("node_id", &NodeState::node_id)
    .def_readwrite("sequence_id", &NodeState::sequence_id)
    .def_readwrite("node_description", &NodeState::node_description)
    .def_readwrite("node_position", &NodeState::node_position)
    .def_readwrite("released", &NodeState::released)
    .def("__eq__", &NodeState::operator==)
    .def("__ne__", &NodeState::operator!=)
    .def("json", [](const NodeState& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> NodeState { return j; });
}

void bind_types_edge(py::module_& m)
{
  py::class_<Edge>(m, "Edge")
    .def(py::init<>())
    .def_readwrite("edge_id", &Edge::edge_id)
    .def_readwrite("sequence_id", &Edge::sequence_id)
    .def_readwrite("start_node_id", &Edge::start_node_id)
    .def_readwrite("end_node_id", &Edge::end_node_id)
    .def_readwrite("released", &Edge::released)
    .def_readwrite("actions", &Edge::actions)
    .def_readwrite("edge_description", &Edge::edge_description)
    .def_readwrite("max_speed", &Edge::max_speed)
    .def_readwrite("max_height", &Edge::max_height)
    .def_readwrite("min_height", &Edge::min_height)
    .def_readwrite("orientation", &Edge::orientation)
    .def_readwrite("orientation_type", &Edge::orientation_type)
    .def_readwrite("direction", &Edge::direction)
    .def_readwrite("rotation_allowed", &Edge::rotation_allowed)
    .def_readwrite("max_rotation_speed", &Edge::max_rotation_speed)
    .def_readwrite("trajectory", &Edge::trajectory)
    .def_readwrite("length", &Edge::length)
    .def("__eq__", &Edge::operator==)
    .def("__ne__", &Edge::operator!=)
    .def("json", [](const Edge& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> Edge { return j; });
}

void bind_types_node(py::module_& m)
{
  py::class_<Node>(m, "Node")
    .def(py::init<>())
    .def_readwrite("node_id", &Node::node_id)
    .def_readwrite("sequence_id", &Node::sequence_id)
    .def_readwrite("released", &Node::released)
    .def_readwrite("actions", &Node::actions)
    .def_readwrite("node_position", &Node::node_position)
    .def_readwrite("node_description", &Node::node_description)
    .def("__eq__", &Node::operator==)
    .def("__ne__", &Node::operator!=)
    .def("json", [](const Node& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> Node { return j; });
}

void bind_types_type_specification(py::module_& m)
{
  py::class_<TypeSpecification>(m, "TypeSpecification")
    .def(py::init<>())
    .def_readwrite("series_name", &TypeSpecification::series_name)
    .def_readwrite("agv_kinematic", &TypeSpecification::agv_kinematic)
    .def_readwrite("agv_class", &TypeSpecification::agv_class)
    .def_readwrite("max_load_mass", &TypeSpecification::max_load_mass)
    .def_readwrite("localization_types", &TypeSpecification::localization_types)
    .def_readwrite("navigation_types", &TypeSpecification::navigation_types)
    .def_readwrite("series_description", &TypeSpecification::series_description)
    .def("__eq__", &TypeSpecification::operator==)
    .def("__ne__", &TypeSpecification::operator!=)
    .def(
      "json",
      [](const TypeSpecification& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> TypeSpecification {
      return j;
    });
}

void bind_types_protocol_limits(py::module_& m)
{
  py::class_<ProtocolLimits>(m, "ProtocolLimits")
    .def(py::init<>())
    .def_readwrite("max_string_lens", &ProtocolLimits::max_string_lens)
    .def_readwrite("max_array_lens", &ProtocolLimits::max_array_lens)
    .def_readwrite("timing", &ProtocolLimits::timing)
    .def("__eq__", &ProtocolLimits::operator==)
    .def("__ne__", &ProtocolLimits::operator!=)
    .def(
      "json", [](const ProtocolLimits& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> ProtocolLimits { return j; });
}

void bind_types_protocol_features(py::module_& m)
{
  py::class_<ProtocolFeatures>(m, "ProtocolFeatures")
    .def(py::init<>())
    .def_readwrite(
      "optional_parameters", &ProtocolFeatures::optional_parameters)
    .def_readwrite("agv_actions", &ProtocolFeatures::agv_actions)
    .def("__eq__", &ProtocolFeatures::operator==)
    .def("__ne__", &ProtocolFeatures::operator!=)
    .def(
      "json",
      [](const ProtocolFeatures& self) -> nlohmann::json { return self; })
    .def_static("from_json", [](const nlohmann::json& j) -> ProtocolFeatures {
      return j;
    });
}

void bind_types_connection(py::module_& m)
{
  py::class_<Connection>(m, "Connection")
    .def(py::init<>())
    .def_readwrite("header", &Connection::header)
    .def_readwrite("connection_state", &Connection::connection_state)
    .def("__eq__", &Connection::operator==)
    .def("__ne__", &Connection::operator!=)
    .def("json", [](const Connection& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Connection { return j; });
}

void bind_types_instant_actions(py::module_& m)
{
  py::class_<InstantActions>(m, "InstantActions")
    .def(py::init<>())
    .def_readwrite("header", &InstantActions::header)
    .def_readwrite("actions", &InstantActions::actions)
    .def("__eq__", &InstantActions::operator==)
    .def("__ne__", &InstantActions::operator!=)
    .def(
      "json", [](const InstantActions& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> InstantActions { return j; });
}

void bind_types_order(py::module_& m)
{
  py::class_<Order>(m, "Order")
    .def(py::init<>())
    .def_readwrite("header", &Order::header)
    .def_readwrite("order_id", &Order::order_id)
    .def_readwrite("order_update_id", &Order::order_update_id)
    .def_readwrite("nodes", &Order::nodes)
    .def_readwrite("edges", &Order::edges)
    .def_readwrite("zone_set_id", &Order::zone_set_id)
    .def("__eq__", &Order::operator==)
    .def("__ne__", &Order::operator!=)
    .def("json", [](const Order& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Order { return j; });
}

void bind_types_visualization(py::module_& m)
{
  py::class_<Visualization>(m, "Visualization")
    .def(py::init<>())
    .def_readwrite("header", &Visualization::header)
    .def_readwrite("agv_position", &Visualization::agv_position)
    .def_readwrite("velocity", &Visualization::velocity)
    .def("__eq__", &Visualization::operator==)
    .def("__ne__", &Visualization::operator!=)
    .def(
      "json", [](const Visualization& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Visualization { return j; });
}

void bind_types_factsheet(py::module_& m)
{
  py::class_<Factsheet>(m, "Factsheet")
    .def(py::init<>())
    .def_readwrite("header", &Factsheet::header)
    .def_readwrite("type_specification", &Factsheet::type_specification)
    .def_readwrite("physical_parameters", &Factsheet::physical_parameters)
    .def_readwrite("protocol_limits", &Factsheet::protocol_limits)
    .def_readwrite("protocol_features", &Factsheet::protocol_features)
    .def_readwrite("agv_geometry", &Factsheet::agv_geometry)
    .def_readwrite("load_specification", &Factsheet::load_specification)
    .def_readwrite(
      "localization_parameters", &Factsheet::localization_parameters)
    .def("__eq__", &Factsheet::operator==)
    .def("__ne__", &Factsheet::operator!=)
    .def("json", [](const Factsheet& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> Factsheet { return j; });
}

void bind_types_state(py::module_& m)
{
  py::class_<State>(m, "State")
    .def(py::init<>())
    .def_readwrite("header", &State::header)
    .def_readwrite("order_id", &State::order_id)
    .def_readwrite("order_update_id", &State::order_update_id)
    .def_readwrite("zone_set_id", &State::zone_set_id)
    .def_readwrite("last_node_id", &State::last_node_id)
    .def_readwrite("last_node_sequence_id", &State::last_node_sequence_id)
    .def_readwrite("node_states", &State::node_states)
    .def_readwrite("edge_states", &State::edge_states)
    .def_readwrite("agv_position", &State::agv_position)
    .def_readwrite("velocity", &State::velocity)
    .def_readwrite("loads", &State::loads)
    .def_readwrite("driving", &State::driving)
    .def_readwrite("paused", &State::paused)
    .def_readwrite("new_base_request", &State::new_base_request)
    .def_readwrite("distance_since_last_node", &State::distance_since_last_node)
    .def_readwrite("action_states", &State::action_states)
    .def_readwrite("battery_state", &State::battery_state)
    .def_readwrite("operating_mode", &State::operating_mode)
    .def_readwrite("errors", &State::errors)
    .def_readwrite("information", &State::information)
    .def_readwrite("safety_state", &State::safety_state)
    .def("__eq__", &State::operator==)
    .def("__ne__", &State::operator!=)
    .def("json", [](const State& self) -> nlohmann::json { return self; })
    .def_static(
      "from_json", [](const nlohmann::json& j) -> State { return j; });
}

}  // namespace

void bind_types(py::module_& m)
{
  auto m_types = m.def_submodule("types", "VDA5050 types");

  // Enums
  bind_types_action_status(m_types);
  bind_types_e_stop(m_types);
  bind_types_error_level(m_types);
  bind_types_info_level(m_types);
  bind_types_operating_mode(m_types);
  bind_types_blocking_type(m_types);
  bind_types_connection_state(m_types);
  bind_types_orientation_type(m_types);
  bind_types_value_data_type(m_types);
  bind_types_action_scope(m_types);
  bind_types_support(m_types);
  bind_types_agv_kinematic(m_types);
  bind_types_agv_class(m_types);
  bind_types_wheel_definition_type(m_types);

  // Structs
  bind_types_action_state(m_types);
  bind_types_battery_state(m_types);
  bind_types_agv_position(m_types);
  bind_types_velocity(m_types);
  bind_types_safety_state(m_types);
  bind_types_error_reference(m_types);
  bind_types_error(m_types);
  bind_types_info_reference(m_types);
  bind_types_info(m_types);
  bind_types_action_parameter(m_types);
  bind_types_action_parameter_factsheet(m_types);
  bind_types_bounding_box_reference(m_types);
  bind_types_control_point(m_types);
  bind_types_polygon_point(m_types);
  bind_types_position(m_types);
  bind_types_load_dimensions(m_types);
  bind_types_max_string_lens(m_types);
  bind_types_max_array_lens(m_types);
  bind_types_timing(m_types);
  bind_types_physical_parameters(m_types);
  bind_types_header(m_types);
  bind_types_action(m_types);
  bind_types_agv_action(m_types);
  bind_types_optional_parameter(m_types);
  bind_types_trajectory(m_types);
  bind_types_envelope2d(m_types);
  bind_types_envelope3d(m_types);
  bind_types_node_position(m_types);
  bind_types_load(m_types);
  bind_types_load_set(m_types);
  bind_types_load_specification(m_types);
  bind_types_wheel_definition(m_types);
  bind_types_agv_geometry(m_types);
  bind_types_edge_state(m_types);
  bind_types_node_state(m_types);
  bind_types_edge(m_types);
  bind_types_node(m_types);
  bind_types_type_specification(m_types);
  bind_types_protocol_limits(m_types);
  bind_types_protocol_features(m_types);
  bind_types_connection(m_types);
  bind_types_instant_actions(m_types);
  bind_types_order(m_types);
  bind_types_visualization(m_types);
  bind_types_factsheet(m_types);
  bind_types_state(m_types);
}

}  // namespace vda5050_core_py
