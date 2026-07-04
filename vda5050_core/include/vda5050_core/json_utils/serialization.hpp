/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
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

#ifndef VDA5050_CORE__JSON_UTILS__SERIALIZATION_HPP_
#define VDA5050_CORE__JSON_UTILS__SERIALIZATION_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"
#include "vda5050_core/types/action_parameter_factsheet.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/agv_action.hpp"
#include "vda5050_core/types/agv_geometry.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/battery_state.hpp"
#include "vda5050_core/types/bounding_box_reference.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/control_point.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/edge_state.hpp"
#include "vda5050_core/types/envelope2d.hpp"
#include "vda5050_core/types/envelope3d.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/error_reference.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/info.hpp"
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
#include "vda5050_core/types/optional_parameter.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/physical_parameters.hpp"
#include "vda5050_core/types/polygon_point.hpp"
#include "vda5050_core/types/position.hpp"
#include "vda5050_core/types/protocol_features.hpp"
#include "vda5050_core/types/protocol_limits.hpp"
#include "vda5050_core/types/safety_state.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/timing.hpp"
#include "vda5050_core/types/trajectory.hpp"
#include "vda5050_core/types/type_specification.hpp"
#include "vda5050_core/types/velocity.hpp"
#include "vda5050_core/types/visualization.hpp"
#include "vda5050_core/types/wheel_definition.hpp"

#ifdef ENABLE_ROS2
#include <vda5050_interfaces/msg/action.hpp>
#include <vda5050_interfaces/msg/action_parameter.hpp>
#include <vda5050_interfaces/msg/action_parameter_factsheet.hpp>
#include <vda5050_interfaces/msg/action_state.hpp>
#include <vda5050_interfaces/msg/agv_action.hpp>
#include <vda5050_interfaces/msg/agv_geometry.hpp>
#include <vda5050_interfaces/msg/agv_position.hpp>
#include <vda5050_interfaces/msg/battery_state.hpp>
#include <vda5050_interfaces/msg/bounding_box_reference.hpp>
#include <vda5050_interfaces/msg/connection.hpp>
#include <vda5050_interfaces/msg/control_point.hpp>
#include <vda5050_interfaces/msg/edge.hpp>
#include <vda5050_interfaces/msg/edge_state.hpp>
#include <vda5050_interfaces/msg/envelope2d.hpp>
#include <vda5050_interfaces/msg/envelope3d.hpp>
#include <vda5050_interfaces/msg/error.hpp>
#include <vda5050_interfaces/msg/error_reference.hpp>
#include <vda5050_interfaces/msg/factsheet.hpp>
#include <vda5050_interfaces/msg/header.hpp>
#include <vda5050_interfaces/msg/info.hpp>
#include <vda5050_interfaces/msg/info_reference.hpp>
#include <vda5050_interfaces/msg/instant_actions.hpp>
#include <vda5050_interfaces/msg/load.hpp>
#include <vda5050_interfaces/msg/load_dimensions.hpp>
#include <vda5050_interfaces/msg/load_set.hpp>
#include <vda5050_interfaces/msg/load_specification.hpp>
#include <vda5050_interfaces/msg/max_array_lens.hpp>
#include <vda5050_interfaces/msg/max_string_lens.hpp>
#include <vda5050_interfaces/msg/node.hpp>
#include <vda5050_interfaces/msg/node_position.hpp>
#include <vda5050_interfaces/msg/node_state.hpp>
#include <vda5050_interfaces/msg/optional_parameter.hpp>
#include <vda5050_interfaces/msg/order.hpp>
#include <vda5050_interfaces/msg/physical_parameters.hpp>
#include <vda5050_interfaces/msg/polygon_point.hpp>
#include <vda5050_interfaces/msg/position.hpp>
#include <vda5050_interfaces/msg/protocol_features.hpp>
#include <vda5050_interfaces/msg/protocol_limits.hpp>
#include <vda5050_interfaces/msg/safety_state.hpp>
#include <vda5050_interfaces/msg/state.hpp>
#include <vda5050_interfaces/msg/timing.hpp>
#include <vda5050_interfaces/msg/trajectory.hpp>
#include <vda5050_interfaces/msg/type_specification.hpp>
#include <vda5050_interfaces/msg/velocity.hpp>
#include <vda5050_interfaces/msg/visualization.hpp>
#include <vda5050_interfaces/msg/wheel_definition.hpp>
#endif  // ENABLE_ROS2

#include "traits.hpp"

namespace vda5050_core {

namespace types {

namespace action_detail {

//=============================================================================
template <typename ActionT>
void to_json(nlohmann::json& j, const ActionT& msg)
{
  using json_utils::blocking_type_traits;
  using json_utils::optional_field_traits;

  using action_description_trait =
    optional_field_traits<decltype(msg.action_description)>;
  using action_parameters_trait =
    optional_field_traits<decltype(msg.action_parameters)>;

  j["actionType"] = msg.action_type;
  j["actionId"] = msg.action_id;

  j["blockingType"] =
    blocking_type_traits<decltype(msg.blocking_type)>::to_string(
      msg.blocking_type);

  if (action_description_trait::has_value(msg.action_description))
  {
    j["actionDescription"] =
      action_description_trait::get(msg.action_description);
  }

  if (action_parameters_trait::has_value(msg.action_parameters))
  {
    j["actionParameters"] = action_parameters_trait::get(msg.action_parameters);
  }
}

//=============================================================================
template <typename ActionT>
void from_json(const nlohmann::json& j, ActionT& msg)
{
  using json_utils::blocking_type_traits;
  using json_utils::optional_field_traits;

  using action_description_trait =
    optional_field_traits<decltype(msg.action_description)>;
  using action_parameters_trait =
    optional_field_traits<decltype(msg.action_parameters)>;

  msg.action_type = j.at("actionType").get<std::string>();
  msg.action_id = j.at("actionId").get<std::string>();
  msg.blocking_type =
    blocking_type_traits<decltype(msg.blocking_type)>::from_string(
      j.at("blockingType").get<std::string>());

  if (j.contains("actionDescription"))
  {
    action_description_trait::set(
      msg.action_description, j.at("actionDescription").get<std::string>());
  }

  if (j.contains("actionParameters"))
  {
    action_parameters_trait::set(
      msg.action_parameters, j.at("actionParameters"));
  }
}

}  // namespace action_detail

namespace action_parameter_detail {

//=============================================================================
template <typename ActionParameterT>
void to_json(nlohmann::json& j, const ActionParameterT& msg)
{
  j["key"] = msg.key;
  j["value"] = msg.value;
}

//=============================================================================
template <typename ActionParameterT>
void from_json(const nlohmann::json& j, ActionParameterT& msg)
{
  msg.key = j.at("key").get<std::string>();
  msg.value = j.at("value").get<std::string>();
}

}  // namespace action_parameter_detail

namespace action_parameter_factsheet_detail {

//=============================================================================
template <typename ActionParameterFactsheetT>
void to_json(nlohmann::json& j, const ActionParameterFactsheetT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::value_data_type_traits;

  using description_trait = optional_field_traits<decltype(msg.description)>;
  using is_optional_trait = optional_field_traits<decltype(msg.is_optional)>;

  j["key"] = msg.key;

  j["valueDataType"] =
    value_data_type_traits<decltype(msg.value_data_type)>::to_string(
      msg.value_data_type);

  if (description_trait::has_value(msg.description))
  {
    j["description"] = description_trait::get(msg.description);
  }

  if (is_optional_trait::has_value(msg.is_optional))
  {
    j["isOptional"] = is_optional_trait::get(msg.is_optional);
  }
}

//=============================================================================
template <typename ActionParameterFactsheetT>
void from_json(const nlohmann::json& j, ActionParameterFactsheetT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::value_data_type_traits;

  using description_trait = optional_field_traits<decltype(msg.description)>;
  using is_optional_trait = optional_field_traits<decltype(msg.is_optional)>;

  msg.key = j.at("key").get<std::string>();

  msg.value_data_type =
    value_data_type_traits<decltype(msg.value_data_type)>::from_string(
      j.at("valueDataType").get<std::string>());

  if (j.contains("description"))
  {
    description_trait::set(
      msg.description, j.at("description").get<std::string>());
  }

  if (j.contains("isOptional"))
  {
    is_optional_trait::set(msg.is_optional, j.at("isOptional").get<bool>());
  }
}

}  // namespace action_parameter_factsheet_detail

namespace action_state_detail {

//=============================================================================
template <typename ActionStateT>
void to_json(nlohmann::json& j, const ActionStateT& msg)
{
  using json_utils::action_status_traits;
  using json_utils::optional_field_traits;

  using action_type_trait = optional_field_traits<decltype(msg.action_type)>;
  using action_description_trait =
    optional_field_traits<decltype(msg.action_description)>;
  using result_description_trait =
    optional_field_traits<decltype(msg.result_description)>;

  j["actionId"] = msg.action_id;

  j["actionStatus"] =
    action_status_traits<decltype(msg.action_status)>::to_string(
      msg.action_status);

  if (action_type_trait::has_value(msg.action_type))
  {
    j["actionType"] = action_type_trait::get(msg.action_type);
  }

  if (action_description_trait::has_value(msg.action_description))
  {
    j["actionDescription"] =
      action_description_trait::get(msg.action_description);
  }

  if (result_description_trait::has_value(msg.result_description))
  {
    j["resultDescription"] =
      result_description_trait::get(msg.result_description);
  }
}

//=============================================================================
template <typename ActionStateT>
void from_json(const nlohmann::json& j, ActionStateT& msg)
{
  using json_utils::action_status_traits;
  using json_utils::optional_field_traits;

  using action_type_trait = optional_field_traits<decltype(msg.action_type)>;
  using action_description_trait =
    optional_field_traits<decltype(msg.action_description)>;
  using result_description_trait =
    optional_field_traits<decltype(msg.result_description)>;

  msg.action_id = j.at("actionId").get<std::string>();

  msg.action_status =
    action_status_traits<decltype(msg.action_status)>::from_string(
      j.at("actionStatus").get<std::string>());

  if (j.contains("actionType"))
  {
    action_type_trait::set(
      msg.action_type, j.at("actionType").get<std::string>());
  }

  if (j.contains("actionDescription"))
  {
    action_description_trait::set(
      msg.action_description, j.at("actionDescription").get<std::string>());
  }

  if (j.contains("resultDescription"))
  {
    result_description_trait::set(
      msg.result_description, j.at("resultDescription").get<std::string>());
  }
}

}  // namespace action_state_detail

namespace agv_action_detail {

//=============================================================================
template <typename AGVActionT>
void to_json(nlohmann::json& j, const AGVActionT& msg)
{
  using json_utils::action_scopes_traits;
  using json_utils::blocking_types_traits;
  using json_utils::optional_field_traits;

  using action_parameters_trait =
    optional_field_traits<decltype(msg.action_parameters)>;
  using result_description_trait =
    optional_field_traits<decltype(msg.result_description)>;
  using action_description_trait =
    optional_field_traits<decltype(msg.action_description)>;
  using blocking_types_optional_trait =
    optional_field_traits<decltype(msg.blocking_types)>;

  j["actionType"] = msg.action_type;

  j["actionScopes"] =
    action_scopes_traits<decltype(msg.action_scopes)>::to_string(
      msg.action_scopes);

  if (action_parameters_trait::has_value(msg.action_parameters))
  {
    j["actionParameters"] = action_parameters_trait::get(msg.action_parameters);
  }

  if (result_description_trait::has_value(msg.result_description))
  {
    j["resultDescription"] =
      result_description_trait::get(msg.result_description);
  }

  if (action_description_trait::has_value(msg.action_description))
  {
    j["actionDescription"] =
      action_description_trait::get(msg.action_description);
  }

  if (blocking_types_optional_trait::has_value(msg.blocking_types))
  {
    using inner_t = typename blocking_types_optional_trait::value_type;

    inner_t value = blocking_types_optional_trait::get(msg.blocking_types);
    j["blockingTypes"] =
      blocking_types_traits<decltype(value)>::to_string(value);
  }
}

//=============================================================================
template <typename AGVActionT>
void from_json(const nlohmann::json& j, AGVActionT& msg)
{
  using json_utils::action_scopes_traits;
  using json_utils::blocking_types_traits;
  using json_utils::optional_field_traits;

  using action_parameters_trait =
    optional_field_traits<decltype(msg.action_parameters)>;
  using result_description_trait =
    optional_field_traits<decltype(msg.result_description)>;
  using action_description_trait =
    optional_field_traits<decltype(msg.action_description)>;
  using blocking_types_optional_trait =
    optional_field_traits<decltype(msg.blocking_types)>;

  msg.action_type = j.at("actionType").get<std::string>();

  msg.action_scopes =
    action_scopes_traits<decltype(msg.action_scopes)>::from_string(
      j.at("actionScopes"));

  if (j.contains("actionParameters"))
  {
    action_parameters_trait::set(
      msg.action_parameters, j.at("actionParameters"));
  }

  if (j.contains("resultDescription"))
  {
    result_description_trait::set(
      msg.result_description, j.at("resultDescription"));
  }

  if (j.contains("actionDescription"))
  {
    action_description_trait::set(
      msg.action_description, j.at("actionDescription"));
  }

  if (j.contains("blockingTypes"))
  {
    using inner_t = typename blocking_types_optional_trait::value_type;

    inner_t value =
      blocking_types_traits<inner_t>::from_string(j.at("blockingTypes"));
    blocking_types_optional_trait::set(msg.blocking_types, value);
  }
}

}  // namespace agv_action_detail

namespace agv_geometry_detail {

//=============================================================================
template <typename AGVGeometryT>
void to_json(nlohmann::json& j, const AGVGeometryT& msg)
{
  using json_utils::optional_field_traits;

  using wheel_definitions_trait =
    optional_field_traits<decltype(msg.wheel_definitions)>;
  using envelopes2d_trait = optional_field_traits<decltype(msg.envelopes2d)>;
  using envelopes3d_trait = optional_field_traits<decltype(msg.envelopes3d)>;

  if (wheel_definitions_trait::has_value(msg.wheel_definitions))
  {
    j["wheelDefinitions"] = wheel_definitions_trait::get(msg.wheel_definitions);
  }

  if (envelopes2d_trait::has_value(msg.envelopes2d))
  {
    j["envelopes2d"] = envelopes2d_trait::get(msg.envelopes2d);
  }

  if (envelopes3d_trait::has_value(msg.envelopes3d))
  {
    j["envelopes3d"] = envelopes3d_trait::get(msg.envelopes3d);
  }
}

//=============================================================================
template <typename AGVGeometryT>
void from_json(const nlohmann::json& j, AGVGeometryT& msg)
{
  using json_utils::optional_field_traits;

  using wheel_definitions_trait =
    optional_field_traits<decltype(msg.wheel_definitions)>;
  using envelopes2d_trait = optional_field_traits<decltype(msg.envelopes2d)>;
  using envelopes3d_trait = optional_field_traits<decltype(msg.envelopes3d)>;

  if (j.contains("wheelDefinitions"))
  {
    wheel_definitions_trait::set(
      msg.wheel_definitions, j.at("wheelDefinitions"));
  }

  if (j.contains("envelopes2d"))
  {
    envelopes2d_trait::set(msg.envelopes2d, j.at("envelopes2d"));
  }

  if (j.contains("envelopes3d"))
  {
    envelopes3d_trait::set(msg.envelopes3d, j.at("envelopes3d"));
  }
}

}  // namespace agv_geometry_detail

namespace agv_position_detail {

//=============================================================================
template <typename AGVPositionT>
void to_json(nlohmann::json& j, const AGVPositionT& msg)
{
  using json_utils::optional_field_traits;

  using map_description_trait =
    optional_field_traits<decltype(msg.map_description)>;
  using localization_score_trait =
    optional_field_traits<decltype(msg.localization_score)>;
  using deviation_range_trait =
    optional_field_traits<decltype(msg.deviation_range)>;

  j["x"] = msg.x;
  j["y"] = msg.y;
  j["theta"] = msg.theta;
  j["mapId"] = msg.map_id;
  j["positionInitialized"] = msg.position_initialized;

  if (map_description_trait::has_value(msg.map_description))
  {
    j["mapDescription"] = map_description_trait::get(msg.map_description);
  }

  if (localization_score_trait::has_value(msg.localization_score))
  {
    j["localizationScore"] =
      localization_score_trait::get(msg.localization_score);
  }

  if (deviation_range_trait::has_value(msg.deviation_range))
  {
    j["deviationRange"] = deviation_range_trait::get(msg.deviation_range);
  }
}

//=============================================================================
template <typename AGVPositionT>
void from_json(const nlohmann::json& j, AGVPositionT& msg)
{
  using json_utils::optional_field_traits;

  using map_description_trait =
    optional_field_traits<decltype(msg.map_description)>;
  using localization_score_trait =
    optional_field_traits<decltype(msg.localization_score)>;
  using deviation_range_trait =
    optional_field_traits<decltype(msg.deviation_range)>;

  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.theta = j.at("theta").get<double>();
  msg.map_id = j.at("mapId").get<std::string>();
  msg.position_initialized = j.at("positionInitialized").get<bool>();

  if (j.contains("mapDescription"))
  {
    map_description_trait::set(
      msg.map_description, j.at("mapDescription").get<std::string>());
  }

  if (j.contains("localizationScore"))
  {
    localization_score_trait::set(
      msg.localization_score, j.at("localizationScore").get<double>());
  }

  if (j.contains("deviationRange"))
  {
    deviation_range_trait::set(
      msg.deviation_range, j.at("deviationRange").get<double>());
  }
}

}  // namespace agv_position_detail

namespace battery_state_detail {

//=============================================================================
template <typename BatteryStateT>
void to_json(nlohmann::json& j, const BatteryStateT& msg)
{
  using json_utils::optional_field_traits;

  using battery_voltage_trait =
    optional_field_traits<decltype(msg.battery_voltage)>;
  using battery_health_trait =
    optional_field_traits<decltype(msg.battery_health)>;
  using reach_trait = optional_field_traits<decltype(msg.reach)>;

  j["batteryCharge"] = msg.battery_charge;
  j["charging"] = msg.charging;

  if (battery_voltage_trait::has_value(msg.battery_voltage))
  {
    j["batteryVoltage"] = battery_voltage_trait::get(msg.battery_voltage);
  }

  if (battery_health_trait::has_value(msg.battery_health))
  {
    j["batteryHealth"] = battery_health_trait::get(msg.battery_health);
  }

  if (reach_trait::has_value(msg.reach))
  {
    j["reach"] = reach_trait::get(msg.reach);
  }
}

//=============================================================================
template <typename BatteryStateT>
void from_json(const nlohmann::json& j, BatteryStateT& msg)
{
  using json_utils::optional_field_traits;

  using battery_voltage_trait =
    optional_field_traits<decltype(msg.battery_voltage)>;
  using battery_health_trait =
    optional_field_traits<decltype(msg.battery_health)>;
  using reach_trait = optional_field_traits<decltype(msg.reach)>;

  msg.battery_charge = j.at("batteryCharge").get<double>();
  msg.charging = j.at("charging").get<bool>();

  if (j.contains("batteryVoltage"))
  {
    battery_voltage_trait::set(
      msg.battery_voltage, j.at("batteryVoltage").get<double>());
  }

  if (j.contains("batteryHealth"))
  {
    battery_health_trait::set(
      msg.battery_health, j.at("batteryHealth").get<int8_t>());
  }

  if (j.contains("reach"))
  {
    reach_trait::set(msg.reach, j.at("reach").get<uint32_t>());
  }
}

}  // namespace battery_state_detail

namespace bounding_box_reference_detail {

//=============================================================================
template <typename BoundingBoxReferenceT>
void to_json(nlohmann::json& j, const BoundingBoxReferenceT& msg)
{
  using json_utils::optional_field_traits;

  using theta_trait = optional_field_traits<decltype(msg.theta)>;

  j["x"] = msg.x;
  j["y"] = msg.y;
  j["z"] = msg.z;

  if (theta_trait::has_value(msg.theta))
  {
    j["theta"] = theta_trait::get(msg.theta);
  }
}

//=============================================================================
template <typename BoundingBoxReferenceT>
void from_json(const nlohmann::json& j, BoundingBoxReferenceT& msg)
{
  using json_utils::optional_field_traits;

  using theta_trait = optional_field_traits<decltype(msg.theta)>;

  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.z = j.at("z").get<double>();

  if (j.contains("theta"))
  {
    theta_trait::set(msg.theta, j.at("theta").get<double>());
  }
}

}  // namespace bounding_box_reference_detail

namespace connection_detail {

//=============================================================================
template <typename ConnectionT>
void to_json(nlohmann::json& j, const ConnectionT& msg)
{
  using json_utils::connection_state_traits;

  to_json(j, msg.header);

  j["connectionState"] =
    connection_state_traits<decltype(msg.connection_state)>::to_string(
      msg.connection_state);
}

//=============================================================================
template <typename ConnectionT>
void from_json(const nlohmann::json& j, ConnectionT& msg)
{
  using json_utils::connection_state_traits;

  from_json(j, msg.header);

  msg.connection_state =
    connection_state_traits<decltype(msg.connection_state)>::from_string(
      j.at("connectionState").get<std::string>());
}

}  // namespace connection_detail

namespace control_point_detail {

//=============================================================================
template <typename ControlPointT>
void to_json(nlohmann::json& j, const ControlPointT& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  j["weight"] = msg.weight;
}

//=============================================================================
template <typename ControlPointT>
void from_json(const nlohmann::json& j, ControlPointT& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();

  if (j.contains("weight"))
  {
    msg.weight = j.at("weight").get<double>();
  }
}

}  // namespace control_point_detail

namespace edge_detail {

//=============================================================================
template <typename EdgeT>
void to_json(nlohmann::json& j, const EdgeT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::orientation_type_traits;

  using edge_description_trait =
    optional_field_traits<decltype(msg.edge_description)>;
  using max_speed_trait = optional_field_traits<decltype(msg.max_speed)>;
  using max_height_trait = optional_field_traits<decltype(msg.max_height)>;
  using min_height_trait = optional_field_traits<decltype(msg.min_height)>;
  using orientation_trait = optional_field_traits<decltype(msg.orientation)>;
  using orientation_type_trait =
    optional_field_traits<decltype(msg.orientation_type)>;
  using direction_trait = optional_field_traits<decltype(msg.direction)>;
  using rotation_allowed_trait =
    optional_field_traits<decltype(msg.rotation_allowed)>;
  using max_rotation_speed_trait =
    optional_field_traits<decltype(msg.max_rotation_speed)>;
  using trajectory_trait = optional_field_traits<decltype(msg.trajectory)>;
  using length_trait = optional_field_traits<decltype(msg.length)>;

  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["startNodeId"] = msg.start_node_id;
  j["endNodeId"] = msg.end_node_id;
  j["released"] = msg.released;
  j["actions"] = msg.actions;

  if (edge_description_trait::has_value(msg.edge_description))
  {
    j["edgeDescription"] = edge_description_trait::get(msg.edge_description);
  }

  if (max_speed_trait::has_value(msg.max_speed))
  {
    j["maxSpeed"] = max_speed_trait::get(msg.max_speed);
  }

  if (max_height_trait::has_value(msg.max_height))
  {
    j["maxHeight"] = max_height_trait::get(msg.max_height);
  }

  if (min_height_trait::has_value(msg.min_height))
  {
    j["minHeight"] = min_height_trait::get(msg.min_height);
  }

  if (orientation_trait::has_value(msg.orientation))
  {
    j["orientation"] = orientation_trait::get(msg.orientation);
  }

  if (orientation_type_trait::has_value(msg.orientation_type))
  {
    using inner_t = typename orientation_type_trait::value_type;

    inner_t value = orientation_type_trait::get(msg.orientation_type);
    j["orientationType"] =
      orientation_type_traits<decltype(value)>::to_string(value);
  }

  if (direction_trait::has_value(msg.direction))
  {
    j["direction"] = direction_trait::get(msg.direction);
  }

  if (rotation_allowed_trait::has_value(msg.rotation_allowed))
  {
    j["rotationAllowed"] = rotation_allowed_trait::get(msg.rotation_allowed);
  }

  if (max_rotation_speed_trait::has_value(msg.max_rotation_speed))
  {
    j["maxRotationSpeed"] =
      max_rotation_speed_trait::get(msg.max_rotation_speed);
  }

  if (trajectory_trait::has_value(msg.trajectory))
  {
    j["trajectory"] = trajectory_trait::get(msg.trajectory);
  }

  if (length_trait::has_value(msg.length))
  {
    j["length"] = length_trait::get(msg.length);
  }
}

//=============================================================================
template <typename EdgeT>
void from_json(const nlohmann::json& j, EdgeT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::orientation_type_traits;

  using edge_description_trait =
    optional_field_traits<decltype(msg.edge_description)>;
  using max_speed_trait = optional_field_traits<decltype(msg.max_speed)>;
  using max_height_trait = optional_field_traits<decltype(msg.max_height)>;
  using min_height_trait = optional_field_traits<decltype(msg.min_height)>;
  using orientation_trait = optional_field_traits<decltype(msg.orientation)>;
  using orientation_type_trait =
    optional_field_traits<decltype(msg.orientation_type)>;
  using direction_trait = optional_field_traits<decltype(msg.direction)>;
  using rotation_allowed_trait =
    optional_field_traits<decltype(msg.rotation_allowed)>;
  using max_rotation_speed_trait =
    optional_field_traits<decltype(msg.max_rotation_speed)>;
  using trajectory_trait = optional_field_traits<decltype(msg.trajectory)>;
  using length_trait = optional_field_traits<decltype(msg.length)>;

  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.start_node_id = j.at("startNodeId").get<std::string>();
  msg.end_node_id = j.at("endNodeId").get<std::string>();
  msg.released = j.at("released").get<bool>();
  j.at("actions").get_to(msg.actions);

  if (j.contains("edgeDescription"))
  {
    edge_description_trait::set(
      msg.edge_description, j.at("edgeDescription").get<std::string>());
  }

  if (j.contains("maxSpeed"))
  {
    max_speed_trait::set(msg.max_speed, j.at("maxSpeed").get<double>());
  }

  if (j.contains("maxHeight"))
  {
    max_height_trait::set(msg.max_height, j.at("maxHeight").get<double>());
  }

  if (j.contains("minHeight"))
  {
    min_height_trait::set(msg.min_height, j.at("minHeight").get<double>());
  }

  if (j.contains("orientation"))
  {
    orientation_trait::set(msg.orientation, j.at("orientation").get<double>());
  }

  if (j.contains("orientationType"))
  {
    using inner_t = typename orientation_type_trait::value_type;

    inner_t value = orientation_type_traits<inner_t>::from_string(
      j.at("orientationType").get<std::string>());
    orientation_type_trait::set(msg.orientation_type, value);
  }

  if (j.contains("direction"))
  {
    direction_trait::set(msg.direction, j.at("direction").get<std::string>());
  }

  if (j.contains("rotationAllowed"))
  {
    rotation_allowed_trait::set(
      msg.rotation_allowed, j.at("rotationAllowed").get<bool>());
  }

  if (j.contains("maxRotationSpeed"))
  {
    max_rotation_speed_trait::set(
      msg.max_rotation_speed, j.at("maxRotationSpeed").get<double>());
  }

  if (j.contains("trajectory"))
  {
    trajectory_trait::set(msg.trajectory, j.at("trajectory"));
  }

  if (j.contains("length"))
  {
    length_trait::set(msg.length, j.at("length").get<double>());
  }
}

}  // namespace edge_detail

namespace edge_state_detail {

//=============================================================================
template <typename EdgeStateT>
void to_json(nlohmann::json& j, const EdgeStateT& msg)
{
  using json_utils::optional_field_traits;

  using edge_description_trait =
    optional_field_traits<decltype(msg.edge_description)>;
  using trajectory_trait = optional_field_traits<decltype(msg.trajectory)>;

  j["edgeId"] = msg.edge_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;

  if (edge_description_trait::has_value(msg.edge_description))
  {
    j["edgeDescription"] = edge_description_trait::get(msg.edge_description);
  }

  if (trajectory_trait::has_value(msg.trajectory))
  {
    j["trajectory"] = trajectory_trait::get(msg.trajectory);
  }
}

//=============================================================================
template <typename EdgeStateT>
void from_json(const nlohmann::json& j, EdgeStateT& msg)
{
  using json_utils::optional_field_traits;

  using edge_description_trait =
    optional_field_traits<decltype(msg.edge_description)>;
  using trajectory_trait = optional_field_traits<decltype(msg.trajectory)>;

  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.released = j.at("released").get<bool>();

  if (j.contains("edgeDescription"))
  {
    edge_description_trait::set(
      msg.edge_description, j.at("edgeDescription").get<std::string>());
  }

  if (j.contains("trajectory"))
  {
    trajectory_trait::set(msg.trajectory, j.at("trajectory"));
  }
}

}  // namespace edge_state_detail

namespace envelope2d_detail {

//=============================================================================
template <typename Envelope2dT>
void to_json(nlohmann::json& j, const Envelope2dT& msg)
{
  using json_utils::optional_field_traits;

  using description_trait = optional_field_traits<decltype(msg.description)>;

  j["set"] = msg.set;
  j["polygonPoints"] = msg.polygon_points;

  if (description_trait::has_value(msg.description))
  {
    j["description"] = description_trait::get(msg.description);
  }
}

//=============================================================================
template <typename Envelope2dT>
void from_json(const nlohmann::json& j, Envelope2dT& msg)
{
  using json_utils::optional_field_traits;

  using description_trait = optional_field_traits<decltype(msg.description)>;

  msg.set = j.at("set").get<std::string>();
  msg.polygon_points = j.at("polygonPoints");

  if (j.contains("description"))
  {
    description_trait::set(
      msg.description, j.at("description").get<std::string>());
  }
}

}  // namespace envelope2d_detail

namespace envelope3d_detail {

//=============================================================================
template <typename Envelope3dT>
void to_json(nlohmann::json& j, const Envelope3dT& msg)
{
  using json_utils::optional_field_traits;

  using data_trait = optional_field_traits<decltype(msg.data)>;
  using url_trait = optional_field_traits<decltype(msg.url)>;
  using description_trait = optional_field_traits<decltype(msg.description)>;

  j["set"] = msg.set;
  j["format"] = msg.format;

  if (data_trait::has_value(msg.data))
  {
    j["data"] = data_trait::get(msg.data);
  }

  if (url_trait::has_value(msg.url))
  {
    j["url"] = url_trait::get(msg.url);
  }

  if (description_trait::has_value(msg.description))
  {
    j["description"] = description_trait::get(msg.description);
  }
}

//=============================================================================
template <typename Envelope3dT>
void from_json(const nlohmann::json& j, Envelope3dT& msg)
{
  using json_utils::optional_field_traits;

  using data_trait = optional_field_traits<decltype(msg.data)>;
  using url_trait = optional_field_traits<decltype(msg.url)>;
  using description_trait = optional_field_traits<decltype(msg.description)>;

  msg.set = j.at("set").get<std::string>();
  msg.format = j.at("format").get<std::string>();

  if (j.contains("data"))
  {
    data_trait::set(msg.data, j.at("data").get<std::string>());
  }

  if (j.contains("url"))
  {
    url_trait::set(msg.url, j.at("url").get<std::string>());
  }

  if (j.contains("description"))
  {
    description_trait::set(
      msg.description, j.at("description").get<std::string>());
  }
}

}  // namespace envelope3d_detail

namespace error_detail {

//=============================================================================
template <typename ErrorT>
void to_json(nlohmann::json& j, const ErrorT& msg)
{
  using json_utils::error_level_traits;
  using json_utils::optional_field_traits;

  using error_references_trait =
    optional_field_traits<decltype(msg.error_references)>;
  using error_description_trait =
    optional_field_traits<decltype(msg.error_description)>;

  j["errorType"] = msg.error_type;

  j["errorLevel"] =
    error_level_traits<decltype(msg.error_level)>::to_string(msg.error_level);

  if (error_references_trait::has_value(msg.error_references))
  {
    j["errorReferences"] = error_references_trait::get(msg.error_references);
  }

  if (error_description_trait::has_value(msg.error_description))
  {
    j["errorDescription"] = error_description_trait::get(msg.error_description);
  }
}

//=============================================================================
template <typename ErrorT>
void from_json(const nlohmann::json& j, ErrorT& msg)
{
  using json_utils::error_level_traits;
  using json_utils::optional_field_traits;

  using error_references_trait =
    optional_field_traits<decltype(msg.error_references)>;
  using error_description_trait =
    optional_field_traits<decltype(msg.error_description)>;

  msg.error_type = j.at("errorType").get<std::string>();

  msg.error_level = error_level_traits<decltype(msg.error_level)>::from_string(
    j.at("errorLevel").get<std::string>());

  if (j.contains("errorReferences"))
  {
    error_references_trait::set(msg.error_references, j.at("errorReferences"));
  }

  if (j.contains("errorDescription"))
  {
    error_description_trait::set(
      msg.error_description, j.at("errorDescription").get<std::string>());
  }
}

}  // namespace error_detail

namespace error_reference_detail {

//=============================================================================
template <typename ErrorReferenceT>
void to_json(nlohmann::json& j, const ErrorReferenceT& msg)
{
  j["referenceKey"] = msg.reference_key;
  j["referenceValue"] = msg.reference_value;
}

//=============================================================================
template <typename ErrorReferenceT>
void from_json(const nlohmann::json& j, ErrorReferenceT& msg)
{
  msg.reference_key = j.at("referenceKey").get<std::string>();
  msg.reference_value = j.at("referenceValue").get<std::string>();
}

}  // namespace error_reference_detail

namespace factsheet_detail {

//=============================================================================
template <typename FactsheetT>
void to_json(nlohmann::json& j, const FactsheetT& msg)
{
  using json_utils::optional_field_traits;

  using localization_parameter_trait =
    optional_field_traits<decltype(msg.localization_parameters)>;

  to_json(j, msg.header);

  j["typeSpecification"] = msg.type_specification;
  j["physicalParameters"] = msg.physical_parameters;
  j["protocolLimits"] = msg.protocol_limits;
  j["protocolFeatures"] = msg.protocol_features;
  j["agvGeometry"] = msg.agv_geometry;
  j["loadSpecification"] = msg.load_specification;

  if (localization_parameter_trait::has_value(msg.localization_parameters))
  {
    j["localizationParameters"] =
      localization_parameter_trait::get(msg.localization_parameters);
  }
}

//=============================================================================
template <typename FactsheetT>
void from_json(const nlohmann::json& j, FactsheetT& msg)
{
  using json_utils::optional_field_traits;

  using localization_parameter_trait =
    optional_field_traits<decltype(msg.localization_parameters)>;

  from_json(j, msg.header);

  msg.type_specification = j.at("typeSpecification");
  msg.physical_parameters = j.at("physicalParameters");
  msg.protocol_limits = j.at("protocolLimits");
  msg.protocol_features = j.at("protocolFeatures");
  msg.agv_geometry = j.at("agvGeometry");
  msg.load_specification = j.at("loadSpecification");

  if (j.contains("localizationParameters"))
  {
    localization_parameter_trait::set(
      msg.localization_parameters,
      j.at("localizationParameters").get<std::string>());
  }
}

}  // namespace factsheet_detail

namespace header_detail {

//=============================================================================
template <typename HeaderT>
void to_json(nlohmann::json& j, const HeaderT& msg)
{
  using json_utils::timestamp_traits;

  j = nlohmann::json{
    {"headerId", msg.header_id},
    {"timestamp",
     timestamp_traits<decltype(msg.timestamp)>::to_string(msg.timestamp)},
    {"version", msg.version},
    {"manufacturer", msg.manufacturer},
    {"serialNumber", msg.serial_number}};
}

//=============================================================================
template <typename HeaderT>
void from_json(const nlohmann::json& j, HeaderT& msg)
{
  using json_utils::timestamp_traits;

  msg.header_id = j.at("headerId").get<uint32_t>();
  msg.timestamp = timestamp_traits<decltype(msg.timestamp)>::from_string(
    j.at("timestamp").get<std::string>());
  msg.version = j.at("version").get<std::string>();
  msg.manufacturer = j.at("manufacturer").get<std::string>();
  msg.serial_number = j.at("serialNumber").get<std::string>();
}

}  // namespace header_detail

namespace info_detail {

//=============================================================================
template <typename InfoT>
void to_json(nlohmann::json& j, const InfoT& msg)
{
  using json_utils::info_level_traits;
  using json_utils::optional_field_traits;

  using info_references_trait =
    optional_field_traits<decltype(msg.info_references)>;
  using info_description_trait =
    optional_field_traits<decltype(msg.info_description)>;

  j["infoType"] = msg.info_type;

  j["infoLevel"] =
    info_level_traits<decltype(msg.info_level)>::to_string(msg.info_level);

  if (info_references_trait::has_value(msg.info_references))
  {
    j["infoReferences"] = info_references_trait::get(msg.info_references);
  }

  if (info_description_trait::has_value(msg.info_description))
  {
    j["infoDescription"] = info_description_trait::get(msg.info_description);
  }
}

//=============================================================================
template <typename InfoT>
void from_json(const nlohmann::json& j, InfoT& msg)
{
  using json_utils::info_level_traits;
  using json_utils::optional_field_traits;

  using info_references_trait =
    optional_field_traits<decltype(msg.info_references)>;
  using info_description_trait =
    optional_field_traits<decltype(msg.info_description)>;

  msg.info_type = j.at("infoType").get<std::string>();

  msg.info_level = info_level_traits<decltype(msg.info_level)>::from_string(
    j.at("infoLevel").get<std::string>());

  if (j.contains("infoReferences"))
  {
    info_references_trait::set(msg.info_references, j.at("infoReferences"));
  }

  if (j.contains("infoDescription"))
  {
    info_description_trait::set(
      msg.info_description, j.at("infoDescription").get<std::string>());
  }
}

}  // namespace info_detail

namespace info_reference_detail {

//=============================================================================
template <typename InfoReferenceT>
void to_json(nlohmann::json& j, const InfoReferenceT& msg)
{
  j["referenceKey"] = msg.reference_key;
  j["referenceValue"] = msg.reference_value;
}

//=============================================================================
template <typename InfoReferenceT>
void from_json(const nlohmann::json& j, InfoReferenceT& msg)
{
  msg.reference_key = j.at("referenceKey").get<std::string>();
  msg.reference_value = j.at("referenceValue").get<std::string>();
}

}  // namespace info_reference_detail

namespace instant_actions_detail {

//=============================================================================
template <typename InstantActionsT>
void to_json(nlohmann::json& j, const InstantActionsT& msg)
{
  to_json(j, msg.header);

  j["actions"] = msg.actions;
}

//=============================================================================
template <typename InstantActionsT>
void from_json(const nlohmann::json& j, InstantActionsT& msg)
{
  from_json(j, msg.header);

  j.at("actions").get_to(msg.actions);
}

}  // namespace instant_actions_detail

namespace load_detail {

//=============================================================================
template <typename LoadT>
void to_json(nlohmann::json& j, const LoadT& msg)
{
  using json_utils::optional_field_traits;

  using load_id_trait = optional_field_traits<decltype(msg.load_id)>;
  using load_type_trait = optional_field_traits<decltype(msg.load_type)>;
  using load_position_trait =
    optional_field_traits<decltype(msg.load_position)>;
  using bounding_box_reference_trait =
    optional_field_traits<decltype(msg.bounding_box_reference)>;
  using load_dimensions_trait =
    optional_field_traits<decltype(msg.load_dimensions)>;
  using weight_trait = optional_field_traits<decltype(msg.weight)>;

  if (load_id_trait::has_value(msg.load_id))
  {
    j["loadId"] = load_id_trait::get(msg.load_id);
  }

  if (load_type_trait::has_value(msg.load_type))
  {
    j["loadType"] = load_type_trait::get(msg.load_type);
  }

  if (load_position_trait::has_value(msg.load_position))
  {
    j["loadPosition"] = load_position_trait::get(msg.load_position);
  }

  if (bounding_box_reference_trait::has_value(msg.bounding_box_reference))
  {
    j["boundingBoxReference"] =
      bounding_box_reference_trait::get(msg.bounding_box_reference);
  }

  if (load_dimensions_trait::has_value(msg.load_dimensions))
  {
    j["loadDimensions"] = load_dimensions_trait::get(msg.load_dimensions);
  }

  if (weight_trait::has_value(msg.weight))
  {
    j["weight"] = weight_trait::get(msg.weight);
  }
}

//=============================================================================
template <typename LoadT>
void from_json(const nlohmann::json& j, LoadT& msg)
{
  using json_utils::optional_field_traits;

  using load_id_trait = optional_field_traits<decltype(msg.load_id)>;
  using load_type_trait = optional_field_traits<decltype(msg.load_type)>;
  using load_position_trait =
    optional_field_traits<decltype(msg.load_position)>;
  using bounding_box_reference_trait =
    optional_field_traits<decltype(msg.bounding_box_reference)>;
  using load_dimensions_trait =
    optional_field_traits<decltype(msg.load_dimensions)>;
  using weight_trait = optional_field_traits<decltype(msg.weight)>;

  if (j.contains("loadId"))
  {
    load_id_trait::set(msg.load_id, j.at("loadId").get<std::string>());
  }

  if (j.contains("loadType"))
  {
    load_type_trait::set(msg.load_type, j.at("loadType").get<std::string>());
  }

  if (j.contains("loadPosition"))
  {
    load_position_trait::set(
      msg.load_position, j.at("loadPosition").get<std::string>());
  }

  if (j.contains("boundingBoxReference"))
  {
    bounding_box_reference_trait::set(
      msg.bounding_box_reference, j.at("boundingBoxReference"));
  }

  if (j.contains("loadDimensions"))
  {
    load_dimensions_trait::set(msg.load_dimensions, j.at("loadDimensions"));
  }

  if (j.contains("weight"))
  {
    weight_trait::set(msg.weight, j.at("weight").get<double>());
  }
}

}  // namespace load_detail

namespace load_dimensions_detail {

//=============================================================================
template <typename LoadDimensionsT>
void to_json(nlohmann::json& j, const LoadDimensionsT& msg)
{
  using json_utils::optional_field_traits;

  using height_trait = optional_field_traits<decltype(msg.height)>;

  j["length"] = msg.length;
  j["width"] = msg.width;

  if (height_trait::has_value(msg.height))
  {
    j["height"] = height_trait::get(msg.height);
  }
}

//=============================================================================
template <typename LoadDimensionsT>
void from_json(const nlohmann::json& j, LoadDimensionsT& msg)
{
  using json_utils::optional_field_traits;

  using height_trait = optional_field_traits<decltype(msg.height)>;

  msg.length = j.at("length").get<double>();
  msg.width = j.at("width").get<double>();

  if (j.contains("height"))
  {
    height_trait::set(msg.height, j.at("height").get<double>());
  }
}

}  // namespace load_dimensions_detail

namespace load_set_detail {

//=============================================================================
template <typename LoadSetT>
void to_json(nlohmann::json& j, const LoadSetT& msg)
{
  using json_utils::optional_field_traits;

  using load_positions_trait =
    optional_field_traits<decltype(msg.load_positions)>;
  using bounding_box_reference_trait =
    optional_field_traits<decltype(msg.bounding_box_reference)>;
  using load_dimensions_trait =
    optional_field_traits<decltype(msg.load_dimensions)>;
  using max_weight_trait = optional_field_traits<decltype(msg.max_weight)>;
  using min_load_handling_height_trait =
    optional_field_traits<decltype(msg.min_load_handling_height)>;
  using max_load_handling_height_trait =
    optional_field_traits<decltype(msg.max_load_handling_height)>;
  using min_load_handling_depth_trait =
    optional_field_traits<decltype(msg.min_load_handling_depth)>;
  using max_load_handling_depth_trait =
    optional_field_traits<decltype(msg.max_load_handling_depth)>;
  using min_load_handling_tilt_trait =
    optional_field_traits<decltype(msg.min_load_handling_tilt)>;
  using max_load_handling_tilt_trait =
    optional_field_traits<decltype(msg.max_load_handling_tilt)>;
  using agv_speed_limit_trait =
    optional_field_traits<decltype(msg.agv_speed_limit)>;
  using agv_acceleration_limit_trait =
    optional_field_traits<decltype(msg.agv_acceleration_limit)>;
  using agv_deceleration_limit_trait =
    optional_field_traits<decltype(msg.agv_deceleration_limit)>;
  using pick_time_trait = optional_field_traits<decltype(msg.pick_time)>;
  using drop_time_trait = optional_field_traits<decltype(msg.drop_time)>;
  using description_trait = optional_field_traits<decltype(msg.description)>;

  j["setName"] = msg.set_name;
  j["loadType"] = msg.load_type;

  if (load_positions_trait::has_value(msg.load_positions))
  {
    j["loadPositions"] = load_positions_trait::get(msg.load_positions);
  }

  if (bounding_box_reference_trait::has_value(msg.bounding_box_reference))
  {
    j["boundingBoxReference"] =
      bounding_box_reference_trait::get(msg.bounding_box_reference);
  }

  if (load_dimensions_trait::has_value(msg.load_dimensions))
  {
    j["loadDimensions"] = load_dimensions_trait::get(msg.load_dimensions);
  }

  if (max_weight_trait::has_value(msg.max_weight))
  {
    j["maxWeight"] = max_weight_trait::get(msg.max_weight);
  }

  if (min_load_handling_height_trait::has_value(msg.min_load_handling_height))
  {
    j["minLoadHandlingHeight"] =
      min_load_handling_height_trait::get(msg.min_load_handling_height);
  }

  if (max_load_handling_height_trait::has_value(msg.max_load_handling_height))
  {
    j["maxLoadHandlingHeight"] =
      max_load_handling_height_trait::get(msg.max_load_handling_height);
  }

  if (min_load_handling_depth_trait::has_value(msg.min_load_handling_depth))
  {
    j["minLoadHandlingDepth"] =
      min_load_handling_depth_trait::get(msg.min_load_handling_depth);
  }

  if (max_load_handling_depth_trait::has_value(msg.max_load_handling_depth))
  {
    j["maxLoadHandlingDepth"] =
      max_load_handling_depth_trait::get(msg.max_load_handling_depth);
  }

  if (min_load_handling_tilt_trait::has_value(msg.min_load_handling_tilt))
  {
    j["minLoadHandlingTilt"] =
      min_load_handling_tilt_trait::get(msg.min_load_handling_tilt);
  }

  if (max_load_handling_tilt_trait::has_value(msg.max_load_handling_tilt))
  {
    j["maxLoadHandlingTilt"] =
      max_load_handling_tilt_trait::get(msg.max_load_handling_tilt);
  }

  if (agv_speed_limit_trait::has_value(msg.agv_speed_limit))
  {
    j["agvSpeedLimit"] = agv_speed_limit_trait::get(msg.agv_speed_limit);
  }

  if (agv_acceleration_limit_trait::has_value(msg.agv_acceleration_limit))
  {
    j["agvAccelerationLimit"] =
      agv_acceleration_limit_trait::get(msg.agv_acceleration_limit);
  }

  if (agv_deceleration_limit_trait::has_value(msg.agv_deceleration_limit))
  {
    j["agvDecelerationLimit"] =
      agv_deceleration_limit_trait::get(msg.agv_deceleration_limit);
  }

  if (pick_time_trait::has_value(msg.pick_time))
  {
    j["pickTime"] = pick_time_trait::get(msg.pick_time);
  }

  if (drop_time_trait::has_value(msg.drop_time))
  {
    j["dropTime"] = drop_time_trait::get(msg.drop_time);
  }

  if (description_trait::has_value(msg.description))
  {
    j["description"] = description_trait::get(msg.description);
  }
}

//=============================================================================
template <typename LoadSetT>
void from_json(const nlohmann::json& j, LoadSetT& msg)
{
  using json_utils::optional_field_traits;

  using load_positions_trait =
    optional_field_traits<decltype(msg.load_positions)>;
  using bounding_box_reference_trait =
    optional_field_traits<decltype(msg.bounding_box_reference)>;
  using load_dimensions_trait =
    optional_field_traits<decltype(msg.load_dimensions)>;
  using max_weight_trait = optional_field_traits<decltype(msg.max_weight)>;
  using min_load_handling_height_trait =
    optional_field_traits<decltype(msg.min_load_handling_height)>;
  using max_load_handling_height_trait =
    optional_field_traits<decltype(msg.max_load_handling_height)>;
  using min_load_handling_depth_trait =
    optional_field_traits<decltype(msg.min_load_handling_depth)>;
  using max_load_handling_depth_trait =
    optional_field_traits<decltype(msg.max_load_handling_depth)>;
  using min_load_handling_tilt_trait =
    optional_field_traits<decltype(msg.min_load_handling_tilt)>;
  using max_load_handling_tilt_trait =
    optional_field_traits<decltype(msg.max_load_handling_tilt)>;
  using agv_speed_limit_trait =
    optional_field_traits<decltype(msg.agv_speed_limit)>;
  using agv_acceleration_limit_trait =
    optional_field_traits<decltype(msg.agv_acceleration_limit)>;
  using agv_deceleration_limit_trait =
    optional_field_traits<decltype(msg.agv_deceleration_limit)>;
  using pick_time_trait = optional_field_traits<decltype(msg.pick_time)>;
  using drop_time_trait = optional_field_traits<decltype(msg.drop_time)>;
  using description_trait = optional_field_traits<decltype(msg.description)>;

  msg.set_name = j.at("setName").get<std::string>();
  msg.load_type = j.at("loadType").get<std::string>();

  if (j.contains("loadPositions"))
  {
    load_positions_trait::set(
      msg.load_positions,
      j.at("loadPositions").get<std::vector<std::string>>());
  }

  if (j.contains("boundingBoxReference"))
  {
    bounding_box_reference_trait::set(
      msg.bounding_box_reference, j.at("boundingBoxReference"));
  }

  if (j.contains("loadDimensions"))
  {
    load_dimensions_trait::set(msg.load_dimensions, j.at("loadDimensions"));
  }

  if (j.contains("maxWeight"))
  {
    max_weight_trait::set(msg.max_weight, j.at("maxWeight").get<double>());
  }

  if (j.contains("minLoadHandlingHeight"))
  {
    min_load_handling_height_trait::set(
      msg.min_load_handling_height,
      j.at("minLoadHandlingHeight").get<double>());
  }

  if (j.contains("maxLoadHandlingHeight"))
  {
    max_load_handling_height_trait::set(
      msg.max_load_handling_height,
      j.at("maxLoadHandlingHeight").get<double>());
  }

  if (j.contains("minLoadHandlingDepth"))
  {
    min_load_handling_depth_trait::set(
      msg.min_load_handling_depth, j.at("minLoadHandlingDepth").get<double>());
  }

  if (j.contains("maxLoadHandlingDepth"))
  {
    max_load_handling_depth_trait::set(
      msg.max_load_handling_depth, j.at("maxLoadHandlingDepth").get<double>());
  }

  if (j.contains("minLoadHandlingTilt"))
  {
    min_load_handling_tilt_trait::set(
      msg.min_load_handling_tilt, j.at("minLoadHandlingTilt").get<double>());
  }

  if (j.contains("maxLoadHandlingTilt"))
  {
    max_load_handling_tilt_trait::set(
      msg.max_load_handling_tilt, j.at("maxLoadHandlingTilt").get<double>());
  }

  if (j.contains("agvSpeedLimit"))
  {
    agv_speed_limit_trait::set(
      msg.agv_speed_limit, j.at("agvSpeedLimit").get<double>());
  }

  if (j.contains("agvAccelerationLimit"))
  {
    agv_acceleration_limit_trait::set(
      msg.agv_acceleration_limit, j.at("agvAccelerationLimit").get<double>());
  }

  if (j.contains("agvDecelerationLimit"))
  {
    agv_deceleration_limit_trait::set(
      msg.agv_deceleration_limit, j.at("agvDecelerationLimit").get<double>());
  }

  if (j.contains("pickTime"))
  {
    pick_time_trait::set(msg.pick_time, j.at("pickTime").get<double>());
  }

  if (j.contains("dropTime"))
  {
    drop_time_trait::set(msg.drop_time, j.at("dropTime").get<double>());
  }

  if (j.contains("description"))
  {
    description_trait::set(
      msg.description, j.at("description").get<std::string>());
  }
}

}  // namespace load_set_detail

namespace load_specification_detail {

//=============================================================================
template <typename LoadSpecificationT>
void to_json(nlohmann::json& j, const LoadSpecificationT& msg)
{
  using json_utils::optional_field_traits;

  using load_positions_trait =
    optional_field_traits<decltype(msg.load_positions)>;
  using load_sets_trait = optional_field_traits<decltype(msg.load_sets)>;

  if (load_positions_trait::has_value(msg.load_positions))
  {
    j["loadPositions"] = load_positions_trait::get(msg.load_positions);
  }

  if (load_sets_trait::has_value(msg.load_sets))
  {
    j["loadSets"] = load_sets_trait::get(msg.load_sets);
  }
}

//=============================================================================
template <typename LoadSpecificationT>
void from_json(const nlohmann::json& j, LoadSpecificationT& msg)
{
  using json_utils::optional_field_traits;

  using load_positions_trait =
    optional_field_traits<decltype(msg.load_positions)>;
  using load_sets_trait = optional_field_traits<decltype(msg.load_sets)>;

  if (j.contains("loadPositions"))
  {
    load_positions_trait::set(
      msg.load_positions,
      j.at("loadPositions").get<std::vector<std::string>>());
  }

  if (j.contains("loadSets"))
  {
    load_sets_trait::set(msg.load_sets, j.at("loadSets"));
  }
}

}  // namespace load_specification_detail

namespace max_array_lens_detail {

//=============================================================================
template <typename MaxArrayLensT>
void to_json(nlohmann::json& j, const MaxArrayLensT& msg)
{
  using json_utils::optional_field_traits;

  using order_nodes_trait = optional_field_traits<decltype(msg.order_nodes)>;
  using order_edges_trait = optional_field_traits<decltype(msg.order_edges)>;
  using node_actions_trait = optional_field_traits<decltype(msg.node_actions)>;
  using edge_actions_trait = optional_field_traits<decltype(msg.edge_actions)>;
  using actions_actions_parameters_trait =
    optional_field_traits<decltype(msg.actions_actions_parameters)>;
  using instant_actions_traits =
    optional_field_traits<decltype(msg.instant_actions)>;
  using trajectory_knot_vector_trait =
    optional_field_traits<decltype(msg.trajectory_knot_vector)>;
  using trajectory_control_points_trait =
    optional_field_traits<decltype(msg.trajectory_control_points)>;
  using state_node_states_trait =
    optional_field_traits<decltype(msg.state_node_states)>;
  using state_edge_states_trait =
    optional_field_traits<decltype(msg.state_edge_states)>;
  using state_loads_trait = optional_field_traits<decltype(msg.state_loads)>;
  using state_action_states_trait =
    optional_field_traits<decltype(msg.state_action_states)>;
  using state_errors_trait = optional_field_traits<decltype(msg.state_errors)>;
  using state_information_trait =
    optional_field_traits<decltype(msg.state_information)>;
  using error_error_references_trait =
    optional_field_traits<decltype(msg.error_error_references)>;
  using information_info_references_trait =
    optional_field_traits<decltype(msg.information_info_references)>;

  if (order_nodes_trait::has_value(msg.order_nodes))
  {
    j["order.nodes"] = order_nodes_trait::get(msg.order_nodes);
  }

  if (order_edges_trait::has_value(msg.order_edges))
  {
    j["order.edges"] = order_edges_trait::get(msg.order_edges);
  }

  if (node_actions_trait::has_value(msg.node_actions))
  {
    j["node.actions"] = node_actions_trait::get(msg.node_actions);
  }

  if (edge_actions_trait::has_value(msg.edge_actions))
  {
    j["edge.actions"] = edge_actions_trait::get(msg.edge_actions);
  }

  if (actions_actions_parameters_trait::has_value(
        msg.actions_actions_parameters))
  {
    j["actions.actionsParameters"] =
      actions_actions_parameters_trait::get(msg.actions_actions_parameters);
  }

  if (instant_actions_traits::has_value(msg.instant_actions))
  {
    j["instantActions"] = instant_actions_traits::get(msg.instant_actions);
  }

  if (trajectory_knot_vector_trait::has_value(msg.trajectory_knot_vector))
  {
    j["trajectory.knotVector"] =
      trajectory_knot_vector_trait::get(msg.trajectory_knot_vector);
  }

  if (trajectory_control_points_trait::has_value(msg.trajectory_control_points))
  {
    j["trajectory.controlPoints"] =
      trajectory_control_points_trait::get(msg.trajectory_control_points);
  }

  if (state_node_states_trait::has_value(msg.state_node_states))
  {
    j["state.nodeStates"] = state_node_states_trait::get(msg.state_node_states);
  }

  if (state_edge_states_trait::has_value(msg.state_edge_states))
  {
    j["state.edgeStates"] = state_edge_states_trait::get(msg.state_edge_states);
  }

  if (state_loads_trait::has_value(msg.state_loads))
  {
    j["state.loads"] = state_loads_trait::get(msg.state_loads);
  }

  if (state_action_states_trait::has_value(msg.state_action_states))
  {
    j["state.actionStates"] =
      state_action_states_trait::get(msg.state_action_states);
  }

  if (state_errors_trait::has_value(msg.state_errors))
  {
    j["state.errors"] = state_errors_trait::get(msg.state_errors);
  }

  if (state_information_trait::has_value(msg.state_information))
  {
    j["state.information"] =
      state_information_trait::get(msg.state_information);
  }

  if (error_error_references_trait::has_value(msg.error_error_references))
  {
    j["error.errorReferences"] =
      error_error_references_trait::get(msg.error_error_references);
  }

  if (information_info_references_trait::has_value(
        msg.information_info_references))
  {
    j["information.infoReferences"] =
      information_info_references_trait::get(msg.information_info_references);
  }
}

//=============================================================================
template <typename MaxArrayLensT>
void from_json(const nlohmann::json& j, MaxArrayLensT& msg)
{
  using json_utils::optional_field_traits;

  using order_nodes_trait = optional_field_traits<decltype(msg.order_nodes)>;
  using order_edges_trait = optional_field_traits<decltype(msg.order_edges)>;
  using node_actions_trait = optional_field_traits<decltype(msg.node_actions)>;
  using edge_actions_trait = optional_field_traits<decltype(msg.edge_actions)>;
  using actions_actions_parameters_trait =
    optional_field_traits<decltype(msg.actions_actions_parameters)>;
  using instant_actions_traits =
    optional_field_traits<decltype(msg.instant_actions)>;
  using trajectory_knot_vector_trait =
    optional_field_traits<decltype(msg.trajectory_knot_vector)>;
  using trajectory_control_points_trait =
    optional_field_traits<decltype(msg.trajectory_control_points)>;
  using state_node_states_trait =
    optional_field_traits<decltype(msg.state_node_states)>;
  using state_edge_states_trait =
    optional_field_traits<decltype(msg.state_edge_states)>;
  using state_loads_trait = optional_field_traits<decltype(msg.state_loads)>;
  using state_action_states_trait =
    optional_field_traits<decltype(msg.state_action_states)>;
  using state_errors_trait = optional_field_traits<decltype(msg.state_errors)>;
  using state_information_trait =
    optional_field_traits<decltype(msg.state_information)>;
  using error_error_references_trait =
    optional_field_traits<decltype(msg.error_error_references)>;
  using information_info_references_trait =
    optional_field_traits<decltype(msg.information_info_references)>;

  if (j.contains("order.nodes"))
  {
    order_nodes_trait::set(
      msg.order_nodes, j.at("order.nodes").get<uint32_t>());
  }

  if (j.contains("order.edges"))
  {
    order_edges_trait::set(
      msg.order_edges, j.at("order.edges").get<uint32_t>());
  }

  if (j.contains("node.actions"))
  {
    node_actions_trait::set(
      msg.node_actions, j.at("node.actions").get<uint32_t>());
  }

  if (j.contains("edge.actions"))
  {
    edge_actions_trait::set(
      msg.edge_actions, j.at("edge.actions").get<uint32_t>());
  }

  if (j.contains("actions.actionsParameters"))
  {
    actions_actions_parameters_trait::set(
      msg.actions_actions_parameters,
      j.at("actions.actionsParameters").get<uint32_t>());
  }

  if (j.contains("instantActions"))
  {
    instant_actions_traits::set(
      msg.instant_actions, j.at("instantActions").get<uint32_t>());
  }

  if (j.contains("trajectory.knotVector"))
  {
    trajectory_knot_vector_trait::set(
      msg.trajectory_knot_vector,
      j.at("trajectory.knotVector").get<uint32_t>());
  }

  if (j.contains("trajectory.controlPoints"))
  {
    trajectory_control_points_trait::set(
      msg.trajectory_control_points,
      j.at("trajectory.controlPoints").get<uint32_t>());
  }

  if (j.contains("state.nodeStates"))
  {
    state_node_states_trait::set(
      msg.state_node_states, j.at("state.nodeStates").get<uint32_t>());
  }

  if (j.contains("state.edgeStates"))
  {
    state_edge_states_trait::set(
      msg.state_edge_states, j.at("state.edgeStates").get<uint32_t>());
  }

  if (j.contains("state.loads"))
  {
    state_loads_trait::set(
      msg.state_loads, j.at("state.loads").get<uint32_t>());
  }

  if (j.contains("state.actionStates"))
  {
    state_action_states_trait::set(
      msg.state_action_states, j.at("state.actionStates").get<uint32_t>());
  }

  if (j.contains("state.errors"))
  {
    state_errors_trait::set(
      msg.state_errors, j.at("state.errors").get<uint32_t>());
  }

  if (j.contains("state.information"))
  {
    state_information_trait::set(
      msg.state_information, j.at("state.information").get<uint32_t>());
  }

  if (j.contains("error.errorReferences"))
  {
    error_error_references_trait::set(
      msg.error_error_references,
      j.at("error.errorReferences").get<uint32_t>());
  }

  if (j.contains("information.infoReferences"))
  {
    information_info_references_trait::set(
      msg.information_info_references,
      j.at("information.infoReferences").get<uint32_t>());
  }
}

}  // namespace max_array_lens_detail

namespace max_string_lens_detail {

//=============================================================================
template <typename MaxStringLensT>
void to_json(nlohmann::json& j, const MaxStringLensT& msg)
{
  using json_utils::optional_field_traits;

  using msg_len_trait = optional_field_traits<decltype(msg.msg_len)>;
  using topic_serial_len_trait =
    optional_field_traits<decltype(msg.topic_serial_len)>;
  using topic_elem_len_trait =
    optional_field_traits<decltype(msg.topic_elem_len)>;
  using id_len_trait = optional_field_traits<decltype(msg.id_len)>;
  using enum_len_trait = optional_field_traits<decltype(msg.enum_len)>;
  using load_id_len_trait = optional_field_traits<decltype(msg.load_id_len)>;
  using id_numerical_only_trait =
    optional_field_traits<decltype(msg.id_numerical_only)>;

  if (msg_len_trait::has_value(msg.msg_len))
  {
    j["msgLen"] = msg_len_trait::get(msg.msg_len);
  }

  if (topic_serial_len_trait::has_value(msg.topic_serial_len))
  {
    j["topicSerialLen"] = topic_serial_len_trait::get(msg.topic_serial_len);
  }

  if (topic_elem_len_trait::has_value(msg.topic_elem_len))
  {
    j["topicElemLen"] = topic_elem_len_trait::get(msg.topic_elem_len);
  }

  if (id_len_trait::has_value(msg.id_len))
  {
    j["idLen"] = id_len_trait::get(msg.id_len);
  }

  if (enum_len_trait::has_value(msg.enum_len))
  {
    j["enumLen"] = enum_len_trait::get(msg.enum_len);
  }

  if (load_id_len_trait::has_value(msg.load_id_len))
  {
    j["loadIdLen"] = load_id_len_trait::get(msg.load_id_len);
  }

  if (id_numerical_only_trait::has_value(msg.id_numerical_only))
  {
    j["idNumericalOnly"] = id_numerical_only_trait::get(msg.id_numerical_only);
  }
}

//=============================================================================
template <typename MaxStringLensT>
void from_json(const nlohmann::json& j, MaxStringLensT& msg)
{
  using json_utils::optional_field_traits;

  using msg_len_trait = optional_field_traits<decltype(msg.msg_len)>;
  using topic_serial_len_trait =
    optional_field_traits<decltype(msg.topic_serial_len)>;
  using topic_elem_len_trait =
    optional_field_traits<decltype(msg.topic_elem_len)>;
  using id_len_trait = optional_field_traits<decltype(msg.id_len)>;
  using enum_len_trait = optional_field_traits<decltype(msg.enum_len)>;
  using load_id_len_trait = optional_field_traits<decltype(msg.load_id_len)>;
  using id_numerical_only_trait =
    optional_field_traits<decltype(msg.id_numerical_only)>;

  if (j.contains("msgLen"))
  {
    msg_len_trait::set(msg.msg_len, j.at("msgLen").get<uint32_t>());
  }

  if (j.contains("topicSerialLen"))
  {
    topic_serial_len_trait::set(
      msg.topic_serial_len, j.at("topicSerialLen").get<uint32_t>());
  }

  if (j.contains("topicElemLen"))
  {
    topic_elem_len_trait::set(
      msg.topic_elem_len, j.at("topicElemLen").get<uint32_t>());
  }

  if (j.contains("idLen"))
  {
    id_len_trait::set(msg.id_len, j.at("idLen").get<uint32_t>());
  }

  if (j.contains("enumLen"))
  {
    enum_len_trait::set(msg.enum_len, j.at("enumLen").get<uint32_t>());
  }

  if (j.contains("loadIdLen"))
  {
    load_id_len_trait::set(msg.load_id_len, j.at("loadIdLen").get<uint32_t>());
  }

  if (j.contains("idNumericalOnly"))
  {
    id_numerical_only_trait::set(
      msg.id_numerical_only, j.at("idNumericalOnly").get<bool>());
  }
}

}  // namespace max_string_lens_detail

namespace node_detail {

//=============================================================================
template <typename NodeT>
void to_json(nlohmann::json& j, const NodeT& msg)
{
  using json_utils::optional_field_traits;

  using node_position_trait =
    optional_field_traits<decltype(msg.node_position)>;
  using node_description_trait =
    optional_field_traits<decltype(msg.node_description)>;

  j["nodeId"] = msg.node_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;
  j["actions"] = msg.actions;

  if (node_position_trait::has_value(msg.node_position))
  {
    j["nodePosition"] = node_position_trait::get(msg.node_position);
  }

  if (node_description_trait::has_value(msg.node_description))
  {
    j["nodeDescription"] = node_description_trait::get(msg.node_description);
  }
}

//=============================================================================
template <typename NodeT>
void from_json(const nlohmann::json& j, NodeT& msg)
{
  using json_utils::optional_field_traits;

  using node_position_trait =
    optional_field_traits<decltype(msg.node_position)>;
  using node_description_trait =
    optional_field_traits<decltype(msg.node_description)>;

  msg.node_id = j.at("nodeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.released = j.at("released").get<bool>();
  j.at("actions").get_to(msg.actions);

  if (j.contains("nodePosition"))
  {
    node_position_trait::set(msg.node_position, j.at("nodePosition"));
  }

  if (j.contains("nodeDescription"))
  {
    node_description_trait::set(
      msg.node_description, j.at("nodeDescription").get<std::string>());
  }
}

}  // namespace node_detail

namespace node_position_detail {

//=============================================================================
template <typename NodePositionT>
void to_json(nlohmann::json& j, const NodePositionT& msg)
{
  using json_utils::optional_field_traits;

  using theta_trait = optional_field_traits<decltype(msg.theta)>;
  using allowed_deviation_x_y_trait =
    optional_field_traits<decltype(msg.allowed_deviation_x_y)>;
  using allowed_deviation_theta_trait =
    optional_field_traits<decltype(msg.allowed_deviation_theta)>;
  using map_description_trait =
    optional_field_traits<decltype(msg.map_description)>;

  j["x"] = msg.x;
  j["y"] = msg.y;
  j["mapId"] = msg.map_id;

  if (theta_trait::has_value(msg.theta))
  {
    j["theta"] = theta_trait::get(msg.theta);
  }

  if (allowed_deviation_x_y_trait::has_value(msg.allowed_deviation_x_y))
  {
    j["allowedDeviationXY"] =
      allowed_deviation_x_y_trait::get(msg.allowed_deviation_x_y);
  }

  if (allowed_deviation_theta_trait::has_value(msg.allowed_deviation_theta))
  {
    j["allowedDeviationTheta"] =
      allowed_deviation_theta_trait::get(msg.allowed_deviation_theta);
  }

  if (map_description_trait::has_value(msg.map_description))
  {
    j["mapDescription"] = map_description_trait::get(msg.map_description);
  }
}

//=============================================================================
template <typename NodePositionT>
void from_json(const nlohmann::json& j, NodePositionT& msg)
{
  using json_utils::optional_field_traits;

  using theta_trait = optional_field_traits<decltype(msg.theta)>;
  using allowed_deviation_x_y_trait =
    optional_field_traits<decltype(msg.allowed_deviation_x_y)>;
  using allowed_deviation_theta_trait =
    optional_field_traits<decltype(msg.allowed_deviation_theta)>;
  using map_description_trait =
    optional_field_traits<decltype(msg.map_description)>;

  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  msg.map_id = j.at("mapId").get<std::string>();

  if (j.contains("theta"))
  {
    theta_trait::set(msg.theta, j.at("theta").get<double>());
  }

  if (j.contains("allowedDeviationXY"))
  {
    allowed_deviation_x_y_trait::set(
      msg.allowed_deviation_x_y, j.at("allowedDeviationXY").get<double>());
  }

  if (j.contains("allowedDeviationTheta"))
  {
    allowed_deviation_theta_trait::set(
      msg.allowed_deviation_theta, j.at("allowedDeviationTheta").get<double>());
  }

  if (j.contains("mapDescription"))
  {
    map_description_trait::set(
      msg.map_description, j.at("mapDescription").get<std::string>());
  }
}

}  // namespace node_position_detail

namespace node_state_detail {

//=============================================================================
template <typename NodeStateT>
void to_json(nlohmann::json& j, const NodeStateT& msg)
{
  using json_utils::optional_field_traits;

  using node_description_trait =
    optional_field_traits<decltype(msg.node_description)>;
  using node_position_trait =
    optional_field_traits<decltype(msg.node_position)>;

  j["nodeId"] = msg.node_id;
  j["sequenceId"] = msg.sequence_id;
  j["released"] = msg.released;

  if (node_description_trait::has_value(msg.node_description))
  {
    j["nodeDescription"] = node_description_trait::get(msg.node_description);
  }

  if (node_position_trait::has_value(msg.node_position))
  {
    j["nodePosition"] = node_position_trait::get(msg.node_position);
  }
}

//=============================================================================
template <typename NodeStateT>
void from_json(const nlohmann::json& j, NodeStateT& msg)
{
  using json_utils::optional_field_traits;

  using node_description_trait =
    optional_field_traits<decltype(msg.node_description)>;
  using node_position_trait =
    optional_field_traits<decltype(msg.node_position)>;

  msg.node_id = j.at("nodeId").get<std::string>();
  msg.sequence_id = j.at("sequenceId").get<uint32_t>();
  msg.released = j.at("released").get<bool>();

  if (j.contains("nodeDescription"))
  {
    node_description_trait::set(
      msg.node_description, j.at("nodeDescription").get<std::string>());
  }

  if (j.contains("nodePosition"))
  {
    node_position_trait::set(msg.node_position, j.at("nodePosition"));
  }
}

}  // namespace node_state_detail

namespace optional_parameter_detail {

//=============================================================================
template <typename OptionalParameterT>
void to_json(nlohmann::json& j, const OptionalParameterT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::support_traits;

  using description_trait = optional_field_traits<decltype(msg.description)>;

  j["parameter"] = msg.parameter;
  j["support"] = support_traits<decltype(msg.support)>::to_string(msg.support);

  if (description_trait::has_value(msg.description))
  {
    j["description"] = description_trait::get(msg.description);
  }
}

//=============================================================================
template <typename OptionalParameterT>
void from_json(const nlohmann::json& j, OptionalParameterT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::support_traits;

  using description_trait = optional_field_traits<decltype(msg.description)>;

  msg.parameter = j.at("parameter").get<std::string>();
  msg.support =
    support_traits<decltype(msg.support)>::from_string(j.at("support"));

  if (j.contains("description"))
  {
    description_trait::set(
      msg.description, j.at("description").get<std::string>());
  }
}

}  // namespace optional_parameter_detail

namespace order_detail {

//=============================================================================
template <typename OrderT>
void to_json(nlohmann::json& j, const OrderT& msg)
{
  using json_utils::optional_field_traits;

  using zone_set_id_trait = optional_field_traits<decltype(msg.zone_set_id)>;

  to_json(j, msg.header);

  j["orderId"] = msg.order_id;
  j["orderUpdateId"] = msg.order_update_id;
  j["nodes"] = msg.nodes;
  j["edges"] = msg.edges;

  if (zone_set_id_trait::has_value(msg.zone_set_id))
  {
    j["zoneSetId"] = zone_set_id_trait::get(msg.zone_set_id);
  }
}

//=============================================================================
template <typename OrderT>
void from_json(const nlohmann::json& j, OrderT& msg)
{
  using json_utils::optional_field_traits;

  using zone_set_id_trait = optional_field_traits<decltype(msg.zone_set_id)>;

  from_json(j, msg.header);

  msg.order_id = j.at("orderId").get<std::string>();
  msg.order_update_id = j.at("orderUpdateId").get<uint32_t>();
  j.at("nodes").get_to(msg.nodes);
  j.at("edges").get_to(msg.edges);

  if (j.contains("zoneSetId"))
  {
    zone_set_id_trait::set(
      msg.zone_set_id, j.at("zoneSetId").get<std::string>());
  }
}

}  // namespace order_detail

namespace physical_parameters_detail {

//=============================================================================
template <typename PhysicalParametersT>
void to_json(nlohmann::json& j, const PhysicalParametersT& msg)
{
  using json_utils::optional_field_traits;

  using angular_speed_min_trait =
    optional_field_traits<decltype(msg.angular_speed_min)>;
  using angular_speed_max_trait =
    optional_field_traits<decltype(msg.angular_speed_max)>;

  j["speedMin"] = msg.speed_min;
  j["speedMax"] = msg.speed_max;
  j["accelerationMax"] = msg.acceleration_max;
  j["decelerationMax"] = msg.deceleration_max;
  j["heightMin"] = msg.height_min;
  j["heightMax"] = msg.height_max;
  j["width"] = msg.width;
  j["length"] = msg.length;

  if (angular_speed_min_trait::has_value(msg.angular_speed_min))
  {
    j["angularSpeedMin"] = angular_speed_min_trait::get(msg.angular_speed_min);
  }

  if (angular_speed_max_trait::has_value(msg.angular_speed_max))
  {
    j["angularSpeedMax"] = angular_speed_max_trait::get(msg.angular_speed_max);
  }
}

//=============================================================================
template <typename PhysicalParametersT>
void from_json(const nlohmann::json& j, PhysicalParametersT& msg)
{
  using json_utils::optional_field_traits;

  using angular_speed_min_trait =
    optional_field_traits<decltype(msg.angular_speed_min)>;
  using angular_speed_max_trait =
    optional_field_traits<decltype(msg.angular_speed_max)>;

  msg.speed_min = j.at("speedMin").get<double>();
  msg.speed_max = j.at("speedMax").get<double>();
  msg.acceleration_max = j.at("accelerationMax").get<double>();
  msg.deceleration_max = j.at("decelerationMax").get<double>();
  msg.height_min = j.at("heightMin").get<double>();
  msg.height_max = j.at("heightMax").get<double>();
  msg.width = j.at("width").get<double>();
  msg.length = j.at("length").get<double>();

  if (j.contains("angularSpeedMin"))
  {
    angular_speed_min_trait::set(
      msg.angular_speed_min, j.at("angularSpeedMin").get<double>());
  }

  if (j.contains("angularSpeedMax"))
  {
    angular_speed_max_trait::set(
      msg.angular_speed_max, j.at("angularSpeedMax").get<double>());
  }
}

}  // namespace physical_parameters_detail

namespace polygon_point_detail {

//=============================================================================
template <typename PolygonPointT>
void to_json(nlohmann::json& j, const PolygonPointT& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
}

//=============================================================================
template <typename PolygonPointT>
void from_json(const nlohmann::json& j, PolygonPointT& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
}

}  // namespace polygon_point_detail

namespace position_detail {

//=============================================================================
template <typename PositionT>
void to_json(nlohmann::json& j, const PositionT& msg)
{
  using json_utils::optional_field_traits;

  using theta_trait = optional_field_traits<decltype(msg.theta)>;

  j["x"] = msg.x;
  j["y"] = msg.y;

  if (theta_trait::has_value(msg.theta))
  {
    j["theta"] = theta_trait::get(msg.theta);
  }
}

//=============================================================================
template <typename PositionT>
void from_json(const nlohmann::json& j, PositionT& msg)
{
  using json_utils::optional_field_traits;

  using theta_trait = optional_field_traits<decltype(msg.theta)>;

  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();

  if (j.contains("theta"))
  {
    theta_trait::set(msg.theta, j.at("theta").get<double>());
  }
}

}  // namespace position_detail

namespace protocol_features_detail {

//=============================================================================
template <typename ProtocolFeaturesT>
void to_json(nlohmann::json& j, const ProtocolFeaturesT& msg)
{
  j["optionalParameters"] = msg.optional_parameters;
  j["agvActions"] = msg.agv_actions;
}

//=============================================================================
template <typename ProtocolFeaturesT>
void from_json(const nlohmann::json& j, ProtocolFeaturesT& msg)
{
  j.at("optionalParameters").get_to(msg.optional_parameters);
  j.at("agvActions").get_to(msg.agv_actions);
}

}  // namespace protocol_features_detail

namespace protocol_limits_detail {

//=============================================================================
template <typename ProtocolLimitsT>
void to_json(nlohmann::json& j, const ProtocolLimitsT& msg)
{
  j["maxStringLens"] = msg.max_string_lens;
  j["maxArrayLens"] = msg.max_array_lens;
  j["timing"] = msg.timing;
}

//=============================================================================
template <typename ProtocolLimitsT>
void from_json(const nlohmann::json& j, ProtocolLimitsT& msg)
{
  msg.max_string_lens = j.at("maxStringLens");
  msg.max_array_lens = j.at("maxArrayLens");
  msg.timing = j.at("timing");
}

}  // namespace protocol_limits_detail

namespace safety_state_detail {

//=============================================================================
template <typename SafetyStateT>
void to_json(nlohmann::json& j, const SafetyStateT& msg)
{
  using json_utils::e_stop_traits;

  j["eStop"] = e_stop_traits<decltype(msg.e_stop)>::to_string(msg.e_stop);

  j["fieldViolation"] = msg.field_violation;
}

//=============================================================================
template <typename SafetyStateT>
void from_json(const nlohmann::json& j, SafetyStateT& msg)
{
  using json_utils::e_stop_traits;

  msg.e_stop = e_stop_traits<decltype(msg.e_stop)>::from_string(
    j.at("eStop").get<std::string>());

  msg.field_violation = j.at("fieldViolation").get<bool>();
}

}  // namespace safety_state_detail

namespace state_detail {

//=============================================================================
template <typename StateT>
void to_json(nlohmann::json& j, const StateT& msg)
{
  using json_utils::operating_mode_traits;
  using json_utils::optional_field_traits;

  using zone_set_id_trait = optional_field_traits<decltype(msg.zone_set_id)>;
  using paused_trait = optional_field_traits<decltype(msg.paused)>;
  using new_base_request_trait =
    optional_field_traits<decltype(msg.new_base_request)>;
  using distance_since_last_node_trait =
    optional_field_traits<decltype(msg.distance_since_last_node)>;
  using agv_position_trait = optional_field_traits<decltype(msg.agv_position)>;
  using velocity_trait = optional_field_traits<decltype(msg.velocity)>;
  using loads_trait = optional_field_traits<decltype(msg.loads)>;
  using information_traits = optional_field_traits<decltype(msg.information)>;

  to_json(j, msg.header);

  j["orderId"] = msg.order_id;
  j["orderUpdateId"] = msg.order_update_id;
  j["lastNodeId"] = msg.last_node_id;
  j["lastNodeSequenceId"] = msg.last_node_sequence_id;
  j["driving"] = msg.driving;

  j["operatingMode"] =
    operating_mode_traits<decltype(msg.operating_mode)>::to_string(
      msg.operating_mode);

  j["nodeStates"] = msg.node_states;
  j["edgeStates"] = msg.edge_states;
  j["actionStates"] = msg.action_states;
  j["errors"] = msg.errors;
  j["batteryState"] = msg.battery_state;
  j["safetyState"] = msg.safety_state;

  if (zone_set_id_trait::has_value(msg.zone_set_id))
  {
    j["zoneSetId"] = zone_set_id_trait::get(msg.zone_set_id);
  }

  if (paused_trait::has_value(msg.paused))
  {
    j["paused"] = paused_trait::get(msg.paused);
  }

  if (new_base_request_trait::has_value(msg.new_base_request))
  {
    j["newBaseRequest"] = new_base_request_trait::get(msg.new_base_request);
  }

  if (distance_since_last_node_trait::has_value(msg.distance_since_last_node))
  {
    j["distanceSinceLastNode"] =
      distance_since_last_node_trait::get(msg.distance_since_last_node);
  }

  if (agv_position_trait::has_value(msg.agv_position))
  {
    j["agvPosition"] = agv_position_trait::get(msg.agv_position);
  }

  if (velocity_trait::has_value(msg.velocity))
  {
    j["velocity"] = velocity_trait::get(msg.velocity);
  }

  if (loads_trait::has_value(msg.loads))
  {
    j["loads"] = loads_trait::get(msg.loads);
  }

  if (information_traits::has_value(msg.information))
  {
    j["information"] = information_traits::get(msg.information);
  }
}

//=============================================================================
template <typename StateT>
void from_json(const nlohmann::json& j, StateT& msg)
{
  using json_utils::operating_mode_traits;
  using json_utils::optional_field_traits;

  using zone_set_id_trait = optional_field_traits<decltype(msg.zone_set_id)>;
  using paused_trait = optional_field_traits<decltype(msg.paused)>;
  using new_base_request_trait =
    optional_field_traits<decltype(msg.new_base_request)>;
  using distance_since_last_node_trait =
    optional_field_traits<decltype(msg.distance_since_last_node)>;
  using agv_position_trait = optional_field_traits<decltype(msg.agv_position)>;
  using velocity_trait = optional_field_traits<decltype(msg.velocity)>;
  using loads_trait = optional_field_traits<decltype(msg.loads)>;
  using information_traits = optional_field_traits<decltype(msg.information)>;

  from_json(j, msg.header);

  msg.order_id = j.at("orderId").get<std::string>();
  msg.order_update_id = j.at("orderUpdateId").get<uint32_t>();
  msg.last_node_id = j.at("lastNodeId").get<std::string>();
  msg.last_node_sequence_id = j.at("lastNodeSequenceId").get<uint32_t>();
  msg.driving = j.at("driving").get<bool>();

  msg.operating_mode =
    operating_mode_traits<decltype(msg.operating_mode)>::from_string(
      j.at("operatingMode").get<std::string>());

  j.at("nodeStates").get_to(msg.node_states);
  j.at("edgeStates").get_to(msg.edge_states);
  j.at("actionStates").get_to(msg.action_states);
  j.at("errors").get_to(msg.errors);
  msg.battery_state = j.at("batteryState");
  msg.safety_state = j.at("safetyState");

  if (j.contains("zoneSetId"))
  {
    zone_set_id_trait::set(
      msg.zone_set_id, j.at("zoneSetId").get<std::string>());
  }

  if (j.contains("paused"))
  {
    paused_trait::set(msg.paused, j.at("paused").get<bool>());
  }

  if (j.contains("newBaseRequest"))
  {
    new_base_request_trait::set(
      msg.new_base_request, j.at("newBaseRequest").get<bool>());
  }

  if (j.contains("distanceSinceLastNode"))
  {
    distance_since_last_node_trait::set(
      msg.distance_since_last_node,
      j.at("distanceSinceLastNode").get<double>());
  }

  if (j.contains("agvPosition"))
  {
    agv_position_trait::set(msg.agv_position, j.at("agvPosition"));
  }

  if (j.contains("velocity"))
  {
    velocity_trait::set(msg.velocity, j.at("velocity"));
  }

  if (j.contains("loads"))
  {
    loads_trait::set(msg.loads, j.at("loads"));
  }

  if (j.contains("information"))
  {
    information_traits::set(msg.information, j.at("information"));
  }
}

}  // namespace state_detail

namespace timing_detail {

//=============================================================================
template <typename TimingT>
void to_json(nlohmann::json& j, const TimingT& msg)
{
  using json_utils::optional_field_traits;

  using default_state_interval_trait =
    optional_field_traits<decltype(msg.default_state_interval)>;
  using visualization_interval_trait =
    optional_field_traits<decltype(msg.visualization_interval)>;

  j["minOrderInterval"] = msg.min_order_interval;
  j["minStateInterval"] = msg.min_state_interval;

  if (default_state_interval_trait::has_value(msg.default_state_interval))
  {
    j["defaultStateInterval"] =
      default_state_interval_trait::get(msg.default_state_interval);
  }

  if (visualization_interval_trait::has_value(msg.visualization_interval))
  {
    j["visualizationInterval"] =
      visualization_interval_trait::get(msg.visualization_interval);
  }
}

//=============================================================================
template <typename TimingT>
void from_json(const nlohmann::json& j, TimingT& msg)
{
  using json_utils::optional_field_traits;

  using default_state_interval_trait =
    optional_field_traits<decltype(msg.default_state_interval)>;
  using visualization_interval_trait =
    optional_field_traits<decltype(msg.visualization_interval)>;

  msg.min_order_interval = j.at("minOrderInterval").get<double>();
  msg.min_state_interval = j.at("minStateInterval").get<double>();

  if (j.contains("defaultStateInterval"))
  {
    default_state_interval_trait::set(
      msg.default_state_interval, j.at("defaultStateInterval").get<double>());
  }

  if (j.contains("visualizationInterval"))
  {
    visualization_interval_trait::set(
      msg.visualization_interval, j.at("visualizationInterval").get<double>());
  }
}

}  // namespace timing_detail

namespace trajectory_detail {

//=============================================================================
template <typename TrajectoryT>
void to_json(nlohmann::json& j, const TrajectoryT& msg)
{
  j["knotVector"] = msg.knot_vector;
  j["controlPoints"] = msg.control_points;
  j["degree"] = msg.degree;
}

//=============================================================================
template <typename TrajectoryT>
void from_json(const nlohmann::json& j, TrajectoryT& msg)
{
  msg.knot_vector = j.at("knotVector").get<std::vector<double>>();
  msg.control_points = j.at("controlPoints");
  msg.degree = j.at("degree").get<double>();
}

}  // namespace trajectory_detail

namespace type_specification_detail {

//=============================================================================
template <typename TypeSpecificationT>
void to_json(nlohmann::json& j, const TypeSpecificationT& msg)
{
  using json_utils::agv_class_traits;
  using json_utils::agv_kinematic_traits;
  using json_utils::optional_field_traits;

  using series_description_trait =
    optional_field_traits<decltype(msg.series_description)>;

  j["seriesName"] = msg.series_name;

  j["agvKinematic"] =
    agv_kinematic_traits<decltype(msg.agv_kinematic)>::to_string(
      msg.agv_kinematic);

  j["agvClass"] =
    agv_class_traits<decltype(msg.agv_class)>::to_string(msg.agv_class);

  j["maxLoadMass"] = msg.max_load_mass;
  j["localizationTypes"] = msg.localization_types;
  j["navigationTypes"] = msg.navigation_types;

  if (series_description_trait::has_value(msg.series_description))
  {
    j["seriesDescription"] =
      series_description_trait::get(msg.series_description);
  }
}

//=============================================================================
template <typename TypeSpecificationT>
void from_json(const nlohmann::json& j, TypeSpecificationT& msg)
{
  using json_utils::agv_class_traits;
  using json_utils::agv_kinematic_traits;
  using json_utils::optional_field_traits;

  using series_description_trait =
    optional_field_traits<decltype(msg.series_description)>;

  msg.series_name = j.at("seriesName").get<std::string>();

  msg.agv_kinematic =
    agv_kinematic_traits<decltype(msg.agv_kinematic)>::from_string(
      j.at("agvKinematic").get<std::string>());

  msg.agv_class = agv_class_traits<decltype(msg.agv_class)>::from_string(
    j.at("agvClass").get<std::string>());

  msg.max_load_mass = j.at("maxLoadMass").get<double>();
  msg.localization_types =
    j.at("localizationTypes").get<std::vector<std::string>>();
  msg.navigation_types =
    j.at("navigationTypes").get<std::vector<std::string>>();

  if (j.contains("seriesDescription"))
  {
    series_description_trait::set(
      msg.series_description, j.at("seriesDescription").get<std::string>());
  }
}

}  // namespace type_specification_detail

namespace velocity_detail {

//=============================================================================
template <typename VelocityT>
void to_json(nlohmann::json& j, const VelocityT& msg)
{
  using json_utils::optional_field_traits;

  using vx_trait = optional_field_traits<decltype(msg.vx)>;
  using vy_trait = optional_field_traits<decltype(msg.vy)>;
  using omega_trait = optional_field_traits<decltype(msg.omega)>;

  if (vx_trait::has_value(msg.vx))
  {
    j["vx"] = vx_trait::get(msg.vx);
  }

  if (vy_trait::has_value(msg.vy))
  {
    j["vy"] = vy_trait::get(msg.vy);
  }

  if (omega_trait::has_value(msg.omega))
  {
    j["omega"] = omega_trait::get(msg.omega);
  }
}

//=============================================================================
template <typename VelocityT>
void from_json(const nlohmann::json& j, VelocityT& msg)
{
  using json_utils::optional_field_traits;

  using vx_trait = optional_field_traits<decltype(msg.vx)>;
  using vy_trait = optional_field_traits<decltype(msg.vy)>;
  using omega_trait = optional_field_traits<decltype(msg.omega)>;

  if (j.contains("vx"))
  {
    vx_trait::set(msg.vx, j.at("vx").get<double>());
  }

  if (j.contains("vy"))
  {
    vy_trait::set(msg.vy, j.at("vy").get<double>());
  }

  if (j.contains("omega"))
  {
    omega_trait::set(msg.omega, j.at("omega").get<double>());
  }
}

}  // namespace velocity_detail

namespace visualization_detail {

//=============================================================================
template <typename VisualizationT>
void to_json(nlohmann::json& j, const VisualizationT& msg)
{
  using json_utils::optional_field_traits;

  using agv_position_trait = optional_field_traits<decltype(msg.agv_position)>;
  using velocity_trait = optional_field_traits<decltype(msg.velocity)>;

  to_json(j, msg.header);

  if (agv_position_trait::has_value(msg.agv_position))
  {
    j["agvPosition"] = agv_position_trait::get(msg.agv_position);
  }

  if (velocity_trait::has_value(msg.velocity))
  {
    j["velocity"] = velocity_trait::get(msg.velocity);
  }
}

//=============================================================================
template <typename VisualizationT>
void from_json(const nlohmann::json& j, VisualizationT& msg)
{
  using json_utils::optional_field_traits;

  using agv_position_trait = optional_field_traits<decltype(msg.agv_position)>;
  using velocity_trait = optional_field_traits<decltype(msg.velocity)>;

  from_json(j, msg.header);

  if (j.contains("agvPosition"))
  {
    agv_position_trait::set(msg.agv_position, j.at("agvPosition"));
  }

  if (j.contains("velocity"))
  {
    velocity_trait::set(msg.velocity, j.at("velocity"));
  }
}

}  // namespace visualization_detail

namespace wheel_definition_detail {

//=============================================================================
template <typename WheelDefinitionT>
void to_json(nlohmann::json& j, const WheelDefinitionT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::wheel_definition_type_traits;

  using constraints_trait = optional_field_traits<decltype(msg.constraints)>;

  j["type"] =
    wheel_definition_type_traits<decltype(msg.type)>::to_string(msg.type);
  j["isActiveDriven"] = msg.is_active_driven;
  j["isActiveSteered"] = msg.is_active_steered;
  j["position"] = msg.position;
  j["diameter"] = msg.diameter;
  j["width"] = msg.width;
  j["centerDisplacement"] = msg.center_displacement;

  if (constraints_trait::has_value(msg.constraints))
  {
    j["constraints"] = constraints_trait::get(msg.constraints);
  }
}

//=============================================================================
template <typename WheelDefinitionT>
void from_json(const nlohmann::json& j, WheelDefinitionT& msg)
{
  using json_utils::optional_field_traits;
  using json_utils::wheel_definition_type_traits;

  using constraints_trait = optional_field_traits<decltype(msg.constraints)>;

  msg.type = wheel_definition_type_traits<decltype(msg.type)>::from_string(
    j.at("type").get<std::string>());
  msg.is_active_driven = j.at("isActiveDriven").get<bool>();
  msg.is_active_steered = j.at("isActiveSteered").get<bool>();
  msg.position = j.at("position");
  msg.diameter = j.at("diameter").get<double>();
  msg.width = j.at("width").get<double>();
  msg.center_displacement = j.at("centerDisplacement").get<double>();

  if (j.contains("constraints"))
  {
    constraints_trait::set(
      msg.constraints, j.at("constraints").get<std::string>());
  }
}

}  // namespace wheel_definition_detail

}  // namespace types
}  // namespace vda5050_core

//=============================================================================
namespace vda5050_core {

namespace types {

//=============================================================================
inline void to_json(nlohmann::json& j, const Action& msg)
{
  vda5050_core::types::action_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Action& msg)
{
  vda5050_core::types::action_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ActionParameter& msg)
{
  vda5050_core::types::action_parameter_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ActionParameter& msg)
{
  vda5050_core::types::action_parameter_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ActionParameterFactsheet& msg)
{
  vda5050_core::types::action_parameter_factsheet_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ActionParameterFactsheet& msg)
{
  vda5050_core::types::action_parameter_factsheet_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ActionState& msg)
{
  vda5050_core::types::action_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ActionState& msg)
{
  vda5050_core::types::action_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const AGVAction& msg)
{
  vda5050_core::types::agv_action_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, AGVAction& msg)
{
  vda5050_core::types::agv_action_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const AGVGeometry& msg)
{
  vda5050_core::types::agv_geometry_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, AGVGeometry& msg)
{
  vda5050_core::types::agv_geometry_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const AGVPosition& msg)
{
  vda5050_core::types::agv_position_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, AGVPosition& msg)
{
  vda5050_core::types::agv_position_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const BatteryState& msg)
{
  vda5050_core::types::battery_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, BatteryState& msg)
{
  vda5050_core::types::battery_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const BoundingBoxReference& msg)
{
  vda5050_core::types::bounding_box_reference_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, BoundingBoxReference& msg)
{
  vda5050_core::types::bounding_box_reference_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Connection& msg)
{
  vda5050_core::types::connection_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Connection& msg)
{
  vda5050_core::types::connection_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ControlPoint& msg)
{
  vda5050_core::types::control_point_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ControlPoint& msg)
{
  vda5050_core::types::control_point_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Edge& msg)
{
  vda5050_core::types::edge_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Edge& msg)
{
  vda5050_core::types::edge_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const EdgeState& msg)
{
  vda5050_core::types::edge_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, EdgeState& msg)
{
  vda5050_core::types::edge_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Envelope2d& msg)
{
  vda5050_core::types::envelope2d_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Envelope2d& msg)
{
  vda5050_core::types::envelope2d_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Envelope3d& msg)
{
  vda5050_core::types::envelope3d_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Envelope3d& msg)
{
  vda5050_core::types::envelope3d_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Error& msg)
{
  vda5050_core::types::error_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Error& msg)
{
  vda5050_core::types::error_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ErrorReference& msg)
{
  vda5050_core::types::error_reference_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ErrorReference& msg)
{
  vda5050_core::types::error_reference_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Factsheet& msg)
{
  vda5050_core::types::factsheet_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Factsheet& msg)
{
  vda5050_core::types::factsheet_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Header& msg)
{
  vda5050_core::types::header_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Header& msg)
{
  vda5050_core::types::header_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Info& msg)
{
  vda5050_core::types::info_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Info& msg)
{
  vda5050_core::types::info_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const InfoReference& msg)
{
  vda5050_core::types::info_reference_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, InfoReference& msg)
{
  vda5050_core::types::info_reference_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const InstantActions& msg)
{
  vda5050_core::types::instant_actions_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, InstantActions& msg)
{
  vda5050_core::types::instant_actions_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Load& msg)
{
  vda5050_core::types::load_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Load& msg)
{
  vda5050_core::types::load_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const LoadDimensions& msg)
{
  vda5050_core::types::load_dimensions_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, LoadDimensions& msg)
{
  vda5050_core::types::load_dimensions_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const LoadSet& msg)
{
  vda5050_core::types::load_set_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, LoadSet& msg)
{
  vda5050_core::types::load_set_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const LoadSpecification& msg)
{
  vda5050_core::types::load_specification_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, LoadSpecification& msg)
{
  vda5050_core::types::load_specification_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const MaxArrayLens& msg)
{
  vda5050_core::types::max_array_lens_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, MaxArrayLens& msg)
{
  vda5050_core::types::max_array_lens_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const MaxStringLens& msg)
{
  vda5050_core::types::max_string_lens_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, MaxStringLens& msg)
{
  vda5050_core::types::max_string_lens_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Node& msg)
{
  vda5050_core::types::node_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Node& msg)
{
  vda5050_core::types::node_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const NodePosition& msg)
{
  vda5050_core::types::node_position_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, NodePosition& msg)
{
  vda5050_core::types::node_position_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const NodeState& msg)
{
  vda5050_core::types::node_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, NodeState& msg)
{
  vda5050_core::types::node_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const OptionalParameter& msg)
{
  vda5050_core::types::optional_parameter_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, OptionalParameter& msg)
{
  vda5050_core::types::optional_parameter_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Order& msg)
{
  vda5050_core::types::order_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Order& msg)
{
  vda5050_core::types::order_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const PhysicalParameters& msg)
{
  vda5050_core::types::physical_parameters_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, PhysicalParameters& msg)
{
  vda5050_core::types::physical_parameters_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const PolygonPoint& msg)
{
  vda5050_core::types::polygon_point_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, PolygonPoint& msg)
{
  vda5050_core::types::polygon_point_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Position& msg)
{
  vda5050_core::types::position_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Position& msg)
{
  vda5050_core::types::position_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ProtocolFeatures& msg)
{
  vda5050_core::types::protocol_features_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ProtocolFeatures& msg)
{
  vda5050_core::types::protocol_features_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ProtocolLimits& msg)
{
  vda5050_core::types::protocol_limits_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ProtocolLimits& msg)
{
  vda5050_core::types::protocol_limits_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const SafetyState& msg)
{
  vda5050_core::types::safety_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, SafetyState& msg)
{
  vda5050_core::types::safety_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const State& msg)
{
  vda5050_core::types::state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, State& msg)
{
  vda5050_core::types::state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Timing& msg)
{
  vda5050_core::types::timing_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Timing& msg)
{
  vda5050_core::types::timing_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Trajectory& msg)
{
  vda5050_core::types::trajectory_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Trajectory& msg)
{
  vda5050_core::types::trajectory_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const TypeSpecification& msg)
{
  vda5050_core::types::type_specification_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, TypeSpecification& msg)
{
  vda5050_core::types::type_specification_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Velocity& msg)
{
  vda5050_core::types::velocity_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Velocity& msg)
{
  vda5050_core::types::velocity_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Visualization& msg)
{
  vda5050_core::types::visualization_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Visualization& msg)
{
  vda5050_core::types::visualization_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const WheelDefinition& msg)
{
  vda5050_core::types::wheel_definition_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, WheelDefinition& msg)
{
  vda5050_core::types::wheel_definition_detail::from_json(j, msg);
}

}  // namespace types
}  // namespace vda5050_core

//=============================================================================
#ifdef ENABLE_ROS2
namespace vda5050_interfaces {

namespace msg {

//=============================================================================
inline void to_json(nlohmann::json& j, const Action& msg)
{
  vda5050_core::types::action_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Action& msg)
{
  vda5050_core::types::action_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ActionParameter& msg)
{
  vda5050_core::types::action_parameter_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ActionParameter& msg)
{
  vda5050_core::types::action_parameter_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ActionParameterFactsheet& msg)
{
  vda5050_core::types::action_parameter_factsheet_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ActionParameterFactsheet& msg)
{
  vda5050_core::types::action_parameter_factsheet_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ActionState& msg)
{
  vda5050_core::types::action_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ActionState& msg)
{
  vda5050_core::types::action_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const AGVAction& msg)
{
  vda5050_core::types::agv_action_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, AGVAction& msg)
{
  vda5050_core::types::agv_action_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const AGVGeometry& msg)
{
  vda5050_core::types::agv_geometry_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, AGVGeometry& msg)
{
  vda5050_core::types::agv_geometry_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const AGVPosition& msg)
{
  vda5050_core::types::agv_position_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, AGVPosition& msg)
{
  vda5050_core::types::agv_position_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const BatteryState& msg)
{
  vda5050_core::types::battery_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, BatteryState& msg)
{
  vda5050_core::types::battery_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const BoundingBoxReference& msg)
{
  vda5050_core::types::bounding_box_reference_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, BoundingBoxReference& msg)
{
  vda5050_core::types::bounding_box_reference_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Connection& msg)
{
  vda5050_core::types::connection_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Connection& msg)
{
  vda5050_core::types::connection_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ControlPoint& msg)
{
  vda5050_core::types::control_point_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ControlPoint& msg)
{
  vda5050_core::types::control_point_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Edge& msg)
{
  vda5050_core::types::edge_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Edge& msg)
{
  vda5050_core::types::edge_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const EdgeState& msg)
{
  vda5050_core::types::edge_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, EdgeState& msg)
{
  vda5050_core::types::edge_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Envelope2d& msg)
{
  vda5050_core::types::envelope2d_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Envelope2d& msg)
{
  vda5050_core::types::envelope2d_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Envelope3d& msg)
{
  vda5050_core::types::envelope3d_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Envelope3d& msg)
{
  vda5050_core::types::envelope3d_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Error& msg)
{
  vda5050_core::types::error_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Error& msg)
{
  vda5050_core::types::error_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ErrorReference& msg)
{
  vda5050_core::types::error_reference_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ErrorReference& msg)
{
  vda5050_core::types::error_reference_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Factsheet& msg)
{
  vda5050_core::types::factsheet_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Factsheet& msg)
{
  vda5050_core::types::factsheet_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Header& msg)
{
  vda5050_core::types::header_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Header& msg)
{
  vda5050_core::types::header_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Info& msg)
{
  vda5050_core::types::info_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Info& msg)
{
  vda5050_core::types::info_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const InfoReference& msg)
{
  vda5050_core::types::info_reference_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, InfoReference& msg)
{
  vda5050_core::types::info_reference_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const InstantActions& msg)
{
  vda5050_core::types::instant_actions_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, InstantActions& msg)
{
  vda5050_core::types::instant_actions_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Load& msg)
{
  vda5050_core::types::load_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Load& msg)
{
  vda5050_core::types::load_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const LoadDimensions& msg)
{
  vda5050_core::types::load_dimensions_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, LoadDimensions& msg)
{
  vda5050_core::types::load_dimensions_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const LoadSet& msg)
{
  vda5050_core::types::load_set_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, LoadSet& msg)
{
  vda5050_core::types::load_set_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const LoadSpecification& msg)
{
  vda5050_core::types::load_specification_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, LoadSpecification& msg)
{
  vda5050_core::types::load_specification_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const MaxArrayLens& msg)
{
  vda5050_core::types::max_array_lens_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, MaxArrayLens& msg)
{
  vda5050_core::types::max_array_lens_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const MaxStringLens& msg)
{
  vda5050_core::types::max_string_lens_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, MaxStringLens& msg)
{
  vda5050_core::types::max_string_lens_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Node& msg)
{
  vda5050_core::types::node_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Node& msg)
{
  vda5050_core::types::node_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const NodePosition& msg)
{
  vda5050_core::types::node_position_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, NodePosition& msg)
{
  vda5050_core::types::node_position_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const NodeState& msg)
{
  vda5050_core::types::node_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, NodeState& msg)
{
  vda5050_core::types::node_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const OptionalParameter& msg)
{
  vda5050_core::types::optional_parameter_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, OptionalParameter& msg)
{
  vda5050_core::types::optional_parameter_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Order& msg)
{
  vda5050_core::types::order_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Order& msg)
{
  vda5050_core::types::order_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const PhysicalParameters& msg)
{
  vda5050_core::types::physical_parameters_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, PhysicalParameters& msg)
{
  vda5050_core::types::physical_parameters_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const PolygonPoint& msg)
{
  vda5050_core::types::polygon_point_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, PolygonPoint& msg)
{
  vda5050_core::types::polygon_point_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Position& msg)
{
  vda5050_core::types::position_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Position& msg)
{
  vda5050_core::types::position_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ProtocolFeatures& msg)
{
  vda5050_core::types::protocol_features_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ProtocolFeatures& msg)
{
  vda5050_core::types::protocol_features_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const ProtocolLimits& msg)
{
  vda5050_core::types::protocol_limits_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, ProtocolLimits& msg)
{
  vda5050_core::types::protocol_limits_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const SafetyState& msg)
{
  vda5050_core::types::safety_state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, SafetyState& msg)
{
  vda5050_core::types::safety_state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const State& msg)
{
  vda5050_core::types::state_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, State& msg)
{
  vda5050_core::types::state_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Timing& msg)
{
  vda5050_core::types::timing_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Timing& msg)
{
  vda5050_core::types::timing_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Trajectory& msg)
{
  vda5050_core::types::trajectory_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Trajectory& msg)
{
  vda5050_core::types::trajectory_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const TypeSpecification& msg)
{
  vda5050_core::types::type_specification_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, TypeSpecification& msg)
{
  vda5050_core::types::type_specification_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Velocity& msg)
{
  vda5050_core::types::velocity_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Velocity& msg)
{
  vda5050_core::types::velocity_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const Visualization& msg)
{
  vda5050_core::types::visualization_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, Visualization& msg)
{
  vda5050_core::types::visualization_detail::from_json(j, msg);
}

//=============================================================================
inline void to_json(nlohmann::json& j, const WheelDefinition& msg)
{
  vda5050_core::types::wheel_definition_detail::to_json(j, msg);
}

inline void from_json(const nlohmann::json& j, WheelDefinition& msg)
{
  vda5050_core::types::wheel_definition_detail::from_json(j, msg);
}

}  // namespace msg

}  // namespace vda5050_interfaces
#endif  // ENABLE_ROS2

#endif  // VDA5050_CORE__JSON_UTILS__SERIALIZATION_HPP_
