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

#ifndef VDA5050_CORE__JSON_UTILS__LAYOUT_SERIALIZATION_HPP_
#define VDA5050_CORE__JSON_UTILS__LAYOUT_SERIALIZATION_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "vda5050_core/layout/edge.hpp"
#include "vda5050_core/layout/layout.hpp"
#include "vda5050_core/layout/layout_action.hpp"
#include "vda5050_core/layout/lif.hpp"
#include "vda5050_core/layout/load_restriction.hpp"
#include "vda5050_core/layout/node.hpp"
#include "vda5050_core/layout/station.hpp"
#include "vda5050_core/layout/trajectory.hpp"
#include "vda5050_core/layout/vehicle_type_edge_property.hpp"
#include "vda5050_core/layout/vehicle_type_node_property.hpp"
#include "vda5050_core/types/blocking_type.hpp"

namespace vda5050_core::layout {

inline std::string to_string(RequirementType v)
{
  switch (v)
  {
    case RequirementType::REQUIRED:
      return "REQUIRED";
    case RequirementType::CONDITIONAL:
      return "CONDITIONAL";
    case RequirementType::OPTIONAL:
      return "OPTIONAL";
  }
  throw std::invalid_argument("Unknown RequirementType");
}
inline RequirementType requirement_type_from_string(const std::string& s)
{
  if (s == "REQUIRED") return RequirementType::REQUIRED;
  if (s == "CONDITIONAL") return RequirementType::CONDITIONAL;
  if (s == "OPTIONAL") return RequirementType::OPTIONAL;
  throw std::invalid_argument("Unknown RequirementType value: '" + s + "'");
}

inline std::string to_string(OrientationType v)
{
  switch (v)
  {
    case OrientationType::GLOBAL:
      return "GLOBAL";
    case OrientationType::TANGENTIAL:
      return "TANGENTIAL";
  }
  throw std::invalid_argument("Unknown OrientationType");
}
inline OrientationType orientation_type_from_string(const std::string& s)
{
  if (s == "GLOBAL") return OrientationType::GLOBAL;
  if (s == "TANGENTIAL") return OrientationType::TANGENTIAL;
  throw std::invalid_argument("Unknown OrientationType value: '" + s + "'");
}

inline std::string to_string(RotationAtNode v)
{
  switch (v)
  {
    case RotationAtNode::NONE:
      return "NONE";
    case RotationAtNode::CCW:
      return "CCW";
    case RotationAtNode::CW:
      return "CW";
    case RotationAtNode::BOTH:
      return "BOTH";
  }
  throw std::invalid_argument("Unknown RotationAtNode");
}
inline RotationAtNode rotation_at_node_from_string(const std::string& s)
{
  if (s == "NONE") return RotationAtNode::NONE;
  if (s == "CCW") return RotationAtNode::CCW;
  if (s == "CW") return RotationAtNode::CW;
  if (s == "BOTH") return RotationAtNode::BOTH;
  throw std::invalid_argument("Unknown RotationAtNode value: '" + s + "'");
}

inline std::string to_string(vda5050_core::types::BlockingType v)
{
  using BT = vda5050_core::types::BlockingType;
  switch (v)
  {
    case BT::NONE:
      return "NONE";
    case BT::SOFT:
      return "SOFT";
    case BT::HARD:
      return "HARD";
  }
  throw std::invalid_argument("Unknown BlockingType");
}
inline vda5050_core::types::BlockingType blocking_type_from_string(
  const std::string& s)
{
  using BT = vda5050_core::types::BlockingType;
  if (s == "NONE") return BT::NONE;
  if (s == "SOFT") return BT::SOFT;
  if (s == "HARD") return BT::HARD;
  throw std::invalid_argument("Unknown BlockingType value: '" + s + "'");
}

namespace control_point_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  if (msg.weight) j["weight"] = *msg.weight;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  if (j.contains("weight")) msg.weight = j.at("weight").get<double>();
}

}  // namespace control_point_detail

inline void to_json(nlohmann::json& j, const ControlPoint& v)
{
  control_point_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, ControlPoint& v)
{
  control_point_detail::from_json(j, v);
}

namespace trajectory_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["knotVector"] = msg.knot_vector;
  j["controlPoints"] = msg.control_points;
  if (msg.degree) j["degree"] = *msg.degree;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.knot_vector = j.at("knotVector").get<std::vector<double>>();
  msg.control_points = j.at("controlPoints").get<std::vector<ControlPoint>>();
  if (j.contains("degree")) msg.degree = j.at("degree").get<int>();
}

}  // namespace trajectory_detail

inline void to_json(nlohmann::json& j, const Trajectory& v)
{
  trajectory_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, Trajectory& v)
{
  trajectory_detail::from_json(j, v);
}

namespace load_restriction_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["unloaded"] = msg.unloaded;
  j["loaded"] = msg.loaded;
  if (msg.load_set_names) j["loadSetNames"] = *msg.load_set_names;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.unloaded = j.at("unloaded").get<bool>();
  msg.loaded = j.at("loaded").get<bool>();
  if (j.contains("loadSetNames"))
  {
    msg.load_set_names = j.at("loadSetNames").get<std::vector<std::string>>();
  }
}

}  // namespace load_restriction_detail

inline void to_json(nlohmann::json& j, const LoadRestriction& v)
{
  load_restriction_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, LoadRestriction& v)
{
  load_restriction_detail::from_json(j, v);
}

namespace action_parameter_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["key"] = msg.key;
  j["value"] = msg.value;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.key = j.at("key").get<std::string>();
  msg.value = j.at("value").get<std::string>();
}

}  // namespace action_parameter_detail

inline void to_json(nlohmann::json& j, const ActionParameter& v)
{
  action_parameter_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, ActionParameter& v)
{
  action_parameter_detail::from_json(j, v);
}

namespace action_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["actionType"] = msg.action_type;
  j["blockingType"] = to_string(msg.blocking_type);
  if (msg.action_description) j["actionDescription"] = *msg.action_description;
  if (msg.requirement_type)
  {
    j["requirementType"] = to_string(*msg.requirement_type);
  }
  if (msg.action_parameters) j["actionParameters"] = *msg.action_parameters;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.action_type = j.at("actionType").get<std::string>();
  msg.blocking_type =
    blocking_type_from_string(j.at("blockingType").get<std::string>());
  if (j.contains("actionDescription"))
  {
    msg.action_description = j.at("actionDescription").get<std::string>();
  }
  if (j.contains("requirementType"))
  {
    msg.requirement_type =
      requirement_type_from_string(j.at("requirementType").get<std::string>());
  }
  if (j.contains("actionParameters"))
  {
    msg.action_parameters =
      j.at("actionParameters").get<std::vector<ActionParameter>>();
  }
}

}  // namespace action_detail

inline void to_json(nlohmann::json& j, const Action& v)
{
  action_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, Action& v)
{
  action_detail::from_json(j, v);
}

namespace node_position_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
}

}  // namespace node_position_detail

inline void to_json(nlohmann::json& j, const NodePosition& v)
{
  node_position_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, NodePosition& v)
{
  node_position_detail::from_json(j, v);
}

namespace vehicle_type_node_property_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["vehicleTypeId"] = msg.vehicle_type_id;
  if (msg.theta) j["theta"] = *msg.theta;
  if (msg.actions) j["actions"] = *msg.actions;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.vehicle_type_id = j.at("vehicleTypeId").get<std::string>();
  if (j.contains("theta")) msg.theta = j.at("theta").get<double>();
  if (j.contains("actions"))
  {
    msg.actions = j.at("actions").get<std::vector<Action>>();
  }
}

}  // namespace vehicle_type_node_property_detail

inline void to_json(nlohmann::json& j, const VehicleTypeNodeProperty& v)
{
  vehicle_type_node_property_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, VehicleTypeNodeProperty& v)
{
  vehicle_type_node_property_detail::from_json(j, v);
}

namespace vehicle_type_edge_property_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["vehicleTypeId"] = msg.vehicle_type_id;
  j["rotationAllowed"] = msg.rotation_allowed;

  if (msg.vehicle_orientation)
    j["vehicleOrientation"] = *msg.vehicle_orientation;
  if (msg.orientation_type)
    j["orientationType"] = to_string(*msg.orientation_type);
  if (msg.rotation_at_start_node_allowed)
    j["rotationAtStartNodeAllowed"] =
      to_string(*msg.rotation_at_start_node_allowed);
  if (msg.rotation_at_end_node_allowed)
    j["rotationAtEndNodeAllowed"] =
      to_string(*msg.rotation_at_end_node_allowed);
  if (msg.max_speed) j["maxSpeed"] = *msg.max_speed;
  if (msg.max_rotation_speed) j["maxRotationSpeed"] = *msg.max_rotation_speed;
  if (msg.min_height) j["minHeight"] = *msg.min_height;
  if (msg.max_height) j["maxHeight"] = *msg.max_height;
  if (msg.load_restriction) j["loadRestriction"] = *msg.load_restriction;
  if (msg.actions) j["actions"] = *msg.actions;
  if (msg.trajectory) j["trajectory"] = *msg.trajectory;
  if (msg.reentry_allowed) j["reentryAllowed"] = *msg.reentry_allowed;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.vehicle_type_id = j.at("vehicleTypeId").get<std::string>();
  msg.rotation_allowed = j.at("rotationAllowed").get<bool>();

  if (j.contains("vehicleOrientation"))
    msg.vehicle_orientation = j.at("vehicleOrientation").get<double>();
  if (j.contains("orientationType"))
    msg.orientation_type =
      orientation_type_from_string(j.at("orientationType").get<std::string>());
  if (j.contains("rotationAtStartNodeAllowed"))
    msg.rotation_at_start_node_allowed = rotation_at_node_from_string(
      j.at("rotationAtStartNodeAllowed").get<std::string>());
  if (j.contains("rotationAtEndNodeAllowed"))
    msg.rotation_at_end_node_allowed = rotation_at_node_from_string(
      j.at("rotationAtEndNodeAllowed").get<std::string>());

  if (j.contains("maxSpeed")) msg.max_speed = j.at("maxSpeed").get<double>();
  if (j.contains("maxRotationSpeed"))
    msg.max_rotation_speed = j.at("maxRotationSpeed").get<double>();
  if (j.contains("minHeight")) msg.min_height = j.at("minHeight").get<double>();
  if (j.contains("maxHeight")) msg.max_height = j.at("maxHeight").get<double>();
  if (j.contains("loadRestriction"))
    msg.load_restriction = j.at("loadRestriction").get<LoadRestriction>();
  if (j.contains("actions"))
    msg.actions = j.at("actions").get<std::vector<Action>>();
  if (j.contains("trajectory"))
    msg.trajectory = j.at("trajectory").get<Trajectory>();
  if (j.contains("reentryAllowed"))
    msg.reentry_allowed = j.at("reentryAllowed").get<bool>();
}

}  // namespace vehicle_type_edge_property_detail

inline void to_json(nlohmann::json& j, const VehicleTypeEdgeProperty& v)
{
  vehicle_type_edge_property_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, VehicleTypeEdgeProperty& v)
{
  vehicle_type_edge_property_detail::from_json(j, v);
}

namespace node_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["nodeId"] = msg.node_id;
  j["mapId"] = msg.map_id;
  j["nodePosition"] = msg.node_position;
  j["vehicleTypeNodeProperties"] = msg.vehicle_type_node_properties;
  if (msg.node_name) j["nodeName"] = *msg.node_name;
  if (msg.node_description) j["nodeDescription"] = *msg.node_description;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.node_id = j.at("nodeId").get<std::string>();
  msg.map_id = j.at("mapId").get<std::string>();
  msg.node_position = j.at("nodePosition").get<NodePosition>();
  msg.vehicle_type_node_properties =
    j.at("vehicleTypeNodeProperties")
      .get<std::vector<VehicleTypeNodeProperty>>();
  if (j.contains("nodeName"))
    msg.node_name = j.at("nodeName").get<std::string>();
  if (j.contains("nodeDescription"))
    msg.node_description = j.at("nodeDescription").get<std::string>();
}

}  // namespace node_detail

inline void to_json(nlohmann::json& j, const Node& v)
{
  node_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, Node& v)
{
  node_detail::from_json(j, v);
}

namespace edge_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["edgeId"] = msg.edge_id;
  j["startNodeId"] = msg.start_node_id;
  j["endNodeId"] = msg.end_node_id;
  j["vehicleTypeEdgeProperties"] = msg.vehicle_type_edge_properties;
  if (msg.edge_name) j["edgeName"] = *msg.edge_name;
  if (msg.edge_description) j["edgeDescription"] = *msg.edge_description;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.edge_id = j.at("edgeId").get<std::string>();
  msg.start_node_id = j.at("startNodeId").get<std::string>();
  msg.end_node_id = j.at("endNodeId").get<std::string>();
  msg.vehicle_type_edge_properties =
    j.at("vehicleTypeEdgeProperties")
      .get<std::vector<VehicleTypeEdgeProperty>>();
  if (j.contains("edgeName"))
    msg.edge_name = j.at("edgeName").get<std::string>();
  if (j.contains("edgeDescription"))
    msg.edge_description = j.at("edgeDescription").get<std::string>();
}

}  // namespace edge_detail

inline void to_json(nlohmann::json& j, const Edge& v)
{
  edge_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, Edge& v)
{
  edge_detail::from_json(j, v);
}

namespace station_position_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["x"] = msg.x;
  j["y"] = msg.y;
  if (msg.theta) j["theta"] = *msg.theta;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.x = j.at("x").get<double>();
  msg.y = j.at("y").get<double>();
  if (j.contains("theta")) msg.theta = j.at("theta").get<double>();
}

}  // namespace station_position_detail

inline void to_json(nlohmann::json& j, const StationPosition& v)
{
  station_position_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, StationPosition& v)
{
  station_position_detail::from_json(j, v);
}

namespace station_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["stationId"] = msg.station_id;
  j["interactionNodeIds"] = msg.interaction_node_ids;
  if (msg.station_name) j["stationName"] = *msg.station_name;
  if (msg.station_description)
    j["stationDescription"] = *msg.station_description;
  if (msg.station_height) j["stationHeight"] = *msg.station_height;
  if (msg.station_position) j["stationPosition"] = *msg.station_position;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.station_id = j.at("stationId").get<std::string>();
  msg.interaction_node_ids =
    j.at("interactionNodeIds").get<std::vector<std::string>>();
  if (j.contains("stationName"))
    msg.station_name = j.at("stationName").get<std::string>();
  if (j.contains("stationDescription"))
    msg.station_description = j.at("stationDescription").get<std::string>();
  if (j.contains("stationHeight"))
    msg.station_height = j.at("stationHeight").get<double>();
  if (j.contains("stationPosition"))
    msg.station_position = j.at("stationPosition").get<StationPosition>();
}

}  // namespace station_detail

inline void to_json(nlohmann::json& j, const Station& v)
{
  station_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, Station& v)
{
  station_detail::from_json(j, v);
}

namespace layout_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["layoutId"] = msg.layout_id;
  j["layoutVersion"] = msg.layout_version;
  j["nodes"] = msg.nodes;
  j["edges"] = msg.edges;
  j["stations"] = msg.stations;
  if (msg.layout_name) j["layoutName"] = *msg.layout_name;
  if (msg.layout_level_id) j["layoutLevelId"] = *msg.layout_level_id;
  if (msg.layout_description) j["layoutDescription"] = *msg.layout_description;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.layout_id = j.at("layoutId").get<std::string>();
  msg.layout_version = j.at("layoutVersion").get<std::string>();
  msg.nodes = j.at("nodes").get<std::vector<Node>>();
  msg.edges = j.at("edges").get<std::vector<Edge>>();
  msg.stations = j.at("stations").get<std::vector<Station>>();
  if (j.contains("layoutName"))
    msg.layout_name = j.at("layoutName").get<std::string>();
  if (j.contains("layoutLevelId"))
    msg.layout_level_id = j.at("layoutLevelId").get<std::string>();
  if (j.contains("layoutDescription"))
    msg.layout_description = j.at("layoutDescription").get<std::string>();
}

}  // namespace layout_detail

inline void to_json(nlohmann::json& j, const Layout& v)
{
  layout_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, Layout& v)
{
  layout_detail::from_json(j, v);
}

namespace meta_information_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["projectIdentification"] = msg.project_identification;
  j["creator"] = msg.creator;
  j["exportTimestamp"] = msg.export_timestamp;
  j["lifVersion"] = msg.lif_version;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.project_identification = j.at("projectIdentification").get<std::string>();
  msg.creator = j.at("creator").get<std::string>();
  msg.export_timestamp = j.at("exportTimestamp").get<std::string>();
  msg.lif_version = j.at("lifVersion").get<std::string>();
}

}  // namespace meta_information_detail

inline void to_json(nlohmann::json& j, const MetaInformation& v)
{
  meta_information_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, MetaInformation& v)
{
  meta_information_detail::from_json(j, v);
}

namespace lif_detail {

template <typename T>
void to_json(nlohmann::json& j, const T& msg)
{
  j["metaInformation"] = msg.meta_information;
  j["layouts"] = msg.layouts;
}

template <typename T>
void from_json(const nlohmann::json& j, T& msg)
{
  msg.meta_information = j.at("metaInformation").get<MetaInformation>();
  msg.layouts = j.at("layouts").get<std::vector<Layout>>();
}

}  // namespace lif_detail

inline void to_json(nlohmann::json& j, const LIF& v)
{
  lif_detail::to_json(j, v);
}
inline void from_json(const nlohmann::json& j, LIF& v)
{
  lif_detail::from_json(j, v);
}

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__JSON_UTILS__LAYOUT_SERIALIZATION_HPP_
