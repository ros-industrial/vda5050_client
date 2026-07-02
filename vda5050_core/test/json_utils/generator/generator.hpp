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

#ifndef JSON_UTILS__GENERATOR__GENERATOR_HPP_
#define JSON_UTILS__GENERATOR__GENERATOR_HPP_

#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

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

/// \brief Utility class to generate random instances of VDA 5050 message types
class RandomDataGenerator
{
public:
  /// \brief Default constructor using a non-deterministic seed
  RandomDataGenerator()
  : rng_(std::random_device()()),
    uint_dist_(0, std::numeric_limits<uint32_t>::max()),
    int_dist_(0, 100),
    float_dist_(
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
    bool_dist_(0, 1),
    string_length_dist_(0, 50),
    size_dist_(0, 10),
    percentage_dist_(0, 100)
  {
    // Nothing to do
  }

  /// \brief Constructor with a fixed seed for deterministic results
  explicit RandomDataGenerator(uint32_t seed)
  : rng_(seed),
    uint_dist_(0, std::numeric_limits<uint32_t>::max()),
    int_dist_(0, 100),
    float_dist_(
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max()),
    bool_dist_(0, 1),
    string_length_dist_(0, 50),
    size_dist_(0, 10),
    percentage_dist_(0, 100)
  {
    // Nothing to do
  }

  /// \brief Generate a random unsigned 32-bit integer
  uint32_t generate_random_uint()
  {
    return uint_dist_(rng_);
  }

  /// \brief Generate a random signed 8-bit integer
  int8_t generate_random_int()
  {
    return int_dist_(rng_);
  }

  /// \brief Generate a random boolean value
  bool generate_random_bool()
  {
    return bool_dist_(rng_);
  }

  /// \brief Generate a random index for enum selection
  uint8_t generate_random_index(size_t size)
  {
    // uniform_int_distribution does not permit char-sized types (MSVC enforces
    // this); use int and narrow on return.
    std::uniform_int_distribution<int> index_dist(
      0, static_cast<int>(size) - 1);
    return index_dist(rng_);
  }

  /// \brief Generate a random value for vector length
  uint8_t generate_random_size()
  {
    return size_dist_(rng_);
  }

  /// \brief Generate a random 64-bit floating-point number
  double generate_random_float()
  {
    return float_dist_(rng_);
  }

  /// \brief Generate a random alphanumerical string with length upto 50
  std::string generate_random_string()
  {
    int length = string_length_dist_(rng_);
    std::string s;
    s.reserve(length);

    const std::string charset =
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::uniform_int_distribution<size_t> char_index_dist(
      0, charset.length() - 1);

    for (int i = 0; i < length; i++)
    {
      s += charset[char_index_dist(rng_)];
    }
    return s;
  }

  /// \brief Generate current timestamp
  std::chrono::time_point<std::chrono::system_clock> generate_timestamp()
  {
    return std::chrono::system_clock::now();
  }

  /// \brief Generate a random percentage value
  double generate_random_percentage()
  {
    return percentage_dist_(rng_);
  }

  /// \brief Generate a random connectionState value
  ConnectionState generate_random_connection_state()
  {
    std::vector<ConnectionState> states = {
      ConnectionState::ONLINE, ConnectionState::OFFLINE,
      ConnectionState::CONNECTIONBROKEN};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random operatingMode value
  OperatingMode generate_random_operating_mode()
  {
    std::vector<OperatingMode> mode = {
      OperatingMode::AUTOMATIC, OperatingMode::SEMIAUTOMATIC,
      OperatingMode::MANUAL, OperatingMode::SERVICE, OperatingMode::TEACHIN};
    auto mode_idx = generate_random_index(mode.size());
    return mode[mode_idx];
  }

  /// \brief Generate a random actionStatus value
  ActionStatus generate_action_status()
  {
    std::vector<ActionStatus> statuses = {
      ActionStatus::WAITING, ActionStatus::INITIALIZING, ActionStatus::RUNNING,
      ActionStatus::PAUSED, ActionStatus::FINISHED};
    auto status_idx = generate_random_index(statuses.size());
    return statuses[status_idx];
  }

  /// \brief Generate a random errorLevel value
  ErrorLevel generate_random_error_level()
  {
    std::vector<ErrorLevel> levels = {ErrorLevel::WARNING, ErrorLevel::FATAL};
    auto level_idx = generate_random_index(levels.size());
    return levels[level_idx];
  }

  /// \brief Generate a random eStop value
  EStop generate_random_e_stop()
  {
    std::vector<EStop> types = {
      EStop::AUTOACK, EStop::MANUAL, EStop::REMOTE, EStop::NONE};
    auto type_idx = generate_random_index(types.size());
    return types[type_idx];
  }

  /// \brief Generate a random infoLevel value
  InfoLevel generate_random_info_level()
  {
    std::vector<InfoLevel> levels = {InfoLevel::INFO, InfoLevel::DEBUG};
    auto level_idx = generate_random_index(levels.size());
    return levels[level_idx];
  }

  /// \brief Generate a random orientation type value
  OrientationType generate_random_orientation_type()
  {
    std::vector<OrientationType> types = {
      OrientationType::TANGENTIAL, OrientationType::GLOBAL};
    auto type_idx = generate_random_index(types.size());
    return types[type_idx];
  }

  /// \brief Generte a random blocking type value
  BlockingType generate_random_blocking_type()
  {
    std::vector<BlockingType> types = {
      BlockingType::NONE, BlockingType::SOFT, BlockingType::HARD};
    auto type_idx = generate_random_index(types.size());
    return types[type_idx];
  }

  /// \brief Generate a random blocking types vector
  std::vector<BlockingType> generate_random_blocking_types()
  {
    std::vector<BlockingType> types(generate_random_size());
    for (auto it = types.begin(); it != types.end(); ++it)
    {
      *it = generate_random_blocking_type();
    }
    return types;
  }

  /// \brief Generate a random agv kinematic value
  AGVKinematic generate_random_agv_kinematic()
  {
    std::vector<AGVKinematic> states = {
      AGVKinematic::OMNI, AGVKinematic::DIFF, AGVKinematic::THREEWHEEL};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random agv class value
  AGVClass generate_random_agv_class()
  {
    std::vector<AGVClass> states = {
      AGVClass::FORKLIFT, AGVClass::CONVEYOR, AGVClass::TUGGER,
      AGVClass::CARRIER};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random support value
  Support generate_random_support()
  {
    std::vector<Support> states = {Support::SUPPORTED, Support::REQUIRED};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random action scope value
  ActionScope generate_random_action_scope()
  {
    std::vector<ActionScope> states = {
      ActionScope::INSTANT, ActionScope::NODE, ActionScope::EDGE};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random action scope vector
  std::vector<ActionScope> generate_random_action_scopes()
  {
    std::vector<ActionScope> scopes(generate_random_size());
    for (auto it = scopes.begin(); it != scopes.end(); ++it)
    {
      *it = generate_random_action_scope();
    }
    return scopes;
  }
  /// \brief Generate a random valueDataType value
  ValueDataType generate_random_value_data_type()
  {
    std::vector<ValueDataType> states = {
      ValueDataType::ARRAY,   ValueDataType::BOOL,   ValueDataType::FLOAT,
      ValueDataType::INTEGER, ValueDataType::NUMBER, ValueDataType::OBJECT};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random wheelDefinition type
  WheelDefinitionType generate_random_wheel_definition_type()
  {
    std::vector<WheelDefinitionType> states = {
      WheelDefinitionType::DRIVE, WheelDefinitionType::CASTER,
      WheelDefinitionType::FIXED, WheelDefinitionType::MECANUM};
    auto state_idx = generate_random_index(states.size());
    return states[state_idx];
  }

  /// \brief Generate a random vector of type float64
  std::vector<double> generate_random_float_vector(const uint8_t size)
  {
    std::vector<double> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate_random_float();
    }
    return vec;
  }

  /// \brief Generate a random vector of type string
  std::vector<std::string> generate_random_string_vector(const uint8_t size)
  {
    std::vector<std::string> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate_random_string();
    }
    return vec;
  }

  /// \brief Generate a random vector of type T
  template <typename T>
  std::vector<T> generate_random_vector(const uint8_t size)
  {
    std::vector<T> vec(size);
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
      *it = generate<T>();
    }
    return vec;
  }

  /// \brief Generate a fully populated message of a supported type
  template <typename T>
  T generate()
  {
    T msg;
    if constexpr (std::is_same_v<T, Action>)
    {
      msg.action_type = generate_random_string();
      msg.action_id = generate_random_string();
      msg.blocking_type = generate_random_blocking_type();
      msg.action_description = generate_random_string();
      msg.action_parameters =
        generate_random_vector<ActionParameter>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, ActionParameter>)
    {
      msg.key = generate_random_string();
      msg.value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, ActionParameterFactsheet>)
    {
      msg.key = generate_random_string();
      msg.value_data_type = generate_random_value_data_type();
      msg.description = generate_random_string();
      msg.is_optional = generate_random_bool();
    }
    else if constexpr (std::is_same_v<T, ActionState>)
    {
      msg.action_id = generate_random_string();
      msg.action_type = generate_random_string();
      msg.action_description = generate_random_string();
      msg.action_status = generate_action_status();
      msg.result_description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, AGVPosition>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.theta = generate_random_float();
      msg.map_id = generate_random_string();
      msg.map_description = generate_random_string();
      msg.position_initialized = generate_random_bool();
      msg.localization_score = generate_random_float();
      msg.deviation_range = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, AGVAction>)
    {
      msg.action_type = generate_random_string();
      msg.action_scopes = generate_random_action_scopes();
      msg.action_parameters = generate_random_vector<ActionParameterFactsheet>(
        generate_random_size());
      msg.result_description = generate_random_string();
      msg.action_description = generate_random_string();
      msg.blocking_types = generate_random_blocking_types();
    }
    else if constexpr (std::is_same_v<T, AGVGeometry>)
    {
      msg.wheel_definitions =
        generate_random_vector<WheelDefinition>(generate_random_size());
      msg.envelopes2d =
        generate_random_vector<Envelope2d>(generate_random_size());
      msg.envelopes3d =
        generate_random_vector<Envelope3d>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, BatteryState>)
    {
      msg.battery_charge = generate_random_percentage();
      msg.battery_voltage = generate_random_float();
      msg.battery_health = generate_random_int();
      msg.charging = generate_random_bool();
      msg.reach = generate_random_uint();
    }
    else if constexpr (std::is_same_v<T, BoundingBoxReference>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.z = generate_random_float();
      msg.theta = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, Connection>)
    {
      msg.header = generate<Header>();
      msg.connection_state = generate_random_connection_state();
    }
    else if constexpr (std::is_same_v<T, ControlPoint>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.weight = 1.0;
    }
    else if constexpr (std::is_same_v<T, Edge>)
    {
      msg.edge_id = generate_random_string();
      msg.sequence_id = generate_random_uint();
      msg.start_node_id = generate_random_string();
      msg.end_node_id = generate_random_string();
      msg.released = generate_random_bool();
      msg.actions = generate_random_vector<Action>(generate_random_size());
      msg.edge_description = generate_random_string();
      msg.max_speed = generate_random_float();
      msg.max_height = generate_random_float();
      msg.min_height = generate_random_float();
      msg.orientation = generate_random_float();
      msg.orientation_type = generate_random_orientation_type();
      msg.direction = generate_random_string();
      msg.rotation_allowed = generate_random_bool();
      msg.max_rotation_speed = generate_random_float();
      msg.trajectory = generate<Trajectory>();
      msg.length = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, EdgeState>)
    {
      msg.edge_id = generate_random_string();
      msg.sequence_id = generate_random_uint();
      msg.edge_description = generate_random_string();
      msg.released = generate_random_bool();
      msg.trajectory = generate<Trajectory>();
    }
    else if constexpr (std::is_same_v<T, Envelope2d>)
    {
      msg.set = generate_random_string();
      msg.polygon_points =
        generate_random_vector<PolygonPoint>(generate_random_size());
      msg.description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Envelope3d>)
    {
      msg.set = generate_random_string();
      msg.format = generate_random_string();
      msg.data = generate_random_string();
      msg.url = generate_random_string();
      msg.description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Error>)
    {
      msg.error_type = generate_random_string();
      msg.error_references =
        generate_random_vector<ErrorReference>(generate_random_size());
      msg.error_description = generate_random_string();
      msg.error_level = generate_random_error_level();
    }
    else if constexpr (std::is_same_v<T, ErrorReference>)
    {
      msg.reference_key = generate_random_string();
      msg.reference_value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Factsheet>)
    {
      msg.header = generate<Header>();
      msg.type_specification = generate<TypeSpecification>();
      msg.physical_parameters = generate<PhysicalParameters>();
      msg.protocol_limits = generate<ProtocolLimits>();
      msg.protocol_features = generate<ProtocolFeatures>();
      msg.agv_geometry = generate<AGVGeometry>();
      msg.load_specification = generate<LoadSpecification>();
      msg.localization_parameters = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Header>)
    {
      msg.header_id = generate_random_uint();
      msg.timestamp = generate_timestamp();
      msg.version = "2.0.0";  // Fix the VDA 5050 version to 2.0.0
      msg.manufacturer = generate_random_string();
      msg.serial_number = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Info>)
    {
      msg.info_type = generate_random_string();
      msg.info_references = generate_random_vector<InfoReference>(5);
      msg.info_description = generate_random_string();
      msg.info_level = generate_random_info_level();
    }
    else if constexpr (std::is_same_v<T, InfoReference>)
    {
      msg.reference_key = generate_random_string();
      msg.reference_value = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Load>)
    {
      msg.load_id = generate_random_string();
      msg.load_type = generate_random_string();
      msg.load_position = generate_random_string();
      msg.bounding_box_reference = generate<BoundingBoxReference>();
      msg.load_dimensions = generate<LoadDimensions>();
      msg.weight = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, InstantActions>)
    {
      msg.header = generate<Header>();
      msg.actions = generate_random_vector<Action>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, LoadDimensions>)
    {
      msg.length = generate_random_float();
      msg.width = generate_random_float();
      msg.height = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, LoadSet>)
    {
      msg.set_name = generate_random_string();
      msg.load_type = generate_random_string();
      msg.load_positions =
        generate_random_string_vector(generate_random_size());
      msg.bounding_box_reference = generate<BoundingBoxReference>();
      msg.load_dimensions = generate<LoadDimensions>();
      msg.max_weight = generate_random_float();
      msg.min_load_handling_height = generate_random_float();
      msg.max_load_handling_height = generate_random_float();
      msg.min_load_handling_depth = generate_random_float();
      msg.max_load_handling_depth = generate_random_float();
      msg.min_load_handling_tilt = generate_random_float();
      msg.max_load_handling_tilt = generate_random_float();
      msg.agv_speed_limit = generate_random_float();
      msg.agv_acceleration_limit = generate_random_float();
      msg.agv_deceleration_limit = generate_random_float();
      msg.pick_time = generate_random_float();
      msg.drop_time = generate_random_float();
      msg.description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, LoadSpecification>)
    {
      msg.load_positions =
        generate_random_string_vector(generate_random_size());
      msg.load_sets = generate_random_vector<LoadSet>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, MaxArrayLens>)
    {
      msg.order_nodes = generate_random_uint();
      msg.order_edges = generate_random_uint();
      msg.node_actions = generate_random_uint();
      msg.edge_actions = generate_random_uint();
      msg.actions_actions_parameters = generate_random_uint();
      msg.instant_actions = generate_random_uint();
      msg.trajectory_knot_vector = generate_random_uint();
      msg.trajectory_control_points = generate_random_uint();
      msg.state_node_states = generate_random_uint();
      msg.state_edge_states = generate_random_uint();
      msg.state_loads = generate_random_uint();
      msg.state_action_states = generate_random_uint();
      msg.state_errors = generate_random_uint();
      msg.state_information = generate_random_uint();
      msg.error_error_references = generate_random_uint();
      msg.information_info_references = generate_random_uint();
    }
    else if constexpr (std::is_same_v<T, MaxStringLens>)
    {
      msg.msg_len = generate_random_uint();
      msg.topic_serial_len = generate_random_uint();
      msg.topic_elem_len = generate_random_uint();
      msg.id_len = generate_random_uint();
      msg.enum_len = generate_random_uint();
      msg.load_id_len = generate_random_uint();
      msg.id_numerical_only = generate_random_bool();
    }
    else if constexpr (std::is_same_v<T, Node>)
    {
      msg.node_id = generate_random_string();
      msg.sequence_id = generate_random_uint();
      msg.released = generate_random_bool();
      msg.actions = generate_random_vector<Action>(generate_random_size());
      msg.node_position = generate<NodePosition>();
      msg.node_description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, NodePosition>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.theta = generate_random_float();
      msg.allowed_deviation_x_y = generate_random_float();
      msg.allowed_deviation_theta = generate_random_float();
      msg.map_id = generate_random_string();
      msg.map_description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, NodeState>)
    {
      msg.node_id = generate_random_string();
      msg.sequence_id = generate_random_uint();
      msg.node_description = generate_random_string();
      msg.node_position = generate<NodePosition>();
      msg.released = generate_random_bool();
    }
    else if constexpr (std::is_same_v<T, OptionalParameter>)
    {
      msg.parameter = generate_random_string();
      msg.support = generate_random_support();
      msg.description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Order>)
    {
      msg.header = generate<Header>();
      msg.order_id = generate_random_string();
      msg.order_update_id = generate_random_uint();
      msg.nodes = generate_random_vector<Node>(generate_random_size());
      msg.edges = generate_random_vector<Edge>(generate_random_size());
      msg.zone_set_id = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, PhysicalParameters>)
    {
      msg.speed_min = generate_random_float();
      msg.speed_max = generate_random_float();
      msg.acceleration_max = generate_random_float();
      msg.deceleration_max = generate_random_float();
      msg.height_min = generate_random_float();
      msg.height_max = generate_random_float();
      msg.width = generate_random_float();
      msg.length = generate_random_float();
      msg.angular_speed_min = generate_random_float();
      msg.angular_speed_max = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, PolygonPoint>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, Position>)
    {
      msg.x = generate_random_float();
      msg.y = generate_random_float();
      msg.theta = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, ProtocolFeatures>)
    {
      msg.optional_parameters =
        generate_random_vector<OptionalParameter>(generate_random_size());
      msg.agv_actions =
        generate_random_vector<AGVAction>(generate_random_size());
    }
    else if constexpr (std::is_same_v<T, ProtocolLimits>)
    {
      msg.max_string_lens = generate<MaxStringLens>();
      msg.max_array_lens = generate<MaxArrayLens>();
      msg.timing = generate<Timing>();
    }
    else if constexpr (std::is_same_v<T, SafetyState>)
    {
      msg.e_stop = generate_random_e_stop();
      msg.field_violation = generate_random_bool();
    }
    else if constexpr (std::is_same_v<T, State>)
    {
      msg.header = generate<Header>();
      msg.order_id = generate_random_string();
      msg.order_update_id = generate_random_uint();
      msg.zone_set_id = generate_random_string();
      msg.last_node_id = generate_random_string();
      msg.last_node_sequence_id = generate_random_uint();
      msg.driving = generate_random_bool();
      msg.paused = generate_random_bool();
      msg.new_base_request = generate_random_bool();
      msg.distance_since_last_node = generate_random_float();
      msg.operating_mode = generate_random_operating_mode();
      msg.node_states =
        generate_random_vector<NodeState>(generate_random_size());
      msg.edge_states =
        generate_random_vector<EdgeState>(generate_random_size());
      msg.agv_position = generate<AGVPosition>();
      msg.velocity = generate<Velocity>();
      msg.loads = generate_random_vector<Load>(generate_random_size());
      msg.action_states =
        generate_random_vector<ActionState>(generate_random_size());
      msg.battery_state = generate<BatteryState>();
      msg.errors = generate_random_vector<Error>(generate_random_size());
      msg.information = generate_random_vector<Info>(generate_random_size());
      msg.safety_state = generate<SafetyState>();
    }
    else if constexpr (std::is_same_v<T, Timing>)
    {
      msg.min_order_interval = generate_random_float();
      msg.min_state_interval = generate_random_float();
      msg.default_state_interval = generate_random_float();
      msg.visualization_interval = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, Trajectory>)
    {
      msg.knot_vector = generate_random_float_vector(generate_random_size());
      msg.control_points =
        generate_random_vector<ControlPoint>(generate_random_size());
      msg.degree = 1.0;
    }
    else if constexpr (std::is_same_v<T, TypeSpecification>)
    {
      msg.series_name = generate_random_string();
      msg.agv_kinematic = generate_random_agv_kinematic();
      msg.agv_class = generate_random_agv_class();
      msg.max_load_mass = generate_random_float();
      msg.localization_types =
        generate_random_string_vector(generate_random_size());
      msg.navigation_types =
        generate_random_string_vector(generate_random_size());
      msg.series_description = generate_random_string();
    }
    else if constexpr (std::is_same_v<T, Velocity>)
    {
      msg.vx = generate_random_float();
      msg.vy = generate_random_float();
      msg.omega = generate_random_float();
    }
    else if constexpr (std::is_same_v<T, Visualization>)
    {
      msg.header = generate<Header>();
      msg.agv_position = generate<AGVPosition>();
      msg.velocity = generate<Velocity>();
    }
    else if constexpr (std::is_same_v<T, WheelDefinition>)
    {
      msg.type = generate_random_wheel_definition_type();
      msg.is_active_driven = generate_random_bool();
      msg.is_active_steered = generate_random_bool();
      msg.position = generate<Position>();
      msg.diameter = generate_random_float();
      msg.width = generate_random_float();
      msg.center_displacement = generate_random_float();
      msg.constraints = generate_random_string();
    }
    else
    {
      throw std::runtime_error(
        "No random data generator defined for this custom type: " +
        std::string(typeid(T).name()));
    }
    return msg;
  }

private:
  /// \brief Mersenne Twister random number engine
  std::mt19937 rng_;

  /// \brief Distribution for unsigned 32-bit integers
  std::uniform_int_distribution<uint32_t> uint_dist_;

  /// \brief Distribution for signed 8-bit integers (int-typed; char-sized
  /// distribution types are not permitted by the standard / MSVC)
  std::uniform_int_distribution<int> int_dist_;

  /// \brief Distribution for 64-bit floating-point numbers
  std::uniform_real_distribution<double> float_dist_;

  /// \brief Distribution for a boolean value
  std::uniform_int_distribution<int> bool_dist_;

  /// \brief Distribution for random string lengths
  std::uniform_int_distribution<int> string_length_dist_;

  /// \brief Distribution for random vector size (int-typed; char-sized
  /// distribution types are not permitted by the standard / MSVC)
  std::uniform_int_distribution<int> size_dist_;

  /// \brief Distribution for random percentage values
  std::uniform_real_distribution<double> percentage_dist_;
};

#endif  // JSON_UTILS__GENERATOR__GENERATOR_HPP_
