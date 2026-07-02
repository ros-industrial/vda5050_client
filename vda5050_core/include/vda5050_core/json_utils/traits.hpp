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

#ifndef VDA5050_CORE__JSON_UTILS__TRAITS_HPP_
#define VDA5050_CORE__JSON_UTILS__TRAITS_HPP_

#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/compat/time_utils.hpp"
#include "vda5050_core/types/action_scope.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_class.hpp"
#include "vda5050_core/types/agv_kinematic.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/e_stop.hpp"
#include "vda5050_core/types/error_level.hpp"
#include "vda5050_core/types/header.hpp"
#include "vda5050_core/types/info_level.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/orientation_type.hpp"
#include "vda5050_core/types/support.hpp"
#include "vda5050_core/types/value_data_type.hpp"
#include "vda5050_core/types/wheel_definition_type.hpp"

#ifdef ENABLE_ROS2
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <vda5050_interfaces/msg/action.hpp>
#include <vda5050_interfaces/msg/action_parameter_factsheet.hpp>
#include <vda5050_interfaces/msg/action_state.hpp>
#include <vda5050_interfaces/msg/agv_action.hpp>
#include <vda5050_interfaces/msg/blocking_type.hpp>
#include <vda5050_interfaces/msg/connection.hpp>
#include <vda5050_interfaces/msg/edge.hpp>
#include <vda5050_interfaces/msg/error.hpp>
#include <vda5050_interfaces/msg/info.hpp>
#include <vda5050_interfaces/msg/optional_parameter.hpp>
#include <vda5050_interfaces/msg/safety_state.hpp>
#include <vda5050_interfaces/msg/state.hpp>
#include <vda5050_interfaces/msg/type_specification.hpp>
#include <vda5050_interfaces/msg/wheel_definition.hpp>
#endif  // ENABLE_ROS2

namespace vda5050_core {

namespace json_utils {

//=============================================================================
template <typename T>
struct optional_field_traits;

//=============================================================================
template <typename T>
struct optional_field_traits<std::optional<T>>
{
  using value_type = T;

  static bool has_value(const std::optional<T>& opt)
  {
    return opt.has_value();
  }

  static const T get(const std::optional<T>& opt)
  {
    return opt.value();
  }

  template <typename U>
  static void set(std::optional<T>& opt, U&& val)
  {
    opt = std::forward<U>(val);
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <typename T, typename Alloc>
struct optional_field_traits<rosidl_runtime_cpp::BoundedVector<T, 1, Alloc>>
{
  using value_type = T;

  static bool has_value(
    const rosidl_runtime_cpp::BoundedVector<T, 1, Alloc>& opt)
  {
    return !opt.empty();
  }

  static const T get(const rosidl_runtime_cpp::BoundedVector<T, 1, Alloc>& opt)
  {
    return opt.front();
  }

  template <typename U>
  static void set(rosidl_runtime_cpp::BoundedVector<T, 1, Alloc>& opt, U&& val)
  {
    opt.clear();
    opt.push_back(std::forward<U>(val));
  }
};

//=============================================================================
template <typename T, std::size_t Max, typename Alloc>
struct optional_field_traits<rosidl_runtime_cpp::BoundedVector<T, Max, Alloc>>
{
  using value_type = rosidl_runtime_cpp::BoundedVector<T, Max, Alloc>;

  static bool has_value(
    const rosidl_runtime_cpp::BoundedVector<T, Max, Alloc>& opt)
  {
    return !opt.empty();
  }

  static const rosidl_runtime_cpp::BoundedVector<T, Max, Alloc> get(
    const rosidl_runtime_cpp::BoundedVector<T, Max, Alloc>& opt)
  {
    return opt;
  }

  template <typename U>
  static void set(
    rosidl_runtime_cpp::BoundedVector<T, Max, Alloc>& opt, U&& val)
  {
    opt = std::forward<U>(val);
  }
};

//=============================================================================
template <typename T, typename Alloc>
struct optional_field_traits<std::vector<T, Alloc>>
{
  using value_type = std::vector<T, Alloc>;

  static bool has_value(const std::vector<T, Alloc>& opt)
  {
    return !opt.empty();
  }

  static const std::vector<T> get(const std::vector<T>& opt)
  {
    return opt;
  }

  template <typename U>
  static void set(std::vector<T, Alloc>& opt, U&& val)
  {
    opt = std::forward<U>(val);
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

//=============================================================================
inline std::string to_iso8601(TimePoint time_point)
{
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;
  using std::chrono::system_clock;

  std::time_t time_sec = system_clock::to_time_t(time_point);
  auto duration = time_point.time_since_epoch();
  auto millisec = duration_cast<milliseconds>(duration).count() % 1000;

  std::tm tm_buf = compat::to_gmtime(time_sec);

  std::ostringstream oss;
  oss << std::put_time(&tm_buf, types::ISO8601_FORMAT);
  oss << "." << std::setw(3) << std::setfill('0') << millisec << "Z";
  if (oss.fail())
  {
    throw std::runtime_error("Failed to format timestamp for serialization.");
  }
  return oss.str();
}

//=============================================================================
inline TimePoint from_iso8601(const std::string& time_string)
{
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;
  using std::chrono::system_clock;

  std::tm t = {};
  char sep;
  int millisec = 0;

  std::istringstream ss(time_string);
  ss >> std::get_time(&t, types::ISO8601_FORMAT);

  ss >> sep;
  if (ss.fail() || sep != '.')
  {
    throw std::runtime_error(
      "JSON parsing error for Header: Unexpected character after seconds in "
      "timestamp.");
  }
  else
  {
    ss >> millisec;
    if (ss.fail())
    {
      throw std::runtime_error(
        "JSON parsing error for Header: Failed to parse milliseconds from "
        "timestamp.");
    }

    if (!ss.eof())
    {
      ss.ignore(std::numeric_limits<std::streamsize>::max(), 'Z');
    }
    else
    {
      throw std::runtime_error(
        "JSON parsing error for Header: Expected 'Z' at the end of timestamp "
        "to indicate UTC.");
    }
  }

  auto tp = system_clock::from_time_t(compat::timegm_utc(&t));

  return tp + milliseconds(millisec);
}

//=============================================================================
template <typename T>
struct timestamp_traits;

//=============================================================================
template <>
struct timestamp_traits<TimePoint>
{
  static std::string to_string(const TimePoint& time_point)
  {
    return to_iso8601(time_point);
  }

  static TimePoint from_string(const std::string& time_string)
  {
    return from_iso8601(time_string);
  }
};

//=============================================================================
template <>
struct timestamp_traits<int64_t>
{
  static std::string to_string(const int64_t& millisec)
  {
    using std::chrono::milliseconds;

    TimePoint time_point{milliseconds(millisec)};
    return to_iso8601(time_point);
  }

  static int64_t from_string(const std::string& time_string)
  {
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;

    auto time_point = from_iso8601(time_string);
    return duration_cast<milliseconds>(time_point.time_since_epoch()).count();
  }
};

//=============================================================================
template <typename T>
struct connection_state_traits;

//=============================================================================
template <>
struct connection_state_traits<types::ConnectionState>
{
  static std::string to_string(const types::ConnectionState& state)
  {
    using types::ConnectionState;

    switch (state)
    {
      case ConnectionState::ONLINE:
        return "ONLINE";
      case ConnectionState::OFFLINE:
        return "OFFLINE";
      case ConnectionState::CONNECTIONBROKEN:
        return "CONNECTIONBROKEN";
      default:
        throw std::runtime_error("Invalid ConnectionState enum value");
    }
  }

  static types::ConnectionState from_string(const std::string& state)
  {
    using types::ConnectionState;

    if (state == "ONLINE") return ConnectionState::ONLINE;
    if (state == "OFFLINE") return ConnectionState::OFFLINE;
    if (state == "CONNECTIONBROKEN") return ConnectionState::CONNECTIONBROKEN;
    throw std::runtime_error("Invalid connectionState string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct connection_state_traits<std::string>
{
  static std::string to_string(const std::string& state)
  {
    using vda5050_interfaces::msg::Connection;

    if (
      state == Connection::ONLINE || state == Connection::OFFLINE ||
      state == Connection::CONNECTIONBROKEN)
    {
      return state;
    }
    throw std::runtime_error("Invalid connection_state value");
  }

  static std::string from_string(const std::string& state)
  {
    using vda5050_interfaces::msg::Connection;

    if (
      state == Connection::ONLINE || state == Connection::OFFLINE ||
      state == Connection::CONNECTIONBROKEN)
    {
      return state;
    }
    throw std::runtime_error("Invalid connectionState string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct operating_mode_traits;

//=============================================================================
template <>
struct operating_mode_traits<types::OperatingMode>
{
  static std::string to_string(const types::OperatingMode& mode)
  {
    using types::OperatingMode;

    switch (mode)
    {
      case OperatingMode::AUTOMATIC:
        return "AUTOMATIC";
      case OperatingMode::SEMIAUTOMATIC:
        return "SEMIAUTOMATIC";
      case OperatingMode::MANUAL:
        return "MANUAL";
      case OperatingMode::SERVICE:
        return "SERVICE";
      case OperatingMode::TEACHIN:
        return "TEACHIN";
      default:
        throw std::runtime_error("Invalid OperatingMode enum value");
    }
  }

  static types::OperatingMode from_string(const std::string& mode)
  {
    using types::OperatingMode;

    if (mode == "AUTOMATIC") return OperatingMode::AUTOMATIC;
    if (mode == "SEMIAUTOMATIC") return OperatingMode::SEMIAUTOMATIC;
    if (mode == "MANUAL") return OperatingMode::MANUAL;
    if (mode == "SERVICE") return OperatingMode::SERVICE;
    if (mode == "TEACHIN") return OperatingMode::TEACHIN;
    throw std::runtime_error("Invalid operatingMode string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct operating_mode_traits<std::string>
{
  static std::string to_string(const std::string& mode)
  {
    using vda5050_interfaces::msg::State;

    if (
      mode == State::OPERATING_MODE_AUTOMATIC ||
      mode == State::OPERATING_MODE_SEMIAUTOMATIC ||
      mode == State::OPERATING_MODE_MANUAL ||
      mode == State::OPERATING_MODE_SERVICE ||
      mode == State::OPERATING_MODE_TEACHIN)
    {
      return mode;
    }
    throw std::runtime_error("Invalid operating_mode value");
  }

  static std::string from_string(const std::string& mode)
  {
    using vda5050_interfaces::msg::State;

    if (
      mode == State::OPERATING_MODE_AUTOMATIC ||
      mode == State::OPERATING_MODE_SEMIAUTOMATIC ||
      mode == State::OPERATING_MODE_MANUAL ||
      mode == State::OPERATING_MODE_SERVICE ||
      mode == State::OPERATING_MODE_TEACHIN)
    {
      return mode;
    }
    throw std::runtime_error("Invalid operatingMode string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct action_status_traits;

//=============================================================================
template <>
struct action_status_traits<types::ActionStatus>
{
  static std::string to_string(const types::ActionStatus& status)
  {
    using types::ActionStatus;

    switch (status)
    {
      case ActionStatus::WAITING:
        return "WAITING";
      case ActionStatus::INITIALIZING:
        return "INITIALIZING";
      case ActionStatus::RUNNING:
        return "RUNNING";
      case ActionStatus::PAUSED:
        return "PAUSED";
      case ActionStatus::FINISHED:
        return "FINISHED";
      case ActionStatus::FAILED:
        return "FAILED";
      default:
        throw std::runtime_error("Invalid ActionStatus enum value");
    }
  }

  static types::ActionStatus from_string(const std::string& status)
  {
    using types::ActionStatus;

    if (status == "WAITING") return ActionStatus::WAITING;
    if (status == "INITIALIZING") return ActionStatus::INITIALIZING;
    if (status == "RUNNING") return ActionStatus::RUNNING;
    if (status == "PAUSED") return ActionStatus::PAUSED;
    if (status == "FINISHED") return ActionStatus::FINISHED;
    if (status == "FAILED") return ActionStatus::FAILED;
    throw std::runtime_error("Invalid actionStatus string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct action_status_traits<std::string>
{
  static std::string to_string(const std::string& status)
  {
    using vda5050_interfaces::msg::ActionState;

    if (
      status == ActionState::ACTION_STATUS_WAITING ||
      status == ActionState::ACTION_STATUS_INITIALIZING ||
      status == ActionState::ACTION_STATUS_RUNNING ||
      status == ActionState::ACTION_STATUS_PAUSED ||
      status == ActionState::ACTION_STATUS_FINISHED ||
      status == ActionState::ACTION_STATUS_FAILED)
    {
      return status;
    }
    throw std::runtime_error("Invalid action_status value");
  }

  static std::string from_string(const std::string& status)
  {
    using vda5050_interfaces::msg::ActionState;

    if (
      status == ActionState::ACTION_STATUS_WAITING ||
      status == ActionState::ACTION_STATUS_INITIALIZING ||
      status == ActionState::ACTION_STATUS_RUNNING ||
      status == ActionState::ACTION_STATUS_PAUSED ||
      status == ActionState::ACTION_STATUS_FINISHED ||
      status == ActionState::ACTION_STATUS_FAILED)
    {
      return status;
    }
    throw std::runtime_error("Invalid actionStatus string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct error_level_traits;

//=============================================================================
template <>
struct error_level_traits<types::ErrorLevel>
{
  static std::string to_string(const types::ErrorLevel& level)
  {
    using types::ErrorLevel;

    switch (level)
    {
      case ErrorLevel::WARNING:
        return "WARNING";
      case ErrorLevel::FATAL:
        return "FATAL";
      default:
        throw std::runtime_error("Invalid ErrorLevel enum value");
    }
  }

  static types::ErrorLevel from_string(const std::string& level)
  {
    using types::ErrorLevel;

    if (level == "WARNING") return ErrorLevel::WARNING;
    if (level == "FATAL") return ErrorLevel::FATAL;
    throw std::runtime_error("Invalid errorLevel string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct error_level_traits<std::string>
{
  static std::string to_string(const std::string& level)
  {
    using vda5050_interfaces::msg::Error;

    if (
      level == Error::ERROR_LEVEL_WARNING || level == Error::ERROR_LEVEL_FATAL)
    {
      return level;
    }
    throw std::runtime_error("Invalid error_level value");
  }

  static std::string from_string(const std::string& level)
  {
    using vda5050_interfaces::msg::Error;

    if (
      level == Error::ERROR_LEVEL_WARNING || level == Error::ERROR_LEVEL_FATAL)
    {
      return level;
    }
    throw std::runtime_error("Invalid errorLevel string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct e_stop_traits;

//=============================================================================
template <>
struct e_stop_traits<types::EStop>
{
  static std::string to_string(const types::EStop& type)
  {
    using types::EStop;

    switch (type)
    {
      case EStop::AUTOACK:
        return "AUTOACK";
      case EStop::MANUAL:
        return "MANUAL";
      case EStop::REMOTE:
        return "REMOTE";
      case EStop::NONE:
        return "NONE";
      default:
        throw std::runtime_error("Invalid EStop enum value");
    }
  }

  static types::EStop from_string(const std::string& type)
  {
    using types::EStop;

    if (type == "AUTOACK") return EStop::AUTOACK;
    if (type == "MANUAL") return EStop::MANUAL;
    if (type == "REMOTE") return EStop::REMOTE;
    if (type == "NONE") return EStop::NONE;
    throw std::runtime_error("Invalid eStop string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct e_stop_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::SafetyState;

    if (
      type == SafetyState::E_STOP_AUTOACK ||
      type == SafetyState::E_STOP_MANUAL ||
      type == SafetyState::E_STOP_REMOTE || type == SafetyState::E_STOP_NONE)
    {
      return type;
    }
    throw std::runtime_error("Invalid e_stop value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::SafetyState;

    if (
      type == SafetyState::E_STOP_AUTOACK ||
      type == SafetyState::E_STOP_MANUAL ||
      type == SafetyState::E_STOP_REMOTE || type == SafetyState::E_STOP_NONE)
    {
      return type;
    }
    throw std::runtime_error("Invalid eStop string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct info_level_traits;

//=============================================================================
template <>
struct info_level_traits<types::InfoLevel>
{
  static std::string to_string(const types::InfoLevel& level)
  {
    using types::InfoLevel;

    switch (level)
    {
      case InfoLevel::INFO:
        return "INFO";
      case InfoLevel::DEBUG:
        return "DEBUG";
      default:
        throw std::runtime_error("Invalid InfoLevel enum value");
    }
  }

  static types::InfoLevel from_string(const std::string& level)
  {
    using types::InfoLevel;

    if (level == "INFO") return InfoLevel::INFO;
    if (level == "DEBUG") return InfoLevel::DEBUG;
    throw std::runtime_error("Invalid infoLevel string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct info_level_traits<std::string>
{
  static std::string to_string(const std::string& level)
  {
    using vda5050_interfaces::msg::Info;

    if (level == Info::INFO_LEVEL_INFO || level == Info::INFO_LEVEL_DEBUG)
    {
      return level;
    }
    throw std::runtime_error("Invalid info_level value");
  }

  static std::string from_string(const std::string& level)
  {
    using vda5050_interfaces::msg::Info;

    if (level == Info::INFO_LEVEL_INFO || level == Info::INFO_LEVEL_DEBUG)
    {
      return level;
    }
    throw std::runtime_error("Invalid infoLevel string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct blocking_type_traits;

//=============================================================================
template <>
struct blocking_type_traits<types::BlockingType>
{
  static std::string to_string(const types::BlockingType& type)
  {
    using types::BlockingType;

    switch (type)
    {
      case BlockingType::NONE:
        return "NONE";
      case BlockingType::SOFT:
        return "SOFT";
      case BlockingType::HARD:
        return "HARD";
      default:
        throw std::runtime_error("Invalid BlockingType enum value");
    }
  }

  static types::BlockingType from_string(const std::string& type)
  {
    using types::BlockingType;

    if (type == "NONE") return BlockingType::NONE;
    if (type == "SOFT") return BlockingType::SOFT;
    if (type == "HARD") return BlockingType::HARD;
    throw std::runtime_error("Invalid blockingType string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct blocking_type_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::BlockingType;

    if (
      type == BlockingType::BLOCKING_TYPE_NONE ||
      type == BlockingType::BLOCKING_TYPE_SOFT ||
      type == BlockingType::BLOCKING_TYPE_HARD)
    {
      return type;
    }
    throw std::runtime_error("Invalid blocking_type value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::BlockingType;

    if (
      type == BlockingType::BLOCKING_TYPE_NONE ||
      type == BlockingType::BLOCKING_TYPE_SOFT ||
      type == BlockingType::BLOCKING_TYPE_HARD)
    {
      return type;
    }
    throw std::runtime_error("Invalid blockingType string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct orientation_type_traits;

//=============================================================================
template <>
struct orientation_type_traits<types::OrientationType>
{
  static std::string to_string(const types::OrientationType& type)
  {
    using types::OrientationType;

    switch (type)
    {
      case OrientationType::GLOBAL:
        return "GLOBAL";
      case OrientationType::TANGENTIAL:
        return "TANGENTIAL";
      default:
        throw std::runtime_error("Invalid OrientationType enum value");
    }
  }

  static types::OrientationType from_string(const std::string& type)
  {
    using types::OrientationType;

    if (type == "GLOBAL") return OrientationType::GLOBAL;
    if (type == "TANGENTIAL") return OrientationType::TANGENTIAL;
    throw std::runtime_error("Invalid orientationType string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct orientation_type_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::Edge;

    if (
      type == Edge::ORIENTATION_TYPE_TANGENTIAL ||
      type == Edge::ORIENTATION_TYPE_GLOBAL)
    {
      return type;
    }
    throw std::runtime_error("Invalid orientation_type value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::Edge;

    if (
      type == Edge::ORIENTATION_TYPE_TANGENTIAL ||
      type == Edge::ORIENTATION_TYPE_GLOBAL)
    {
      return type;
    }
    throw std::runtime_error("Invalid orientationType string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct agv_kinematic_traits;

//=============================================================================
template <>
struct agv_kinematic_traits<types::AGVKinematic>
{
  static std::string to_string(const types::AGVKinematic& type)
  {
    using types::AGVKinematic;

    switch (type)
    {
      case AGVKinematic::DIFF:
        return "DIFF";
      case AGVKinematic::OMNI:
        return "OMNI";
      case AGVKinematic::THREEWHEEL:
        return "THREEWHEEL";
      default:
        throw std::runtime_error("Invalid AGVKinematic enum value");
    }
  }

  static types::AGVKinematic from_string(const std::string& type)
  {
    using types::AGVKinematic;

    if (type == "DIFF") return AGVKinematic::DIFF;
    if (type == "OMNI") return AGVKinematic::OMNI;
    if (type == "THREEWHEEL") return AGVKinematic::THREEWHEEL;
    throw std::runtime_error("Invalid agvKinematic string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct agv_kinematic_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::TypeSpecification;

    if (
      type == TypeSpecification::AGV_KINEMATIC_DIFF ||
      type == TypeSpecification::AGV_KINEMATIC_OMNI ||
      type == TypeSpecification::AGV_KINEMATIC_THREEWHEEL)
    {
      return type;
    }
    throw std::runtime_error("Invalid agv_kinematic value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::TypeSpecification;

    if (
      type == TypeSpecification::AGV_KINEMATIC_DIFF ||
      type == TypeSpecification::AGV_KINEMATIC_OMNI ||
      type == TypeSpecification::AGV_KINEMATIC_THREEWHEEL)
    {
      return type;
    }
    throw std::runtime_error("Invalid agvKinematic string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct agv_class_traits;

//=============================================================================
template <>
struct agv_class_traits<types::AGVClass>
{
  static std::string to_string(const types::AGVClass& type)
  {
    using types::AGVClass;

    switch (type)
    {
      case AGVClass::FORKLIFT:
        return "FORKLIFT";
      case AGVClass::CONVEYOR:
        return "CONVEYOR";
      case AGVClass::TUGGER:
        return "TUGGER";
      case AGVClass::CARRIER:
        return "CARRIER";
      default:
        throw std::runtime_error("Invalid AGVClass enum value");
    }
  }

  static types::AGVClass from_string(const std::string& type)
  {
    using types::AGVClass;

    if (type == "FORKLIFT") return AGVClass::FORKLIFT;
    if (type == "CONVEYOR") return AGVClass::CONVEYOR;
    if (type == "TUGGER") return AGVClass::TUGGER;
    if (type == "CARRIER") return AGVClass::CARRIER;
    throw std::runtime_error("Invalid agvClass string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct agv_class_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::TypeSpecification;

    if (
      type == TypeSpecification::AGV_CLASS_FORKLIFT ||
      type == TypeSpecification::AGV_CLASS_CONVEYOR ||
      type == TypeSpecification::AGV_CLASS_TUGGER ||
      type == TypeSpecification::AGV_CLASS_CARRIER)
    {
      return type;
    }
    throw std::runtime_error("Invalid agv_class value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::TypeSpecification;

    if (
      type == TypeSpecification::AGV_CLASS_FORKLIFT ||
      type == TypeSpecification::AGV_CLASS_CONVEYOR ||
      type == TypeSpecification::AGV_CLASS_TUGGER ||
      type == TypeSpecification::AGV_CLASS_CARRIER)
    {
      return type;
    }
    throw std::runtime_error("Invalid agvClass string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct support_traits;

//=============================================================================
template <>
struct support_traits<types::Support>
{
  static std::string to_string(const types::Support& type)
  {
    using types::Support;

    switch (type)
    {
      case Support::SUPPORTED:
        return "SUPPORTED";
      case Support::REQUIRED:
        return "REQUIRED";
      default:
        throw std::runtime_error("Invalid Support enum value");
    }
  }

  static types::Support from_string(const std::string& type)
  {
    using types::Support;

    if (type == "SUPPORTED") return Support::SUPPORTED;
    if (type == "REQUIRED") return Support::REQUIRED;
    throw std::runtime_error("Invalid support string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct support_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::OptionalParameter;

    if (
      type == OptionalParameter::SUPPORT_SUPPORTED ||
      type == OptionalParameter::SUPPORT_REQUIRED)
    {
      return type;
    }
    throw std::runtime_error("Invalid support value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::OptionalParameter;

    if (
      type == OptionalParameter::SUPPORT_SUPPORTED ||
      type == OptionalParameter::SUPPORT_REQUIRED)
    {
      return type;
    }
    throw std::runtime_error("Invalid support string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct action_scopes_traits;

//=============================================================================
template <>
struct action_scopes_traits<std::vector<types::ActionScope>>
{
  static std::vector<std::string> to_string(
    const std::vector<types::ActionScope>& types)
  {
    using types::ActionScope;

    std::vector<std::string> output;
    for (const auto& type : types)
    {
      switch (type)
      {
        case ActionScope::INSTANT:
          output.push_back("INSTANT");
          break;
        case ActionScope::NODE:
          output.push_back("NODE");
          break;
        case ActionScope::EDGE:
          output.push_back("EDGE");
          break;
        default:
          throw std::runtime_error("Invalid ActionScope enum value");
      }
    }
    return output;
  }

  static std::vector<types::ActionScope> from_string(
    const std::vector<std::string>& types)
  {
    using types::ActionScope;

    std::vector<types::ActionScope> output;
    for (const auto& type : types)
    {
      if (type == "INSTANT")
        output.push_back(ActionScope::INSTANT);
      else if (type == "NODE")
        output.push_back(ActionScope::NODE);
      else if (type == "EDGE")
        output.push_back(ActionScope::EDGE);
      else
        throw std::runtime_error("Invalid actionScope string");
    }
    return output;
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct action_scopes_traits<std::vector<std::string>>
{
  static std::vector<std::string> to_string(
    const std::vector<std::string>& types)
  {
    using vda5050_interfaces::msg::AGVAction;

    for (const auto& type : types)
    {
      if (
        type != AGVAction::ACTION_SCOPE_INSTANT &&
        type != AGVAction::ACTION_SCOPE_NODE &&
        type != AGVAction::ACTION_SCOPE_EDGE)
      {
        throw std::runtime_error("Invalid action_scopes value");
      }
    }
    return types;
  }

  static std::vector<std::string> from_string(
    const std::vector<std::string>& types)
  {
    using vda5050_interfaces::msg::AGVAction;

    for (const auto& type : types)
    {
      if (
        type != AGVAction::ACTION_SCOPE_INSTANT &&
        type != AGVAction::ACTION_SCOPE_NODE &&
        type != AGVAction::ACTION_SCOPE_EDGE)
      {
        throw std::runtime_error("Invalid actionScopes value");
      }
    }
    return types;
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct value_data_type_traits;

//=============================================================================
template <>
struct value_data_type_traits<types::ValueDataType>
{
  static std::string to_string(const types::ValueDataType& type)
  {
    using types::ValueDataType;

    switch (type)
    {
      case ValueDataType::BOOL:
        return "BOOL";
      case ValueDataType::NUMBER:
        return "NUMBER";
      case ValueDataType::INTEGER:
        return "INTEGER";
      case ValueDataType::FLOAT:
        return "FLOAT";
      case ValueDataType::STRING:
        return "STRING";
      case ValueDataType::OBJECT:
        return "OBJECT";
      case ValueDataType::ARRAY:
        return "ARRAY";
      default:
        throw std::runtime_error("Invalid ValueDataType enum value");
    }
  }

  static types::ValueDataType from_string(const std::string& type)
  {
    using types::ValueDataType;

    if (type == "BOOL") return ValueDataType::BOOL;
    if (type == "NUMBER") return ValueDataType::NUMBER;
    if (type == "INTEGER") return ValueDataType::INTEGER;
    if (type == "FLOAT") return ValueDataType::FLOAT;
    if (type == "STRING") return ValueDataType::STRING;
    if (type == "OBJECT") return ValueDataType::OBJECT;
    if (type == "ARRAY") return ValueDataType::ARRAY;
    throw std::runtime_error("Invalid valueDataType string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct value_data_type_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::ActionParameterFactsheet;

    if (
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_BOOL ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_NUMBER ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_INTEGER ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_FLOAT ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_STRING ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_OBJECT ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_ARRAY)
    {
      return type;
    }
    throw std::runtime_error("Invalid value_data_type value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::ActionParameterFactsheet;

    if (
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_BOOL ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_NUMBER ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_INTEGER ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_FLOAT ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_STRING ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_OBJECT ||
      type == ActionParameterFactsheet::VALUE_DATA_TYPE_ARRAY)
    {
      return type;
    }
    throw std::runtime_error("Invalid valueDataType string");
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct blocking_types_traits;

//=============================================================================
template <>
struct blocking_types_traits<std::vector<types::BlockingType>>
{
  static std::vector<std::string> to_string(
    const std::vector<types::BlockingType>& types)
  {
    using types::BlockingType;

    std::vector<std::string> output;
    for (const auto& type : types)
    {
      switch (type)
      {
        case BlockingType::NONE:
          output.push_back("NONE");
          break;
        case BlockingType::SOFT:
          output.push_back("SOFT");
          break;
        case BlockingType::HARD:
          output.push_back("HARD");
          break;
        default:
          throw std::runtime_error("Invalid BlockingType enum value");
      }
    }
    return output;
  }

  static std::vector<types::BlockingType> from_string(
    const std::vector<std::string>& types)
  {
    using types::BlockingType;

    std::vector<types::BlockingType> output;
    for (const auto& type : types)
    {
      if (type == "NONE")
        output.push_back(BlockingType::NONE);
      else if (type == "SOFT")
        output.push_back(BlockingType::SOFT);
      else if (type == "HARD")
        output.push_back(BlockingType::HARD);
      else
        throw std::runtime_error("Invalid blockingType string");
    }
    return output;
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct blocking_types_traits<std::vector<std::string>>
{
  static std::vector<std::string> to_string(
    const std::vector<std::string>& types)
  {
    using vda5050_interfaces::msg::BlockingType;

    for (const auto& type : types)
    {
      if (
        type != BlockingType::BLOCKING_TYPE_NONE &&
        type != BlockingType::BLOCKING_TYPE_SOFT &&
        type != BlockingType::BLOCKING_TYPE_HARD)
      {
        throw std::runtime_error("Invalid blocking_types value");
      }
    }
    return types;
  }

  static std::vector<std::string> from_string(
    const std::vector<std::string>& types)
  {
    using vda5050_interfaces::msg::BlockingType;

    for (const auto& type : types)
    {
      if (
        type != BlockingType::BLOCKING_TYPE_NONE &&
        type != BlockingType::BLOCKING_TYPE_SOFT &&
        type != BlockingType::BLOCKING_TYPE_HARD)
      {
        throw std::runtime_error("Invalid blockingTypes value");
      }
    }
    return types;
  }
};
#endif  // ENABLE_ROS2

//=============================================================================
template <typename T>
struct wheel_definition_type_traits;

//=============================================================================
template <>
struct wheel_definition_type_traits<types::WheelDefinitionType>
{
  static std::string to_string(const types::WheelDefinitionType& type)
  {
    using types::WheelDefinitionType;

    switch (type)
    {
      case WheelDefinitionType::DRIVE:
        return "DRIVE";
      case WheelDefinitionType::CASTER:
        return "CASTER";
      case WheelDefinitionType::FIXED:
        return "FIXED";
      case WheelDefinitionType::MECANUM:
        return "MECANUM";
      default:
        throw std::runtime_error("Invalid WheelDefinitionType enum value");
    }
  }

  static types::WheelDefinitionType from_string(const std::string& type)
  {
    using types::WheelDefinitionType;

    if (type == "DRIVE") return WheelDefinitionType::DRIVE;
    if (type == "CASTER") return WheelDefinitionType::CASTER;
    if (type == "FIXED") return WheelDefinitionType::FIXED;
    if (type == "MECANUM") return WheelDefinitionType::MECANUM;
    throw std::runtime_error("Invalid wheelDefinitionType string");
  }
};

//=============================================================================
#ifdef ENABLE_ROS2
template <>
struct wheel_definition_type_traits<std::string>
{
  static std::string to_string(const std::string& type)
  {
    using vda5050_interfaces::msg::WheelDefinition;

    if (
      type == WheelDefinition::TYPE_DRIVE ||
      type == WheelDefinition::TYPE_CASTER ||
      type == WheelDefinition::TYPE_FIXED ||
      type == WheelDefinition::TYPE_MECANUM)
    {
      return type;
    }
    throw std::runtime_error("Invalid wheel_definition_type value");
  }

  static std::string from_string(const std::string& type)
  {
    using vda5050_interfaces::msg::WheelDefinition;

    if (
      type == WheelDefinition::TYPE_DRIVE ||
      type == WheelDefinition::TYPE_CASTER ||
      type == WheelDefinition::TYPE_FIXED ||
      type == WheelDefinition::TYPE_MECANUM)
    {
      return type;
    }
    throw std::runtime_error("Invalid wheelDefinitionType string");
  }
};
#endif  // ENABLE_ROS2

}  // namespace json_utils
}  // namespace vda5050_core

#endif  // VDA5050_CORE__JSON_UTILS__TRAITS_HPP_
