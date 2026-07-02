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

#ifndef VDA5050_CORE__LOGGER__LOGGER_HPP_
#define VDA5050_CORE__LOGGER__LOGGER_HPP_

#include <fmt/format.h>
#include <memory>
#include <sstream>
#include <string>

namespace vda5050_core {

namespace logger {

/// \brief Enum for logging level
enum class LogLevel
{
  DEBUG,
  INFO,
  WARN,
  ERROR,
  FATAL,
};

class LogHandler
{
public:
  /// \brief Default destructor
  virtual ~LogHandler() = default;

  /// \brief Log a message
  ///
  /// \param level Log severity
  /// \param message Log message
  virtual void log(LogLevel level, const std::string& message) = 0;
};

/// \brief Set a handler to receive logs
///
/// \param handler Pointer to the handler
void set_handler(std::unique_ptr<LogHandler> handler);

/// \brief Release the current handler pointer
void release_handler();

/// \brief Set the global log severity to filter out lower severity messages
///
/// \param Log severity level
void set_log_level(LogLevel level);

/// \brief Entry point to the logger
///
/// \param level Log severity
/// \param message Log message
void log(LogLevel level, const std::string& message);

}  // namespace logger
}  // namespace vda5050_core

/// \brief Log a formatted message with a specific log level
///
/// \param level Log level
/// \param ... Variadic arguments passed to fmt::format
#define VDA5050_LOG(level, ...) \
  vda5050_core::logger::log(level, fmt::format(__VA_ARGS__))

/// \brief Log a message with stream syntax with a specific log level
///
/// \param level Log level
/// \param stream Stream expression
#define VDA5050_LOG_STREAM(level, stream)       \
  do {                                          \
    std::stringstream ss;                       \
    ss << stream;                               \
    vda5050_core::logger::log(level, ss.str()); \
  } while (0)

/// \brief Log a formatted message with DEBUG level
///
/// \param ... Variadic argument passed to fmt::format
#define VDA5050_DEBUG(...) \
  VDA5050_LOG(vda5050_core::logger::LogLevel::DEBUG, __VA_ARGS__)

/// \brief Log a formatted message with INFO level
///
/// \param ... Variadic argument passed to fmt::format
#define VDA5050_INFO(...) \
  VDA5050_LOG(vda5050_core::logger::LogLevel::INFO, __VA_ARGS__)

/// \brief Log a formatted message with WARN level
///
/// \param ... Variadic argument passed to fmt::format
#define VDA5050_WARN(...) \
  VDA5050_LOG(vda5050_core::logger::LogLevel::WARN, __VA_ARGS__)

/// \brief Log a formatted message with ERROR level
///
/// \param ... Variadic argument passed to fmt::format
#define VDA5050_ERROR(...) \
  VDA5050_LOG(vda5050_core::logger::LogLevel::ERROR, __VA_ARGS__)

/// \brief Log a formatted message with FATAL level
///
/// \param ... Variadic argument passed to fmt::format
#define VDA5050_FATAL(...) \
  VDA5050_LOG(vda5050_core::logger::LogLevel::FATAL, __VA_ARGS__)

/// \brief Log a DEBUG level message with stream syntax
///
/// \param arg Stream expression
#define VDA5050_DEBUG_STREAM(arg) \
  VDA5050_LOG_STREAM(vda5050_core::logger::LogLevel::DEBUG, arg)

/// \brief Log a INFO level message with stream syntax
///
/// \param arg Stream expression
#define VDA5050_INFO_STREAM(arg) \
  VDA5050_LOG_STREAM(vda5050_core::logger::LogLevel::INFO, arg)

/// \brief Log a WARN level message with stream syntax
///
/// \param arg Stream expression
#define VDA5050_WARN_STREAM(arg) \
  VDA5050_LOG_STREAM(vda5050_core::logger::LogLevel::WARN, arg)

/// \brief Log a ERROR level message with stream syntax
///
/// \param arg Stream expression
#define VDA5050_ERROR_STREAM(arg) \
  VDA5050_LOG_STREAM(vda5050_core::logger::LogLevel::ERROR, arg)

/// \brief Log a FATAL level message with stream syntax
///
/// \param arg Stream expression
#define VDA5050_FATAL_STREAM(arg) \
  VDA5050_LOG_STREAM(vda5050_core::logger::LogLevel::FATAL, arg)

#endif  // VDA5050_CORE__LOGGER__LOGGER_HPP_
