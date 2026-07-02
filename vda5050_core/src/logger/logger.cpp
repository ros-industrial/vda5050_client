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

#include "vda5050_core/logger/logger.hpp"

#include <fmt/chrono.h>
#include <chrono>
#include <iostream>
#include <mutex>

#include "vda5050_core/compat/time_utils.hpp"

namespace vda5050_core {

namespace logger {

namespace {

LogLevel current_level = LogLevel::INFO;
std::mutex handler_mutex;

//=============================================================================
class ConsoleLogger : public LogHandler
{
public:
  void log(LogLevel level, const std::string& message) override
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_t = std::chrono::system_clock::to_time_t(now);
    std::tm localtime = compat::to_localtime(now_t);

    std::cout << "[" << fmt::format("{:%Y-%m-%d %H:%M:%S}", localtime) << "]"
              << to_log_level_string(level) << ": " << message << std::endl;
  }

  std::string to_log_level_string(LogLevel level)
  {
    switch (level)
    {
      case LogLevel::DEBUG:
        return "[DEBUG]";
      case LogLevel::INFO:
        return "[INFO]";
      case LogLevel::WARN:
        return "[WARN]";
      case LogLevel::ERROR:
        return "[ERROR]";
      case LogLevel::FATAL:
        return "[FATAL]";
      default:
        return "[UNKNOWN]";
    }
  }
};

}  // namespace

//=============================================================================
std::unique_ptr<LogHandler> log_handler =
  std::unique_ptr<ConsoleLogger>(new ConsoleLogger());

//=============================================================================
void set_handler(std::unique_ptr<LogHandler> handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex);
  log_handler = std::move(handler);
}

//=============================================================================
void release_handler()
{
  std::lock_guard<std::mutex> lock(handler_mutex);
  log_handler.reset();
  current_level = LogLevel::INFO;
}

//=============================================================================
void set_log_level(LogLevel level)
{
  std::lock_guard<std::mutex> lock(handler_mutex);
  current_level = level;
}

//=============================================================================
void log(LogLevel level, const std::string& message)
{
  std::lock_guard<std::mutex> lock(handler_mutex);

  if (level < current_level) return;

  if (!log_handler)
    log_handler = std::unique_ptr<ConsoleLogger>(new ConsoleLogger());

  log_handler->log(level, message);
}

}  // namespace logger
}  // namespace vda5050_core
