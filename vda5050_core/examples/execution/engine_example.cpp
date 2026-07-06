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

#include <vda5050_core/logger/logger.hpp>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/engine.hpp"

using EventBase = vda5050_core::execution::EventBase;
using UpdateBase = vda5050_core::execution::UpdateBase;
using Engine = vda5050_core::execution::Engine;
using Priority = vda5050_core::execution::Priority;

struct MoveEvent
: public vda5050_core::execution::Initialize<MoveEvent, EventBase>
{
  std::string direction;

  explicit MoveEvent(const std::string& direction)
  : direction(std::move(direction))
  {
    // Nothing to do here ...
  }
};

struct SensorData
: public vda5050_core::execution::Initialize<SensorData, UpdateBase>
{
  double value;

  explicit SensorData(double value) : value(value)
  {
    // Nothing to do here ...
  }
};

int main()
{
  auto engine = std::make_shared<Engine>();

  engine->on<MoveEvent>([](auto event) {
    VDA5050_INFO_STREAM("[ROBOT] Movement: " << event->direction);
  });

  VDA5050_INFO("Stage 1: Priority Execution ...");

  engine->emit<MoveEvent>(Priority::NORMAL, "Forward");
  engine->emit<MoveEvent>(Priority::CRITICAL, "EMERGENCY STOP");

  engine->step();
  engine->step();

  VDA5050_INFO("Stage 2: Wait For Update ...");

  engine->suspend_for<SensorData>(
    std::chrono::seconds(2),
    [](auto update) -> bool { return update->value >= 10.0; });

  engine->emit<MoveEvent>(Priority::NORMAL, "Post-Wait Action");

  VDA5050_INFO("Stepping engine while waiting ...");
  engine->step();
  VDA5050_INFO_STREAM(
    "Is engine waiting? " << (engine->waiting() ? "Yes" : "No"));

  VDA5050_INFO("Sending SensorData(12.0) to resolve wait ...");
  engine->notify(std::make_shared<SensorData>(12.0));

  engine->step();
  VDA5050_INFO_STREAM(
    "Is engine waiting? " << (engine->waiting() ? "Yes" : "No"));

  return 0;
}
