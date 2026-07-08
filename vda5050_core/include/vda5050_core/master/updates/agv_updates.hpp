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

#ifndef VDA5050_CORE__MASTER__UPDATES__AGV_UPDATES_HPP_
#define VDA5050_CORE__MASTER__UPDATES__AGV_UPDATES_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/load.hpp"
#include "vda5050_core/types/operating_mode.hpp"

namespace vda5050_core {

namespace master {

/// \brief Kind of connection-state transition. CONNECTIONBROKEN = broker
/// last-will (unexpected drop); OFFLINE = graceful.
enum class ConnectionTransition
{
  NONE,
  CONNECTED,
  OFFLINE,
  CONNECTIONBROKEN,
};

/// \brief A node the AGV newly reported as reached (lastNodeId +
/// lastNodeSequenceId).
struct NodeReachedUpdate
: execution::Initialize<NodeReachedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  std::string node_id;
  uint32_t sequence_id;
  NodeReachedUpdate(std::string id, std::string node, uint32_t sequence)
  : agv_id(std::move(id)), node_id(std::move(node)), sequence_id(sequence)
  {
  }
};

/// \brief Errors that appeared and/or cleared since the previous State.
struct ErrorsChangedUpdate
: execution::Initialize<ErrorsChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  std::vector<types::Error> appeared;
  std::vector<types::Error> resolved;
  ErrorsChangedUpdate(
    std::string id, std::vector<types::Error> a, std::vector<types::Error> r)
  : agv_id(std::move(id)), appeared(std::move(a)), resolved(std::move(r))
  {
  }
};

/// \brief The AGV's connection state changed (or first report).
struct ConnectionChangedUpdate
: execution::Initialize<ConnectionChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  ConnectionTransition kind;
  ConnectionChangedUpdate(std::string id, ConnectionTransition k)
  : agv_id(std::move(id)), kind(k)
  {
  }
};

/// \brief The AGV's operating mode changed. Carries the prev and new mode.
struct OperatingModeChangedUpdate
: execution::Initialize<OperatingModeChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  types::OperatingMode mode;       ///< mode after the change
  types::OperatingMode prev_mode;  ///< mode before the change
  OperatingModeChangedUpdate(
    std::string id, types::OperatingMode m, types::OperatingMode p)
  : agv_id(std::move(id)), mode(m), prev_mode(p)
  {
  }
};

/// \brief The AGV's paused flag changed.
struct PausedChangedUpdate
: execution::Initialize<PausedChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  bool paused;
  PausedChangedUpdate(std::string id, bool p) : agv_id(std::move(id)), paused(p)
  {
  }
};

/// \brief The AGV's driving flag changed.
struct DrivingChangedUpdate
: execution::Initialize<DrivingChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  bool driving;
  DrivingChangedUpdate(std::string id, bool d)
  : agv_id(std::move(id)), driving(d)
  {
  }
};

/// \brief The AGV raised a new-base request (rising edge).
struct NewBaseRequestUpdate
: execution::Initialize<NewBaseRequestUpdate, execution::UpdateBase>
{
  std::string agv_id;
  explicit NewBaseRequestUpdate(std::string id) : agv_id(std::move(id)) {}
};

/// \brief The AGV's load set changed.
struct LoadsChangedUpdate
: execution::Initialize<LoadsChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  std::vector<types::Load> loads;
  LoadsChangedUpdate(std::string id, std::vector<types::Load> l)
  : agv_id(std::move(id)), loads(std::move(l))
  {
  }
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__UPDATES__AGV_UPDATES_HPP_
