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

#ifndef VDA5050_CORE__MASTER__EVENT_DETECTOR_HPP_
#define VDA5050_CORE__MASTER__EVENT_DETECTOR_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/provider.hpp"
#include "vda5050_core/master/connection/connection_event_detector.hpp"
#include "vda5050_core/master/state/state_event_detector.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/load.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {
namespace master {

// Discrete transitions the detector publishes, one type per concern so a
// consumer can subscribe to exactly what it cares about. Each carries the AGV
// it came from; value-bearing ones also carry the new value so a consumer
// needn't re-read cached state (NewBaseRequest is a pure edge, no value).

/// \brief A node the AGV newly reported as reached.
struct NodeReachedUpdate
: execution::Initialize<NodeReachedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  event::ReachedNode node;
  NodeReachedUpdate(std::string id, event::ReachedNode n)
  : agv_id(std::move(id)), node(std::move(n))
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
  ConnectionEventKind kind;
  ConnectionChangedUpdate(std::string id, ConnectionEventKind k)
  : agv_id(std::move(id)), kind(k)
  {
  }
};

/// \brief The AGV's operating mode changed.
struct OperatingModeChangedUpdate
: execution::Initialize<OperatingModeChangedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  types::OperatingMode mode;
  OperatingModeChangedUpdate(std::string id, types::OperatingMode m)
  : agv_id(std::move(id)), mode(m)
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

/// \brief Per-AGV transition detector over a single AGV's State / Connection
/// messages.
///
/// Each on_state / on_connection call diffs against the previously seen message
/// and pushes one update per transition through the shared provider, tagged
/// with this AGV's id. The first message of each kind only seeds the baseline
/// (a Connection ONLINE is itself the CONNECTED transition).
///
/// One AGV's messages must be delivered in arrival order: concurrent calls are
/// race-free (the snapshot is mutex-guarded) but transition order across them
/// is not guaranteed.
class EventDetector
{
public:
  /// \brief Construct a detector that publishes this AGV's transitions.
  ///
  /// \param agv_id Id stamped on every update this detector publishes.
  /// \param provider Shared provider the transition updates are pushed through.
  EventDetector(
    std::string agv_id, std::shared_ptr<execution::Provider> provider);

  /// \brief Diff a newly received State and publish an update per transition.
  ///
  /// \param state Latest State message for this AGV.
  void on_state(const types::State& state);

  /// \brief Diff a newly received Connection and publish on a state change.
  ///
  /// \param connection Latest Connection message for this AGV.
  void on_connection(const types::Connection& connection);

private:
  const std::string agv_id_;
  const std::shared_ptr<execution::Provider> provider_;

  std::mutex mutex_;
  types::State prev_state_;
  bool have_prev_state_ = false;
  std::optional<types::Connection> prev_connection_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__EVENT_DETECTOR_HPP_
