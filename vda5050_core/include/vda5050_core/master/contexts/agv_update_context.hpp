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

#ifndef VDA5050_CORE__MASTER__CONTEXTS__AGV_UPDATE_CONTEXT_HPP_
#define VDA5050_CORE__MASTER__CONTEXTS__AGV_UPDATE_CONTEXT_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/provider.hpp"
#include "vda5050_core/master/connection/connection_update_detector.hpp"
#include "vda5050_core/master/state/state_update_detector.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/load.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {

namespace master {

/// \brief A node the AGV newly reported as reached.
struct NodeReachedUpdate
: execution::Initialize<NodeReachedUpdate, execution::UpdateBase>
{
  std::string agv_id;
  update::ReachedNode node;
  NodeReachedUpdate(std::string id, update::ReachedNode n)
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

/// \brief Per-AGV context: turns inbound State/Connection into typed updates
/// on its Provider, caching the latest of each type for get_update<T>().
///
/// on_state / on_connection must be called from one thread; get_update<T>()
/// is safe from any thread.
class AGVUpdateContext : public execution::ContextInterface
{
public:
  /// \brief Construct a context that stamps this AGV's id on every update.
  ///
  /// \param agv_id Id stamped on every update this context publishes.
  explicit AGVUpdateContext(std::string agv_id);

  /// \brief No-op: updates are cached at production, so nothing to register.
  void init() override;

  /// \brief Diff a newly received State and publish an update per transition.
  ///
  /// \param state Latest State message for this AGV.
  void on_state(const types::State& state);

  /// \brief Diff a newly received Connection and publish on a state change.
  ///
  /// \param connection Latest Connection message for this AGV.
  void on_connection(const types::Connection& connection);

protected:
  /// \brief Latest cached update of the given type, or nullptr if none.
  std::shared_ptr<execution::UpdateBase> get_update_raw(
    std::type_index type) const override;

  /// \brief No resources are cached; always nullptr.
  std::shared_ptr<execution::ResourceBase> get_resource_raw(
    std::type_index type) const override;

private:
  // Cache the latest of each type, then publish to subscribers.
  template <typename UpdateT, typename... Args>
  void publish(Args&&... args)
  {
    auto update = std::make_shared<UpdateT>(std::forward<Args>(args)...);
    cache_update(update);
    provider()->push_shared(update);
  }

  void cache_update(const std::shared_ptr<execution::UpdateBase>& update);

  const std::string agv_id_;
  std::optional<types::State> prev_state_;
  std::optional<types::Connection> prev_connection_;

  mutable std::mutex storage_mutex_;
  std::unordered_map<std::type_index, std::shared_ptr<execution::UpdateBase>>
    updates_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__CONTEXTS__AGV_UPDATE_CONTEXT_HPP_
