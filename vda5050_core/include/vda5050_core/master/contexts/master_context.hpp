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

#ifndef VDA5050_CORE__MASTER__CONTEXTS__MASTER_CONTEXT_HPP_
#define VDA5050_CORE__MASTER__CONTEXTS__MASTER_CONTEXT_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <typeindex>
#include <unordered_map>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/provider.hpp"
#include "vda5050_core/master/updates/agv_updates.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {

namespace master {

/// \brief Diffs every AGV's State / Connection into typed updates (tagged with
///        agv_id) on one shared Provider. One instance for the fleet; per-AGV
///        baselines are mutex-guarded. Producer-only.
class MasterContext : public execution::ContextInterface
{
public:
  MasterContext() = default;

  /// \brief No-op: updates are produced on demand, nothing to register.
  void init() override;

  /// \brief Diff a newly received State for `agv_id` and publish an update
  ///        per transition. Out-of-order States (older header_id) are dropped.
  void on_state(const std::string& agv_id, const types::State& state);

  /// \brief Diff a newly received Connection for `agv_id` and publish on a
  ///        state change.
  void on_connection(
    const std::string& agv_id, const types::Connection& connection);

  /// \brief Drop an AGV's baselines on offboard so a re-onboard of the same
  ///        id starts clean (no phantom edges, no leaked State).
  void forget_agv(const std::string& agv_id);

protected:
  /// \brief Producer-only: the per-AGV latest lives on the AGV, so no cache.
  std::shared_ptr<execution::UpdateBase> get_update_raw(
    std::type_index type) const override;

  /// \brief No resources; always nullptr.
  std::shared_ptr<execution::ResourceBase> get_resource_raw(
    std::type_index type) const override;

private:
  // Per-AGV baselines, guarded by mutex_.
  mutable std::mutex mutex_;
  std::unordered_map<std::string, types::State> prev_states_;
  std::unordered_map<std::string, types::Connection> prev_connections_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__CONTEXTS__MASTER_CONTEXT_HPP_
