/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VDA5050_CORE__CLIENT__CONTEXTS__AGV_CONTEXT_HPP_
#define VDA5050_CORE__CLIENT__CONTEXTS__AGV_CONTEXT_HPP_

#include <memory>
#include <mutex>
#include <typeindex>
#include <unordered_map>
#include <utility>

#include "vda5050_core/client/resources/config.hpp"
#include "vda5050_core/execution/context_interface.hpp"

namespace vda5050_core {

namespace client {

/// \brief AGV client context: in-memory storage for VDA5050 execution data.
///
/// Seeds `HeaderConfigResource` and `OrderExecutionResource` as persistent
/// resources during construction. init() then registers a provider callback for
/// `OrderUpdate` so strategies can read the latest inbound order via
/// get_update<OrderUpdate>().
///
/// Order execution state is owned by the `OrderExecutionResource`; strategies
/// reach it via get_resource<OrderExecutionResource>() and use its thread-safe
/// accessors. The context itself only handles update/resource plumbing.
///
/// All map reads and writes are protected by a single `storage_mutex_` via
/// std::lock_guard to prevent race conditions when strategies run concurrently.
class AGVContext : public execution::ContextInterface,
                   public std::enable_shared_from_this<AGVContext>
{
public:
  /// \brief Create an AGVContext owned by a shared_ptr.
  ///
  /// `HeaderConfigResource` and a blank `OrderExecutionResource` are seeded at
  /// construction and available via get_resource<T>() once make() returns.
  ///
  /// \throws std::invalid_argument if `config` is nullptr.
  static std::shared_ptr<AGVContext> make(
    std::shared_ptr<HeaderConfigResource> config)
  {
    return std::shared_ptr<AGVContext>(new AGVContext(std::move(config)));
  }

  ~AGVContext() override = default;

  /// \brief Register the provider callback for inbound updates.
  void init() override;

protected:
  /// \brief Thread-safe lookup in the updates map. Returns nullptr if not found.
  std::shared_ptr<execution::UpdateBase> get_update_raw(
    std::type_index type) const override;

  /// \brief Thread-safe lookup in the resources map. Returns nullptr if not found.
  std::shared_ptr<execution::ResourceBase> get_resource_raw(
    std::type_index type) const override;

private:
  /// \brief Private constructor; use make() to obtain a shared_ptr instance.
  explicit AGVContext(std::shared_ptr<HeaderConfigResource> config);

  // Thread-safe internal layout caches
  void cache_update(std::shared_ptr<execution::UpdateBase> update);
  void cache_resource(std::shared_ptr<execution::ResourceBase> resource);

  bool initialized_ = false;

  std::unordered_map<std::type_index, std::shared_ptr<execution::UpdateBase>>
    updates_;
  std::unordered_map<std::type_index, std::shared_ptr<execution::ResourceBase>>
    resources_;
  mutable std::mutex storage_mutex_;
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__CONTEXTS__AGV_CONTEXT_HPP_
