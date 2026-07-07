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

#ifndef VDA5050_CORE__MASTER__ORDER__ORDER_LIFECYCLE_MANAGER_HPP_
#define VDA5050_CORE__MASTER__ORDER__ORDER_LIFECYCLE_MANAGER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {
namespace master {

// Tracks the master's view of one AGV's in-flight order; one instance per
// managed AGV. Owns lifecycle_mutex_, acquired without holding any AGV mutex.

/// \brief Outcome of combine_order(): a merged Order or accumulated errors.
struct CombineResult
{
  /// Merged Order; populated only when errors is empty.
  vda5050_core::types::Order order;

  /// Combine violations; non-empty means failure.
  std::vector<vda5050_core::types::Error> errors;

  explicit operator bool() const
  {
    return errors.empty();
  }
};

/// \brief Read-only snapshot of the manager's tracked state.
struct ActiveOrderSnapshot
{
  /// True when an active order is being tracked.
  bool has_active = false;

  /// Active order_id (empty when has_active == false).
  std::string order_id;

  uint32_t order_update_id = 0;

  /// Merged nodes (base + horizon, in seq order).
  std::vector<vda5050_core::types::Node> nodes;

  std::vector<vda5050_core::types::Edge> edges;

  /// AGV's most recent reported last_node_sequence_id (0 if no state yet).
  uint32_t last_node_sequence_id = 0;

  /// AGV's most recent reported order_update_id (0 if no state yet).
  uint32_t state_order_update_id = 0;

  /// AGV's most recent reported order_id (empty if no state yet).
  std::string state_order_id;

  /// True once the AGV reports it reached the active order's last node.
  bool order_complete = false;
};

/// \brief Merge base and update at the stitch point, enforcing the VDA5050
///        stitching rules. Pure; does not touch manager state.
///
/// \param last_node_sequence_id  Used to check the AGV hasn't passed the
///                               stitch point.
CombineResult combine_order(
  const vda5050_core::types::Order& base,
  const vda5050_core::types::Order& update, uint32_t last_node_sequence_id);

class OrderLifecycleManager
{
public:
  /// Consecutive mismatched-order_id States before stale tracking clears.
  static constexpr int kDefaultMismatchThreshold = 3;

  /// Pending-update queue cap; enqueue rejects the newest past this.
  static constexpr std::size_t kDefaultPendingQueueCap = 8;

  /// \brief Construct.
  /// \param agv_id  Used in log messages only.
  explicit OrderLifecycleManager(
    std::string agv_id, int mismatch_threshold = kDefaultMismatchThreshold,
    std::size_t pending_queue_cap = kDefaultPendingQueueCap);

  ~OrderLifecycleManager() = default;
  OrderLifecycleManager(const OrderLifecycleManager&) = delete;
  OrderLifecycleManager& operator=(const OrderLifecycleManager&) = delete;
  OrderLifecycleManager(OrderLifecycleManager&&) = delete;
  OrderLifecycleManager& operator=(OrderLifecycleManager&&) = delete;

  // =====================================================================
  // Mutators
  // =====================================================================

  /// \brief Record a successful Order publish and set it active. Call after
  ///        publish succeeds.
  /// \param merged  When the publish stitched onto an active order, the
  ///                merged order to adopt without re-combining.
  void record_published(
    const vda5050_core::types::Order& order,
    const std::optional<vda5050_core::types::Order>& merged = {});

  /// \brief Apply an incoming State: update last-node tracking, mismatch
  ///        counter, completion detection, and pending-queue drain.
  /// \return Pending updates now eligible to publish (caller sends via
  ///         AGV::send_order); returned by value for lock-free iteration.
  std::vector<vda5050_core::types::Order> on_state_update(
    const vda5050_core::types::State& state);

  /// \brief Add an order update to the pending-updates queue. The
  ///        OrderStitcher uses this when a stitch guard returns
  ///        QUEUE_PENDING.
  /// \return false if the queue is at capacity (reject-newest); true on
  ///         successful enqueue.
  bool enqueue_pending_update(const vda5050_core::types::Order& update);

  /// \brief Forced reset: clears active order, pending queue, mismatch
  ///        counter, completion / needs-more-base flags. For master-side
  ///        cancelOrder, AGV restart, etc.
  void clear();

  // =====================================================================
  // Read-only accessors (each acquires lifecycle_mutex_ briefly)
  // =====================================================================

  ActiveOrderSnapshot snapshot() const;
  bool has_active_order() const;
  std::optional<std::string> active_order_id() const;
  std::optional<uint32_t> active_order_update_id() const;
  bool is_order_complete() const;

  /// \brief True when AGV has reported newBaseRequest for the active order
  ///        and master has not yet recorded a strictly higher
  ///        order_update_id for that order_id. Cleared on real extensions
  ///        (no-op republish does not clear it).
  bool active_order_needs_more_base() const;

  std::size_t pending_update_count() const;

private:
  // Drive the mismatch counter under lifecycle_mutex_. Returns true if
  // recovery (3-strike clear) just fired.
  bool tick_mismatch(const vda5050_core::types::State& state);

  // Drain pending_updates_ FIFO under lifecycle_mutex_, returning the
  // updates whose stitch conditions are now satisfied. Combine errors are
  // logged and the offending update is discarded. Adopts each successfully
  // combined update as the new active order in-place.
  std::vector<vda5050_core::types::Order> drain_pending_locked(
    const vda5050_core::types::State& state);

  // Internal accept-and-replace helper. Caller holds lifecycle_mutex_.
  void adopt_active_locked(const vda5050_core::types::Order& order);

  std::string agv_id_;  // log identity, immutable
  const int mismatch_threshold_;
  const std::size_t pending_queue_cap_;

  mutable std::mutex lifecycle_mutex_;

  // Active order (combined view) and cached primary keys for fast paths.
  std::optional<vda5050_core::types::Order> active_order_;
  std::string active_order_id_;
  uint32_t active_order_update_id_ = 0;

  // Mirrors of the most recent State.
  uint32_t last_node_sequence_id_ = 0;
  uint32_t state_order_update_id_ = 0;
  std::string last_state_order_id_;

  // Sticky flags.
  bool order_complete_ = false;
  bool needs_more_base_ = false;

  // Queue-and-retry (deque, not std::queue — we walk front->back
  // before popping).
  std::deque<vda5050_core::types::Order> pending_updates_;

  int mismatch_count_ = 0;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_LIFECYCLE_MANAGER_HPP_
