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

namespace vda5050_core::master {

// =============================================================================
// OrderLifecycleManager — per-AGV order tracking.
// =============================================================================
//
// First step of the order-management epic. Tracks
// the master's view of one AGV's in-flight order:
//
//   - active order_id / order_update_id
//   - base / horizon split (released vs unreleased nodes/edges)
//   - pending-updates queue (FIWARE queue-and-retry pattern)
//   - 3-strike order_id mismatch counter with internal recovery
//   - newBaseRequest flag
//   - order completion detection (last-node arrival)
//
// Owned by AGV; one instance per managed AGV.
//
// **Stitch logic is NOT here** -- that's #14's OrderStitcher. This class
// only tracks state and exposes snapshots for the stitch guard to query.
//
// **Thread-safety**: this class has its own lifecycle_mutex_ separate from
// AGV's data_mutex_/state_mutex_/queue_mutex_. lifecycle_mutex_ is acquired
// without holding any AGV mutex — no nested locking. Snapshot getters
// return values; on_state_update returns the ready-to-publish update list
// by value so callers iterate lock-free.

/// \brief Outcome of combine_order(): either a merged Order or accumulated
///        errors. Use the bool conversion to short-circuit on failure.
struct CombineResult
{
  /// Merged Order. Only populated when errors is empty.
  vda5050_core::types::Order order;

  /// Spec-rule violations encountered during the combine. Non-empty == fail.
  std::vector<vda5050_core::types::Error> errors;

  /// Allows use in boolean contexts.
  explicit operator bool() const
  {
    return errors.empty();
  }
};

/// \brief Read-only view of the manager's tracked state. Returned by
///        snapshot() under the lifecycle_mutex_, then read lock-free.
struct ActiveOrderSnapshot
{
  /// True when an active order is being tracked (i.e., at least one
  /// successful publish has been recorded and recovery hasn't fired).
  bool has_active = false;

  /// Active order_id (empty when has_active == false).
  std::string order_id;

  /// Active order_update_id (most recent recorded update).
  uint32_t order_update_id = 0;

  /// Merged nodes from the active order (base + horizon, in seq order).
  std::vector<vda5050_core::types::Node> nodes;

  /// Merged edges from the active order.
  std::vector<vda5050_core::types::Edge> edges;

  /// AGV's most recent reported last_node_sequence_id (0 if no state yet).
  uint32_t last_node_sequence_id = 0;

  /// AGV's most recent reported order_update_id (0 if no state yet).
  uint32_t state_order_update_id = 0;

  /// AGV's most recent reported order_id (empty if no state yet, or AGV
  /// reported empty (initial / post-reset).
  std::string state_order_id;

  /// True once the AGV reports last_node_id == active_order.nodes.back().
  bool order_complete = false;
};

/// \brief Combine a base order with an update at the stitch point.
///
/// Pure function: does not touch the manager's state. Encodes VDA5050 v2.0.0
/// stitching rules:
///   - base immutability ("base cannot be changed"): rejects updates that alter a
///     released node/edge.
///   - stitching node content immutability: the node at the
///     stitch sequence_id (last released base node) MUST be byte-equal in
///     base and update; otherwise rejected.
///   - orderUpdateError: rejects update.order_update_id
///     <= base.order_update_id.
///   - order_id mismatch is rejected up-front.
///
/// On success: the merged order has base.order_id, update.order_update_id,
/// update.zone_set_id, and the update's header. nodes/edges are
/// preserved-base-tail + extension, in sequence_id order.
///
/// \param base    The cached active order (last successful publish).
/// \param update  The candidate update (already schema-/graph-valid).
/// \param last_node_sequence_id  AGV's most recent last_node_sequence_id.
///                               Used only to validate that the AGV hasn't
///                               passed the stitch point.
/// \return CombineResult with .order populated on success or .errors set.
CombineResult combine_order(
  const vda5050_core::types::Order& base,
  const vda5050_core::types::Order& update, uint32_t last_node_sequence_id);

class OrderLifecycleManager
{
public:
  /// FIWARE-tested default. Master clears stale tracking after this many
  /// consecutive State messages with mismatched order_id.
  static constexpr int kDefaultMismatchThreshold = 3;

  /// Pending-update queue cap. enqueue_pending_update returns false past
  /// this; reject-newest policy.
  static constexpr std::size_t kDefaultPendingQueueCap = 8;

  /// \brief Construct.
  /// \param agv_id              Used in log messages only (e.g. "ACME/AGV01").
  /// \param mismatch_threshold  Consecutive mismatches before recovery.
  /// \param pending_queue_cap   Max queued pending updates.
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

  /// \brief Record a successful Order publish. Sets the active order; if the
  ///        recorded order has a strictly higher order_update_id for the
  ///        same order_id, clears the needs_more_base_ flag.
  ///        Caller must invoke this AFTER MqttClient::publish returns success.
  ///        When the publish stitched onto an active order, pass the merged
  ///        order from OrderPublisher so it is adopted without re-combining.
  void record_published(
    const vda5050_core::types::Order& order,
    const std::optional<vda5050_core::types::Order>& merged = {});

  /// \brief Apply an incoming State message: update last_node tracking,
  ///        run mismatch counter, detect order completion, drain the
  ///        pending-update queue.
  ///
  /// \return The pending updates that became eligible to publish on this
  ///         state. Caller is responsible for actually publishing them
  ///         (via AGV::send_order). Returned by value so callers iterate
  ///         lock-free.
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

  // FIWARE queue-and-retry (deque, not std::queue — we walk front->back
  // before popping).
  std::deque<vda5050_core::types::Order> pending_updates_;

  int mismatch_count_ = 0;
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_LIFECYCLE_MANAGER_HPP_
