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

#include "vda5050_core/master/order/active_order_snapshot.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core {
namespace master {

// Master's view of one AGV's in-flight order. Owns lifecycle_mutex_.

/// \brief combine_order() result: a merged Order, or errors on failure.
struct CombineResult
{
  vda5050_core::types::Order order;
  std::vector<vda5050_core::types::Error> errors;

  explicit operator bool() const
  {
    return errors.empty();
  }
};

/// \brief Merge base + update at the stitch point per the VDA5050 rules. Pure.
/// \param last_node_sequence_id  AGV last-reached seq; fails if past stitch.
CombineResult combine_order(
  const vda5050_core::types::Order& base,
  const vda5050_core::types::Order& update, uint32_t last_node_sequence_id);

class OrderLifecycleManager
{
public:
  static constexpr int kDefaultMismatchThreshold = 3;
  static constexpr std::size_t kDefaultPendingQueueCap = 8;
  // Counted in States, not time.
  static constexpr int kDefaultPendingMaxWaits = 60;
  // States still reporting the prior order_id tolerated as handover lag before
  // it counts toward the mismatch recovery.
  static constexpr int kMaxHandoverLagStates = 5;

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

  /// \brief Record a successful publish and set it active.
  /// \param merged  Merged order to adopt without re-combining, if stitched.
  void record_published(
    const vda5050_core::types::Order& order,
    const std::optional<vda5050_core::types::Order>& merged = {});

  /// \brief Apply an incoming State (tracking, mismatch, completion, drain).
  /// \param just_completed_order_id  Set to the order_id on the false→true
  ///        completion edge; left untouched otherwise.
  /// \return Pending updates now eligible to publish.
  std::vector<vda5050_core::types::Order> on_state_update(
    const vda5050_core::types::State& state,
    std::optional<std::string>* just_completed_order_id = nullptr);

  /// \brief Queue an order update (used by OrderStitcher on QUEUE_PENDING).
  /// \return false if the queue is at capacity.
  bool enqueue_pending_update(const vda5050_core::types::Order& update);

  /// \brief The error describing an update dropped for want of queue room.
  static vda5050_core::types::Error pending_queue_full_error(
    const std::string& order_id);

  /// \brief Return drained-but-unsent updates to the front of the pending
  ///        queue, preserving order, so a congested outbound queue can't
  ///        reorder the stitch chain.
  void requeue_pending_front(
    const std::vector<vda5050_core::types::Order>& updates);

  /// \brief Forced reset of all tracking. For cancelOrder, AGV restart, etc.
  void clear();

  /// \brief Drop queued pending updates only, keeping active-order tracking.
  void clear_pending();

  /// \brief Reset the stale-State gate so a restarted AGV (header_id back to 0)
  ///        is not locked out. Call on the disconnect edge.
  void reset_state_baseline();

  // =====================================================================
  // Read-only accessors (each acquires lifecycle_mutex_ briefly)
  // =====================================================================

  ActiveOrderSnapshot snapshot() const;
  bool has_active_order() const;
  std::optional<std::string> active_order_id() const;
  std::optional<uint32_t> active_order_update_id() const;
  bool is_order_complete() const;

  /// \brief True while the AGV has requested more base and the master has not
  ///        yet published a higher order_update_id.
  bool active_order_needs_more_base() const;

  std::size_t pending_update_count() const;

private:
  // Returns true if the 3-strike mismatch clear just fired.
  bool tick_mismatch(const vda5050_core::types::State& state);

  std::vector<vda5050_core::types::Order> drain_pending_locked(
    const vda5050_core::types::State& state);

  // Caller holds lifecycle_mutex_.
  void adopt_active_locked(const vda5050_core::types::Order& order);

  std::string agv_id_;
  const int mismatch_threshold_;
  const std::size_t pending_queue_cap_;

  mutable std::mutex lifecycle_mutex_;

  std::optional<vda5050_core::types::Order> active_order_;
  std::string active_order_id_;
  uint32_t active_order_update_id_ = 0;

  // Mirrors of the most recent State.
  uint32_t last_node_sequence_id_ = 0;
  uint32_t state_order_update_id_ = 0;
  std::string last_state_order_id_;

  // Stale-State gate (QoS 0): drop header_id not strictly newer.
  uint32_t last_state_header_id_ = 0;
  bool have_state_baseline_ = false;

  // Prior order_id; a State still reporting it is handover lag, not a mismatch.
  std::string prev_active_order_id_;

  bool order_complete_ = false;
  bool needs_more_base_ = false;

  // waits counts States spent unsent so a wedged update ages out.
  struct PendingUpdate
  {
    vda5050_core::types::Order order;
    int waits = 0;
  };
  std::deque<PendingUpdate> pending_updates_;

  int mismatch_count_ = 0;
  int handover_lag_count_ = 0;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ORDER__ORDER_LIFECYCLE_MANAGER_HPP_
