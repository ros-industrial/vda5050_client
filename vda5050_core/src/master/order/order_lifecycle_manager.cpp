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

#include "vda5050_core/master/order/order_lifecycle_manager.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/logger/logger.hpp"

namespace vda5050_core::master {

namespace {

using vda5050_core::errors::create_error;
using vda5050_core::errors::OrderUpdateError;
using vda5050_core::errors::RefEdgeId;
using vda5050_core::errors::RefNodeId;
using vda5050_core::errors::RefOrderId;
using vda5050_core::errors::RefOrderUpdateId;

// Build an Error with order-id ref attached. Used inside combine_order().
vda5050_core::types::Error make_combine_error(
  const std::string& description, const std::string& order_id,
  std::vector<vda5050_core::types::ErrorReference> extra_refs = {})
{
  std::vector<vda5050_core::types::ErrorReference> refs;
  refs.reserve(1 + extra_refs.size());
  refs.push_back({RefOrderId, order_id});
  for (auto& r : extra_refs) refs.push_back(std::move(r));
  return create_error(OrderUpdateError, description, refs);
}

}  // namespace

// =============================================================================
// combine_order — pure free function
// =============================================================================
CombineResult combine_order(
  const vda5050_core::types::Order& base,
  const vda5050_core::types::Order& update, uint32_t last_node_sequence_id)
{
  CombineResult res;
  auto fail = [&](
                const std::string& msg,
                std::vector<vda5050_core::types::ErrorReference> refs = {}) {
    res.errors.push_back(
      make_combine_error(msg, base.order_id, std::move(refs)));
  };

  // Pre-check: order_id must match (updates extend the same order).
  if (base.order_id != update.order_id)
  {
    fail(
      "Update order_id does not match base order_id",
      {{RefOrderId, update.order_id}});
    return res;
  }

  // Pre-check: order_update_id must strictly increase.
  if (update.order_update_id <= base.order_update_id)
  {
    fail(
      "Update order_update_id is not greater than base order_update_id",
      {{RefOrderUpdateId, std::to_string(update.order_update_id)}});
    return res;
  }

  // Partition base.nodes into released (base) vs unreleased (horizon).
  // Released nodes appear before unreleased per VDA5050 base/horizon ordering.
  std::vector<vda5050_core::types::Node> preserved;
  preserved.reserve(base.nodes.size());

  const vda5050_core::types::Node* old_base_last = nullptr;
  for (const auto& n : base.nodes)
  {
    if (n.released) old_base_last = &n;
  }

  if (old_base_last == nullptr && !base.nodes.empty())
  {
    fail("Base order has no released base node");
    return res;
  }

  // preserved = [old_base_last] + horizon. The last released base node is
  // kept because it's the stitch anchor. AGVs already past it would not
  // reach this code path (graph validator would already have rejected).
  bool seen_base_last = false;
  for (const auto& n : base.nodes)
  {
    if (!n.released)
    {
      preserved.push_back(n);
      continue;
    }
    if (
      old_base_last != nullptr && n.sequence_id == old_base_last->sequence_id &&
      !seen_base_last)
    {
      preserved.push_back(n);
      seen_base_last = true;
    }
  }

  // Reachability check on the stitch point: AGV must not have passed it.
  // (Strict inequality — equal is fine, AGV is parked at the stitch node.)
  if (
    old_base_last != nullptr &&
    last_node_sequence_id > old_base_last->sequence_id)
  {
    fail(
      "AGV has already passed the stitch point",
      {{::vda5050_core::errors::RefSequenceId,
        std::to_string(last_node_sequence_id)}});
    return res;
  }

  // Walk update.nodes and merge into `preserved`.
  uint32_t preserved_max_seq =
    preserved.empty() ? 0 : preserved.back().sequence_id;

  for (const auto& nu : update.nodes)
  {
    if (old_base_last != nullptr && nu.sequence_id < old_base_last->sequence_id)
    {
      fail(
        "Update attempts to alter a released base node; the base cannot be "
        "changed",
        {{RefNodeId, nu.node_id}});
      continue;
    }

    if (
      old_base_last != nullptr && nu.sequence_id == old_base_last->sequence_id)
    {
      // Stitching node — its content must be identical to the base copy.
      if (nu != *old_base_last)
      {
        fail(
          "Stitch node content differs from base; the stitch node must be "
          "identical",
          {{RefNodeId, nu.node_id}});
      }
      // Either way, do NOT replace — base copy already in `preserved`.
      continue;
    }

    // Beyond the preserved tail → append (extension).
    if (nu.sequence_id > preserved_max_seq)
    {
      preserved.push_back(nu);
      preserved_max_seq = nu.sequence_id;
      continue;
    }

    // Within-horizon replacement.
    auto it = std::find_if(
      preserved.begin(), preserved.end(),
      [&](const vda5050_core::types::Node& n) {
        return n.sequence_id == nu.sequence_id;
      });

    if (it == preserved.end())
    {
      // Sparse insert — keep seq order.
      auto pos = std::find_if(
        preserved.begin(), preserved.end(),
        [&](const vda5050_core::types::Node& n) {
          return n.sequence_id > nu.sequence_id;
        });
      preserved.insert(pos, nu);
    }
    else
    {
      if (it->released && !nu.released)
      {
        fail(
          "Update attempts to un-release a released node",
          {{RefNodeId, nu.node_id}});
        continue;
      }
      *it = nu;
    }
  }

  // Apply parallel logic to edges.
  std::vector<vda5050_core::types::Edge> preserved_edges;
  preserved_edges.reserve(base.edges.size());

  uint32_t old_base_last_edge_seq = 0;
  bool has_base_edge = false;
  for (const auto& e : base.edges)
  {
    if (e.released)
    {
      old_base_last_edge_seq = e.sequence_id;
      has_base_edge = true;
    }
  }

  bool seen_base_last_edge = false;
  for (const auto& e : base.edges)
  {
    if (!e.released)
    {
      preserved_edges.push_back(e);
      continue;
    }
    if (
      has_base_edge && e.sequence_id == old_base_last_edge_seq &&
      !seen_base_last_edge)
    {
      preserved_edges.push_back(e);
      seen_base_last_edge = true;
    }
  }

  uint32_t preserved_edge_max_seq =
    preserved_edges.empty() ? 0 : preserved_edges.back().sequence_id;

  for (const auto& eu : update.edges)
  {
    if (has_base_edge && eu.sequence_id < old_base_last_edge_seq)
    {
      fail(
        "Update attempts to alter a released base edge; the base cannot be "
        "changed",
        {{RefEdgeId, eu.edge_id}});
      continue;
    }
    if (has_base_edge && eu.sequence_id == old_base_last_edge_seq)
    {
      // Stitching edge — also content-immutable.
      auto it = std::find_if(
        preserved_edges.begin(), preserved_edges.end(),
        [&](const vda5050_core::types::Edge& e) {
          return e.sequence_id == old_base_last_edge_seq;
        });
      if (it != preserved_edges.end() && eu != *it)
      {
        fail(
          "Stitch edge content differs from base; the stitch edge must be "
          "identical",
          {{RefEdgeId, eu.edge_id}});
      }
      continue;
    }
    if (eu.sequence_id > preserved_edge_max_seq)
    {
      preserved_edges.push_back(eu);
      preserved_edge_max_seq = eu.sequence_id;
      continue;
    }
    auto it = std::find_if(
      preserved_edges.begin(), preserved_edges.end(),
      [&](const vda5050_core::types::Edge& e) {
        return e.sequence_id == eu.sequence_id;
      });
    if (it == preserved_edges.end())
    {
      auto pos = std::find_if(
        preserved_edges.begin(), preserved_edges.end(),
        [&](const vda5050_core::types::Edge& e) {
          return e.sequence_id > eu.sequence_id;
        });
      preserved_edges.insert(pos, eu);
    }
    else
    {
      if (it->released && !eu.released)
      {
        fail(
          "Update attempts to un-release a released edge",
          {{RefEdgeId, eu.edge_id}});
        continue;
      }
      *it = eu;
    }
  }

  if (!res.errors.empty()) return res;

  res.order.header = update.header;
  res.order.order_id = base.order_id;
  res.order.order_update_id = update.order_update_id;
  res.order.zone_set_id = update.zone_set_id;
  res.order.nodes = std::move(preserved);
  res.order.edges = std::move(preserved_edges);
  return res;
}

// =============================================================================
// OrderLifecycleManager
// =============================================================================
OrderLifecycleManager::OrderLifecycleManager(
  std::string agv_id, int mismatch_threshold, std::size_t pending_queue_cap)
: agv_id_(std::move(agv_id)),
  mismatch_threshold_(mismatch_threshold),
  pending_queue_cap_(pending_queue_cap)
{
}

void OrderLifecycleManager::record_published(
  const vda5050_core::types::Order& order,
  const std::optional<vda5050_core::types::Order>& merged)
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);

  const bool same_order =
    !active_order_id_.empty() && active_order_id_ == order.order_id;
  const bool real_extension =
    same_order && order.order_update_id > active_order_update_id_;

  if (real_extension && active_order_.has_value())
  {
    // The wire carries only the stitch node + extension, but the internal
    // view must remain the full merged base+horizon so next-update
    // validation, completion detection, and snapshot consumers see the true
    // route. Adopt the merged order the publisher already validated; that
    // avoids a second combine against a state that may have advanced since.
    if (merged.has_value())
    {
      adopt_active_locked(*merged);
    }
    else
    {
      // Fallback when no merged order was supplied (e.g. a caller that does
      // not run the publisher chain): re-derive it.
      auto combined =
        combine_order(*active_order_, order, last_node_sequence_id_);
      adopt_active_locked(combined ? combined.order : order);
    }
    needs_more_base_ = false;
  }
  else
  {
    adopt_active_locked(order);
    if (!same_order)
    {
      // New order entirely — reset sticky flags.
      order_complete_ = false;
      needs_more_base_ = false;
    }
  }
}

std::vector<vda5050_core::types::Order> OrderLifecycleManager::on_state_update(
  const vda5050_core::types::State& state)
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);

  // Mirror state into our cache.
  last_node_sequence_id_ = state.last_node_sequence_id;
  state_order_update_id_ = state.order_update_id;
  last_state_order_id_ = state.order_id;

  // newBaseRequest is sticky-true; cleared by record_published on extension.
  if (state.new_base_request.value_or(false))
  {
    needs_more_base_ = true;
  }

  // Run mismatch counter. If it fires recovery, drop the rest of the work.
  if (tick_mismatch(state)) return {};

  // Order-completion detection (last node reached).
  if (active_order_ && !active_order_->nodes.empty())
  {
    const auto& last = active_order_->nodes.back();
    if (state.last_node_sequence_id == last.sequence_id)
    {
      if (state.last_node_id == last.node_id)
      {
        if (!order_complete_)
        {
          VDA5050_INFO(
            "[OrderLifecycle] {} order {} (update {}) completed at node {}",
            agv_id_, active_order_id_, active_order_update_id_, last.node_id);
        }
        order_complete_ = true;
      }
      else
      {
        VDA5050_WARN(
          "[OrderLifecycle] {} state seq={} matches but last_node_id={} "
          "differs from order's {}; not marking complete",
          agv_id_, state.last_node_sequence_id, state.last_node_id,
          last.node_id);
      }
    }
  }

  // Drain pending queue.
  return drain_pending_locked(state);
}

bool OrderLifecycleManager::enqueue_pending_update(
  const vda5050_core::types::Order& update)
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (pending_updates_.size() >= pending_queue_cap_)
  {
    VDA5050_WARN(
      "[OrderLifecycle] {} pending queue full ({}); rejecting update {}",
      agv_id_, pending_queue_cap_, update.order_update_id);
    return false;
  }
  pending_updates_.push_back(update);
  return true;
}

void OrderLifecycleManager::clear()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  active_order_.reset();
  active_order_id_.clear();
  active_order_update_id_ = 0;
  last_node_sequence_id_ = 0;
  state_order_update_id_ = 0;
  last_state_order_id_.clear();
  order_complete_ = false;
  needs_more_base_ = false;
  pending_updates_.clear();
  mismatch_count_ = 0;
}

// =============================================================================
// Read-only accessors
// =============================================================================
ActiveOrderSnapshot OrderLifecycleManager::snapshot() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  ActiveOrderSnapshot s;
  s.has_active = active_order_.has_value();
  if (s.has_active)
  {
    s.order_id = active_order_id_;
    s.order_update_id = active_order_update_id_;
    s.nodes = active_order_->nodes;
    s.edges = active_order_->edges;
  }
  s.last_node_sequence_id = last_node_sequence_id_;
  s.state_order_update_id = state_order_update_id_;
  s.state_order_id = last_state_order_id_;
  s.order_complete = order_complete_;
  return s;
}

bool OrderLifecycleManager::has_active_order() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return active_order_.has_value();
}

std::optional<std::string> OrderLifecycleManager::active_order_id() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!active_order_.has_value()) return std::nullopt;
  return active_order_id_;
}

std::optional<uint32_t> OrderLifecycleManager::active_order_update_id() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (!active_order_.has_value()) return std::nullopt;
  return active_order_update_id_;
}

bool OrderLifecycleManager::is_order_complete() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return order_complete_;
}

bool OrderLifecycleManager::active_order_needs_more_base() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return needs_more_base_;
}

std::size_t OrderLifecycleManager::pending_update_count() const
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return pending_updates_.size();
}

// =============================================================================
// Private helpers (caller holds lifecycle_mutex_)
// =============================================================================
bool OrderLifecycleManager::tick_mismatch(
  const vda5050_core::types::State& state)
{
  // Skip if no active order to mismatch against, or AGV reports empty
  // order_id (initial state / post-reset is benign).
  if (active_order_id_.empty() || state.order_id.empty())
  {
    return false;
  }

  if (state.order_id == active_order_id_)
  {
    mismatch_count_ = 0;
    return false;
  }

  ++mismatch_count_;
  if (mismatch_count_ >= mismatch_threshold_)
  {
    VDA5050_WARN(
      "[OrderLifecycle] {} {}-strike order_id mismatch (state={}, "
      "expected={}); clearing stale order tracking",
      agv_id_, mismatch_threshold_, state.order_id, active_order_id_);
    active_order_.reset();
    active_order_id_.clear();
    active_order_update_id_ = 0;
    pending_updates_.clear();
    order_complete_ = false;
    needs_more_base_ = false;
    mismatch_count_ = 0;
    return true;
  }

  VDA5050_WARN(
    "[OrderLifecycle] {} order_id mismatch [{}/{}] (state={}, expected={})",
    agv_id_, mismatch_count_, mismatch_threshold_, state.order_id,
    active_order_id_);
  return false;
}

std::vector<vda5050_core::types::Order>
OrderLifecycleManager::drain_pending_locked(
  const vda5050_core::types::State& state)
{
  std::vector<vda5050_core::types::Order> ready;

  while (!pending_updates_.empty())
  {
    const auto& candidate = pending_updates_.front();

    // Cond 1: order_id must match active.
    if (!active_order_ || candidate.order_id != active_order_id_)
    {
      break;
    }

    if (candidate.nodes.empty())
    {
      VDA5050_ERROR(
        "[OrderLifecycle] {} pending update {} has empty nodes; dropping",
        agv_id_, candidate.order_update_id);
      pending_updates_.pop_front();
      continue;
    }

    const uint32_t first_seq = candidate.nodes.front().sequence_id;

    // Cond 2: AGV has passed this update's stitch point — unrecoverable
    // (last_node_sequence_id only advances). Discard the dead head so it
    // cannot starve later updates, and keep draining.
    if (!order_complete_ && first_seq < state.last_node_sequence_id)
    {
      VDA5050_WARN(
        "[OrderLifecycle] {} dropping pending update {}; AGV already past its "
        "stitch point (seq {} < {})",
        agv_id_, candidate.order_update_id, first_seq,
        state.last_node_sequence_id);
      pending_updates_.pop_front();
      continue;
    }

    // Cond 3: AGV has not yet reached the stitch point — wait (keep queued).
    if (!order_complete_ && first_seq != state.last_node_sequence_id)
    {
      break;
    }

    // Cond 4: AGV has confirmed the previous order_update_id.
    if (state.order_update_id < active_order_update_id_)
    {
      break;
    }

    // All conditions satisfied — validate structurally via combine_order,
    // but publish the candidate as-is (spec-strict:
    // master must not retransmit base nodes, only re-send the stitch
    // node + extension; AGV merges base+update internally). The merged
    // result from combine_order is discarded — we only use it for its
    // structural validation side-effect.
    //
    // Lifecycle adoption (`adopt_active_locked_`) is deferred to
    // `record_published`, which fires on the queue thread AFTER the
    // publish chain succeeds. This avoids the stitcher's duplicate
    // check tripping when the drained candidate hits the publish path.
    auto combined =
      combine_order(*active_order_, candidate, state.last_node_sequence_id);
    vda5050_core::types::Order to_publish = candidate;
    pending_updates_.pop_front();
    if (!combined)
    {
      VDA5050_ERROR(
        "[OrderLifecycle] {} combine_order failed for pending update {}; "
        "dropping ({} errors)",
        agv_id_, candidate.order_update_id, combined.errors.size());
      continue;
    }
    ready.push_back(std::move(to_publish));
  }

  return ready;
}

void OrderLifecycleManager::adopt_active_locked(
  const vda5050_core::types::Order& order)
{
  active_order_ = order;
  active_order_id_ = order.order_id;
  active_order_update_id_ = order.order_update_id;
}

}  // namespace vda5050_core::master
