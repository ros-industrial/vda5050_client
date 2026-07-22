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
#include "vda5050_core/master/order/order_stitcher.hpp"

namespace vda5050_core::master {

namespace {

vda5050_core::types::Error make_combine_error(
  const std::string& description, const std::string& order_id,
  std::vector<vda5050_core::types::ErrorReference> extra_refs = {})
{
  std::vector<vda5050_core::types::ErrorReference> refs;
  refs.reserve(1 + extra_refs.size());
  refs.push_back({errors::RefOrderId, order_id});
  for (auto& r : extra_refs) refs.push_back(std::move(r));
  return errors::create_error(errors::OrderUpdateError, description, refs);
}

bool all_actions_terminal(
  const std::vector<vda5050_core::types::ActionState>& actions)
{
  return std::all_of(
    actions.begin(), actions.end(),
    [](const vda5050_core::types::ActionState& a) {
      return a.action_status == vda5050_core::types::ActionStatus::FINISHED ||
             a.action_status == vda5050_core::types::ActionStatus::FAILED;
    });
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

  if (base.order_id != update.order_id)
  {
    fail(
      "Update order_id does not match base order_id",
      {{errors::RefOrderId, update.order_id}});
    return res;
  }

  if (update.order_update_id <= base.order_update_id)
  {
    fail(
      "Update order_update_id is not greater than base order_update_id",
      {{errors::RefOrderUpdateId, std::to_string(update.order_update_id)}});
    return res;
  }

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

  // preserved = last released base node (the stitch anchor) + horizon.
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

  // AGV must not have passed the stitch point (parked at it is fine).
  if (
    old_base_last != nullptr &&
    last_node_sequence_id > old_base_last->sequence_id)
  {
    fail(
      "AGV has already passed the stitch point",
      {{errors::RefSequenceId, std::to_string(last_node_sequence_id)}});
    return res;
  }

  uint32_t preserved_max_seq =
    preserved.empty() ? 0 : preserved.back().sequence_id;

  for (const auto& nu : update.nodes)
  {
    if (old_base_last != nullptr && nu.sequence_id < old_base_last->sequence_id)
    {
      fail(
        "Update attempts to alter a released base node; the base cannot be "
        "changed",
        {{errors::RefNodeId, nu.node_id}});
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
          {{errors::RefNodeId, nu.node_id}});
      }
      // Either way, do NOT replace — base copy already in `preserved`.
      continue;
    }

    if (nu.sequence_id > preserved_max_seq)
    {
      preserved.push_back(nu);
      preserved_max_seq = nu.sequence_id;
      continue;
    }

    auto it = std::find_if(
      preserved.begin(), preserved.end(),
      [&](const vda5050_core::types::Node& n) {
        return n.sequence_id == nu.sequence_id;
      });

    if (it == preserved.end())
    {
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
          {{errors::RefNodeId, nu.node_id}});
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

  // Keep only horizon (unreleased) edges, mirroring the nodes. The edge into
  // the anchor would dangle — its start node is dropped.
  for (const auto& e : base.edges)
  {
    if (!e.released) preserved_edges.push_back(e);
  }

  uint32_t preserved_edge_max_seq =
    preserved_edges.empty() ? 0 : preserved_edges.back().sequence_id;

  for (const auto& eu : update.edges)
  {
    // An update must not touch a released base edge (at or before the anchor).
    if (has_base_edge && eu.sequence_id <= old_base_last_edge_seq)
    {
      fail(
        "Update attempts to alter a released base edge; the base cannot be "
        "changed",
        {{errors::RefEdgeId, eu.edge_id}});
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
          {{errors::RefEdgeId, eu.edge_id}});
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

  const std::string prior_active_id = active_order_id_;
  const bool same_order =
    !active_order_id_.empty() && active_order_id_ == order.order_id;
  const bool real_extension =
    same_order && order.order_update_id > active_order_update_id_;

  if (real_extension && active_order_.has_value())
  {
    // Adopt the full merged base+horizon, not the sparse wire update, so
    // later checks see the true route.
    if (merged.has_value())
    {
      adopt_active_locked(*merged);
    }
    else
    {
      auto combined =
        combine_order(*active_order_, order, last_node_sequence_id_);
      adopt_active_locked(combined ? combined.order : order);
    }
    needs_more_base_ = false;
    order_complete_ = false;  // an extension re-opens the order
  }
  else
  {
    adopt_active_locked(order);
    if (!same_order)
    {
      // New order: reset sticky flags and grace the prior order_id's handover.
      prev_active_order_id_ = prior_active_id;
      order_complete_ = false;
      needs_more_base_ = false;
      mismatch_count_ = 0;
      handover_lag_count_ = 0;
    }
  }
}

std::vector<vda5050_core::types::Order> OrderLifecycleManager::on_state_update(
  const vda5050_core::types::State& state,
  std::optional<std::string>* just_completed_order_id)
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);

  // Drop stale / duplicate / out-of-order State before diffing.
  if (have_state_baseline_ && state.header.header_id < last_state_header_id_)
  {
    return {};
  }
  last_state_header_id_ = state.header.header_id;
  have_state_baseline_ = true;

  last_node_sequence_id_ = state.last_node_sequence_id;
  state_order_update_id_ = state.order_update_id;
  last_state_order_id_ = state.order_id;

  if (state.new_base_request.value_or(false))
  {
    needs_more_base_ = true;
  }

  if (tick_mismatch(state)) return {};

  // The order/update gate rejects a lagging State from a just-replaced order.
  if (
    active_order_ && !active_order_->nodes.empty() &&
    state.order_id == active_order_id_ &&
    state.order_update_id == active_order_update_id_)
  {
    const auto& last = active_order_->nodes.back();
    if (
      state.last_node_sequence_id == last.sequence_id &&
      state.last_node_id == last.node_id && state.node_states.empty() &&
      state.edge_states.empty() && all_actions_terminal(state.action_states))
    {
      if (!order_complete_)
      {
        VDA5050_INFO(
          "[OrderLifecycle] {} order {} (update {}) complete", agv_id_,
          active_order_id_, active_order_update_id_);
        if (just_completed_order_id)
          *just_completed_order_id = active_order_id_;
      }
      order_complete_ = true;
    }
  }

  return drain_pending_locked(state);
}

vda5050_core::types::Error OrderLifecycleManager::pending_queue_full_error(
  const std::string& order_id)
{
  return make_combine_error(
    "Pending update queue is full; order dropped.", order_id);
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
  pending_updates_.push_back(PendingUpdate{update, 0});
  return true;
}

void OrderLifecycleManager::requeue_pending_front(
  const std::vector<vda5050_core::types::Order>& updates)
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  // Reverse so the first update ends up at the front.
  for (auto it = updates.rbegin(); it != updates.rend(); ++it)
  {
    pending_updates_.push_front(PendingUpdate{*it, 0});
  }
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
  last_state_header_id_ = 0;
  have_state_baseline_ = false;
  prev_active_order_id_.clear();
  order_complete_ = false;
  needs_more_base_ = false;
  pending_updates_.clear();
  mismatch_count_ = 0;
  handover_lag_count_ = 0;
}

void OrderLifecycleManager::clear_pending()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  pending_updates_.clear();
}

void OrderLifecycleManager::reset_state_baseline()
{
  std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  last_state_header_id_ = 0;
  have_state_baseline_ = false;
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
  // No active order or empty state order_id (initial / post-reset) is benign.
  if (active_order_id_.empty() || state.order_id.empty())
  {
    return false;
  }

  if (state.order_id == active_order_id_)
  {
    mismatch_count_ = 0;
    handover_lag_count_ = 0;
    return false;
  }

  // Handover lag: still reporting the just-replaced order is expected briefly,
  // but bounded so a permanently-stuck AGV still trips the mismatch recovery.
  if (
    !prev_active_order_id_.empty() && state.order_id == prev_active_order_id_ &&
    ++handover_lag_count_ <= kMaxHandoverLagStates)
  {
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
    handover_lag_count_ = 0;
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
  if (pending_updates_.empty() || !active_order_) return ready;

  // Reuse OrderStitcher::decide so drain and the pre-flight can't drift. The
  // active order and this State are constant across the loop — build once.
  ActiveOrderSnapshot snap;
  snap.has_active = true;
  snap.order_id = active_order_id_;
  snap.order_update_id = active_order_update_id_;
  snap.nodes = active_order_->nodes;
  snap.last_node_sequence_id = state.last_node_sequence_id;
  snap.state_order_update_id = state.order_update_id;
  snap.state_order_id = state.order_id;
  snap.order_complete = order_complete_;

  OrderStitcher stitcher;
  while (!pending_updates_.empty())
  {
    auto& front = pending_updates_.front();
    const auto decision = stitcher.decide(front.order, snap).decision;

    if (decision == StitchDecision::QUEUE_PENDING)
    {
      // Not ready yet; wait, aging the head out if it never clears. FIFO —
      // don't skip past the head.
      if (++front.waits > kDefaultPendingMaxWaits)
      {
        VDA5050_WARN(
          "[OrderLifecycle] {} dropping pending update {}; stitch conditions "
          "not met in {} states",
          agv_id_, front.order.order_update_id, kDefaultPendingMaxWaits);
        pending_updates_.pop_front();
        continue;
      }
      break;
    }

    if (decision != StitchDecision::SEND_NOW)
    {
      // REJECT / IGNORE — unstitchable or already applied; drop.
      VDA5050_WARN(
        "[OrderLifecycle] {} dropping pending update {}; no longer stitchable",
        agv_id_, front.order.order_update_id);
      pending_updates_.pop_front();
      continue;
    }

    // SEND_NOW — release, then advance the snapshot to this update so the next
    // one honors the prev-update-not-confirmed guard instead of also releasing.
    ready.push_back(front.order);
    snap.order_update_id = front.order.order_update_id;
    snap.nodes = front.order.nodes;
    pending_updates_.pop_front();
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
