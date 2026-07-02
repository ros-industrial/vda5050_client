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

#include "vda5050_core/master/order/order_stitcher.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::master {

namespace {

using vda5050_core::errors::create_error;
using vda5050_core::errors::OrderUpdateError;
using vda5050_core::errors::RefOrderId;
using vda5050_core::errors::RefOrderUpdateId;
using vda5050_core::errors::RefSequenceId;

// Build an OrderUpdateError with order_id + order_update_id refs attached.
vda5050_core::types::Error make_error(
  const vda5050_core::types::Order& candidate, const std::string& description,
  std::vector<vda5050_core::types::ErrorReference> extra_refs = {})
{
  std::vector<vda5050_core::types::ErrorReference> refs;
  refs.reserve(2 + extra_refs.size());
  refs.push_back({RefOrderId, candidate.order_id});
  refs.push_back({RefOrderUpdateId, std::to_string(candidate.order_update_id)});
  for (auto& r : extra_refs) refs.push_back(std::move(r));
  return create_error(OrderUpdateError, description, refs);
}

// Find sequence_id of the last released base node in `nodes`.
// Returns nullopt if no released node exists.
std::optional<uint32_t> last_released_seq(
  const std::vector<vda5050_core::types::Node>& nodes)
{
  std::optional<uint32_t> result;
  for (const auto& n : nodes)
  {
    if (n.released) result = n.sequence_id;
  }
  return result;
}

}  // namespace

StitchResult OrderStitcher::decide(
  const vda5050_core::types::Order& candidate,
  const ActiveOrderSnapshot& snapshot) const
{
  StitchResult res;

  auto reject = [&](
                  const std::string& description,
                  std::vector<vda5050_core::types::ErrorReference> refs = {}) {
    res.decision = StitchDecision::REJECT;
    res.errors.push_back(make_error(candidate, description, std::move(refs)));
  };

  auto queue = [&](
                 GuardFailure guard, const std::string& description,
                 std::vector<vda5050_core::types::ErrorReference> refs = {}) {
    res.decision = StitchDecision::QUEUE_PENDING;
    res.first_failed_guard = guard;
    res.errors.push_back(make_error(candidate, description, std::move(refs)));
  };

  // ===========================================================
  // Pre-flight: structural and policy checks. Any failure here
  // is REJECT (no amount of waiting fixes them).
  // ===========================================================

  // No active order — fresh publish, nothing to stitch onto.
  if (!snapshot.has_active)
  {
    return res;  // SEND_NOW (default)
  }

  // An active order always has at least one node.
  if (candidate.nodes.empty())
  {
    reject("Candidate order has no nodes");
    return res;
  }

  // Different order_id. Two cases:
  //   1. The prior order is COMPLETE — this is a fresh assignment, not
  //      a stitch. Per the spec, the AGV legitimately keeps
  //      its state.order_id reporting the finished order until a new
  //      one is accepted, so seeing a new order_id here is normal.
  //      Fall through (SEND_NOW); the publisher chain treats it as a
  //      new order regardless of the stale `has_active` tracking flag.
  //   2. The prior order is still in flight — concurrent orders aren't
  //      supported in v2.0.0; FMS must cancel via cancelOrder
  //      instantAction first.
  if (candidate.order_id != snapshot.order_id)
  {
    if (snapshot.order_complete)
    {
      return res;  // SEND_NOW — fresh assignment, bypass stitch guards
    }
    reject(
      "Candidate order_id does not match active order_id; cancel "
      "the active order before sending a different order");
    return res;
  }

  // Duplicate (same id + same update_id) — the spec says ignore it; we
  // surface as REJECT so the caller sees the diagnostic.
  if (candidate.order_update_id == snapshot.order_update_id)
  {
    reject("Duplicate order_update_id; suppressed as already applied");
    return res;
  }

  // Backward update_id — a stale or out-of-order update.
  if (candidate.order_update_id < snapshot.order_update_id)
  {
    reject("Candidate order_update_id is lower than active order_update_id");
    return res;
  }

  // Cannot evaluate timing guards without a State message yet.
  if (snapshot.state_order_id.empty())
  {
    reject("Cannot evaluate stitch guards: AGV has not yet reported any State");
    return res;
  }

  // Active order with no released base node: combine_order would also
  // reject this; mirrors lifecycle behavior.
  const auto stitch_seq = last_released_seq(snapshot.nodes);
  if (!stitch_seq.has_value())
  {
    reject("Active order has no released base node");
    return res;
  }

  // ===========================================================
  // 4 FIWARE guards. Any failure is
  // QUEUE_PENDING — the AGV will eventually catch up and a future
  // State message will release the queued update.
  // ===========================================================

  const bool relax_progress_guards = snapshot.order_complete;

  // Guard 1: AGV is on the correct order_id.
  if (snapshot.state_order_id != snapshot.order_id)
  {
    queue(
      GuardFailure::ORDER_ID_MISMATCH,
      "AGV not yet on the active order_id (state reports a different "
      "order_id)");
    return res;
  }

  // Guard 2: AGV has not passed the stitch point. A passed stitch is
  // unrecoverable — last_node_sequence_id only advances within an order, so
  // the AGV can never return to the anchor. Reject (not queue) so a stale
  // update cannot sit at the pending-queue head and starve later updates.
  // Strict `>` — parked-at-stitch-node is fine (matches combine_order).
  // Relaxed when the active order is complete: any future-segment update
  // can be forwarded immediately.
  if (!relax_progress_guards && snapshot.last_node_sequence_id > *stitch_seq)
  {
    reject(
      "AGV has already passed the stitch point",
      {{RefSequenceId, std::to_string(snapshot.last_node_sequence_id)}});
    return res;
  }

  // Guard 3: AGV has reached the stitch point.
  if (!relax_progress_guards && snapshot.last_node_sequence_id != *stitch_seq)
  {
    queue(
      GuardFailure::STITCH_NOT_REACHED,
      "AGV has not yet reached the stitch point",
      {{RefSequenceId, std::to_string(snapshot.last_node_sequence_id)}});
    return res;
  }

  // Guard 4: AGV has confirmed the previous order_update_id.
  // Always enforced (even when order_complete == true), to avoid a new
  // update racing past an unconfirmed prior one.
  if (snapshot.state_order_update_id < snapshot.order_update_id)
  {
    queue(
      GuardFailure::PREV_UPDATE_NOT_CONFIRMED,
      "AGV has not yet confirmed the previous order_update_id");
    return res;
  }

  // All pre-flight + guards passed — forward to publisher chain.
  return res;  // SEND_NOW (default)
}

}  // namespace vda5050_core::master
