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

vda5050_core::types::Error make_error(
  const vda5050_core::types::Order& candidate, const std::string& description,
  std::vector<vda5050_core::types::ErrorReference> extra_refs = {})
{
  std::vector<vda5050_core::types::ErrorReference> refs;
  refs.reserve(2 + extra_refs.size());
  refs.push_back({errors::RefOrderId, candidate.order_id});
  refs.push_back(
    {errors::RefOrderUpdateId, std::to_string(candidate.order_update_id)});
  for (auto& r : extra_refs) refs.push_back(std::move(r));
  return errors::create_error(errors::OrderUpdateError, description, refs);
}

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

  if (!snapshot.has_active)
  {
    return res;  // SEND_NOW (default)
  }

  if (candidate.nodes.empty())
  {
    reject("Candidate order has no nodes");
    return res;
  }

  // Different order_id: fresh assignment if the prior is complete, else reject.
  if (candidate.order_id != snapshot.order_id)
  {
    if (snapshot.order_complete)
    {
      return res;  // fresh assignment
    }
    reject(
      "Candidate order_id does not match active order_id; cancel "
      "the active order before sending a different order");
    return res;
  }

  // Duplicate update_id — already applied; ignore as a no-op.
  if (candidate.order_update_id == snapshot.order_update_id)
  {
    res.decision = StitchDecision::IGNORE;
    return res;
  }

  if (candidate.order_update_id < snapshot.order_update_id)
  {
    reject("Candidate order_update_id is lower than active order_update_id");
    return res;
  }

  // No State yet: can't evaluate timing — queue and retry.
  if (snapshot.state_order_id.empty())
  {
    queue(
      GuardFailure::NO_STATE_YET,
      "AGV has not yet reported any State; queued until it does");
    return res;
  }

  // No released base node — combine_order would reject too.
  const auto stitch_seq = last_released_seq(snapshot.nodes);
  if (!stitch_seq.has_value())
  {
    reject("Active order has no released base node");
    return res;
  }

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

  // Guard 2: AGV must not have passed the stitch point. Reaching it is not
  // required — an update may be sent ahead while the AGV drives toward it.
  if (!relax_progress_guards && snapshot.last_node_sequence_id > *stitch_seq)
  {
    reject(
      "AGV has already passed the stitch point",
      {{errors::RefSequenceId,
        std::to_string(snapshot.last_node_sequence_id)}});
    return res;
  }

  // Guard 3: AGV has confirmed the previous order_update_id. Always enforced
  // (even when complete) so a new update can't race a prior unconfirmed one.
  if (snapshot.state_order_update_id < snapshot.order_update_id)
  {
    queue(
      GuardFailure::PREV_UPDATE_NOT_CONFIRMED,
      "AGV has not yet confirmed the previous order_update_id");
    return res;
  }

  return res;  // SEND_NOW (default)
}

}  // namespace vda5050_core::master
