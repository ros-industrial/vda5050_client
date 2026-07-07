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

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/master/order/order_stitcher.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/order.hpp"

namespace vda5050_core::master {
namespace test {

namespace {

constexpr const char* kOrderId = "ORD_A";

vda5050_core::types::Node make_node(
  const std::string& id, uint32_t seq, bool released)
{
  vda5050_core::types::Node n;
  n.node_id = id;
  n.sequence_id = seq;
  n.released = released;
  return n;
}

vda5050_core::types::Order make_candidate(
  const std::string& order_id, uint32_t order_update_id)
{
  vda5050_core::types::Order o;
  o.order_id = order_id;
  o.order_update_id = order_update_id;
  o.nodes = {make_node("STITCH", 2, true), make_node("EXT", 6, true)};
  return o;
}

// Active order: 3 nodes — N0/N1 released base (last released = N1@2),
// N2 horizon. last_node_sequence_id = 2 by default (parked at stitch).
ActiveOrderSnapshot make_active_snapshot(
  const std::string& order_id = kOrderId, uint32_t order_update_id = 0,
  uint32_t state_order_update_id = 0, uint32_t last_node_sequence_id = 2,
  const std::string& state_order_id = std::string(kOrderId),
  bool order_complete = false)
{
  ActiveOrderSnapshot s;
  s.has_active = true;
  s.order_id = order_id;
  s.order_update_id = order_update_id;
  s.nodes = {
    make_node("N0", 0, true), make_node("N1", 2, true),
    make_node("N2", 4, false)};
  s.last_node_sequence_id = last_node_sequence_id;
  s.state_order_update_id = state_order_update_id;
  s.state_order_id = state_order_id;
  s.order_complete = order_complete;
  return s;
}

bool every_error_is_order_update(const StitchResult& res)
{
  for (const auto& e : res.errors)
  {
    if (e.error_type != vda5050_core::errors::OrderUpdateError) return false;
  }
  return true;
}

}  // namespace

// =============================================================================
// Pre-flight (REJECT path)
// =============================================================================
TEST(OrderStitcher, NoActiveOrder_SendsNow)
{
  OrderStitcher stitcher;
  ActiveOrderSnapshot snap;  // has_active = false (default)
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::SEND_NOW);
  EXPECT_TRUE(res.errors.empty());
}

TEST(OrderStitcher, DifferentOrderIdWithActive_Rejects)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot();
  auto res = stitcher.decide(make_candidate("OTHER", 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
  ASSERT_FALSE(res.errors.empty());
  EXPECT_TRUE(every_error_is_order_update(res));
}

// New order with different order_id after the prior order has completed
// should pass through the stitcher untouched — the lifecycle's stale
// has_active flag must not block a legitimate fresh assignment.
// Without this, master-side state confusion (AGV legitimately keeps
// state.order_id reporting the finished order per §6.6.1) makes every
// subsequent order_id appear as a stitch conflict.
TEST(OrderStitcher, DifferentOrderIdAfterComplete_SendsNow)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot(
    /*order_id=*/kOrderId, /*order_update_id=*/0,
    /*state_order_update_id=*/0, /*last_node_sequence_id=*/2,
    /*state_order_id=*/std::string(kOrderId), /*order_complete=*/true);
  auto res = stitcher.decide(make_candidate("OTHER", 0), snap);
  EXPECT_EQ(res.decision, StitchDecision::SEND_NOW);
  EXPECT_TRUE(res.errors.empty());
}

TEST(OrderStitcher, DuplicateUpdateId_Rejects)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot(kOrderId, /*update_id=*/3);
  auto res = stitcher.decide(make_candidate(kOrderId, 3), snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
}

TEST(OrderStitcher, BackwardUpdateId_Rejects)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot(kOrderId, /*update_id=*/5);
  auto res = stitcher.decide(make_candidate(kOrderId, 4), snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
}

TEST(OrderStitcher, EmptyCandidateNodes_Rejects)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot();
  vda5050_core::types::Order candidate;
  candidate.order_id = kOrderId;
  candidate.order_update_id = 1;
  // candidate.nodes intentionally empty
  auto res = stitcher.decide(candidate, snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
}

TEST(OrderStitcher, NoStateYet_Rejects)
{
  OrderStitcher stitcher;
  // state_order_id empty → no State has been received yet.
  auto snap = make_active_snapshot(kOrderId, 0, 0, 2, /*state_order_id=*/"");
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
}

TEST(OrderStitcher, ActiveOrderHasNoReleasedBaseNode_Rejects)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot();
  // Force all nodes unreleased.
  for (auto& n : snap.nodes) n.released = false;
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
}

// =============================================================================
// 4 FIWARE guards (QUEUE_PENDING path)
// =============================================================================
TEST(OrderStitcher, Cond1_StateOrderIdMismatch_Queues)
{
  OrderStitcher stitcher;
  auto snap =
    make_active_snapshot(kOrderId, 0, 0, 2, /*state_order_id=*/"OTHER_ORDER");
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::QUEUE_PENDING);
  EXPECT_EQ(res.first_failed_guard, GuardFailure::ORDER_ID_MISMATCH);
}

TEST(OrderStitcher, Cond2_AgvPassedStitchPoint_Rejects)
{
  OrderStitcher stitcher;
  // stitch_seq = 2 (last released base node N1). State reports seq = 4.
  // A passed stitch is unrecoverable — reject (not queue) so a stale update
  // cannot jam the pending queue.
  auto snap = make_active_snapshot(kOrderId, 0, 0, /*last_node_sequence_id=*/4);
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::REJECT);
}

TEST(OrderStitcher, Cond3_AgvNotYetAtStitchPoint_Queues)
{
  OrderStitcher stitcher;
  // stitch_seq = 2. State reports seq = 0 (AGV at start).
  auto snap = make_active_snapshot(kOrderId, 0, 0, /*last_node_sequence_id=*/0);
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::QUEUE_PENDING);
  EXPECT_EQ(res.first_failed_guard, GuardFailure::STITCH_NOT_REACHED);
}

TEST(OrderStitcher, Cond4_PreviousUpdateIdNotConfirmed_Queues)
{
  OrderStitcher stitcher;
  // active.update_id = 5, state.update_id = 3 (not yet caught up).
  auto snap = make_active_snapshot(
    kOrderId, /*update_id=*/5, /*state_update_id=*/3, /*last_seq=*/2);
  auto res = stitcher.decide(make_candidate(kOrderId, 6), snap);
  EXPECT_EQ(res.decision, StitchDecision::QUEUE_PENDING);
  EXPECT_EQ(res.first_failed_guard, GuardFailure::PREV_UPDATE_NOT_CONFIRMED);
}

// =============================================================================
// Order-complete relaxation
// =============================================================================
TEST(OrderStitcher, OrderCompleteRelaxesCond2And3_SendsNow)
{
  OrderStitcher stitcher;
  // Active order complete; cond 2/3 don't apply. last_node_seq is past
  // the stitch point but order_complete relaxes.
  auto snap = make_active_snapshot(
    kOrderId, /*update_id=*/0, /*state_update_id=*/0,
    /*last_seq=*/4, /*state_order_id=*/std::string(kOrderId),
    /*order_complete=*/true);
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::SEND_NOW);
}

TEST(OrderStitcher, OrderCompleteStillEnforcesCond4_Queues)
{
  OrderStitcher stitcher;
  // Order complete BUT AGV hasn't confirmed prior update_id.
  auto snap = make_active_snapshot(
    kOrderId, /*update_id=*/5, /*state_update_id=*/3,
    /*last_seq=*/4, /*state_order_id=*/std::string(kOrderId),
    /*order_complete=*/true);
  auto res = stitcher.decide(make_candidate(kOrderId, 6), snap);
  EXPECT_EQ(res.decision, StitchDecision::QUEUE_PENDING);
  EXPECT_EQ(res.first_failed_guard, GuardFailure::PREV_UPDATE_NOT_CONFIRMED);
}

// =============================================================================
// Boundary + happy path + error content
// =============================================================================
TEST(OrderStitcher, ParkedAtStitchPoint_SendsNow)
{
  OrderStitcher stitcher;
  // last_node_sequence_id == stitch_seq (2). All other guards pass.
  auto snap = make_active_snapshot(kOrderId, 0, 0, /*last_node_sequence_id=*/2);
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::SEND_NOW);
}

TEST(OrderStitcher, AllGuardsPass_SendsNow)
{
  OrderStitcher stitcher;
  auto snap = make_active_snapshot();
  auto res = stitcher.decide(make_candidate(kOrderId, 1), snap);
  EXPECT_EQ(res.decision, StitchDecision::SEND_NOW);
  EXPECT_TRUE(res.errors.empty());
  EXPECT_EQ(res.first_failed_guard, GuardFailure::NONE);
}

TEST(OrderStitcher, ErrorContentSurfacesOrderUpdateError)
{
  OrderStitcher stitcher;
  // Force a REJECT path and a QUEUE path; both must use OrderUpdateError
  // and include RefOrderId + RefOrderUpdateId.
  auto snap_reject = make_active_snapshot(kOrderId, 5);
  auto reject_res = stitcher.decide(make_candidate(kOrderId, 4), snap_reject);
  ASSERT_EQ(reject_res.decision, StitchDecision::REJECT);
  ASSERT_FALSE(reject_res.errors.empty());
  EXPECT_TRUE(every_error_is_order_update(reject_res));

  // Stitch not yet reached (seq 1 < stitch_seq 2) — a genuine QUEUE case.
  auto snap_queue =
    make_active_snapshot(kOrderId, 0, 0, /*last_node_sequence_id=*/1);
  auto queue_res = stitcher.decide(make_candidate(kOrderId, 1), snap_queue);
  ASSERT_EQ(queue_res.decision, StitchDecision::QUEUE_PENDING);
  ASSERT_FALSE(queue_res.errors.empty());
  EXPECT_TRUE(every_error_is_order_update(queue_res));

  // Inspect refs of one error.
  bool found_order_id = false, found_update_id = false;
  for (const auto& ref : queue_res.errors.front().error_references.value_or(
         std::vector<vda5050_core::types::ErrorReference>{}))
  {
    if (ref.reference_key == vda5050_core::errors::RefOrderId)
      found_order_id = true;
    if (ref.reference_key == vda5050_core::errors::RefOrderUpdateId)
      found_update_id = true;
  }
  EXPECT_TRUE(found_order_id);
  EXPECT_TRUE(found_update_id);
}

}  // namespace test
}  // namespace vda5050_core::master
