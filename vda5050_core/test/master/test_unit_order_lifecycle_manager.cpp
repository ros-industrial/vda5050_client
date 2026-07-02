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
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master {
namespace test {

namespace {

constexpr const char* kAGV = "ACME/AGV01";
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

vda5050_core::types::Edge make_edge(
  const std::string& id, uint32_t seq, const std::string& from,
  const std::string& to, bool released)
{
  vda5050_core::types::Edge e;
  e.edge_id = id;
  e.sequence_id = seq;
  e.start_node_id = from;
  e.end_node_id = to;
  e.released = released;
  return e;
}

// Base order: 3 nodes (N0, N1 released; N2 horizon), 2 edges between them.
vda5050_core::types::Order make_base_order()
{
  vda5050_core::types::Order o;
  o.order_id = kOrderId;
  o.order_update_id = 0;
  o.nodes = {
    make_node("N0", 0, true), make_node("N1", 2, true),
    make_node("N2", 4, false)};
  o.edges = {
    make_edge("E0", 1, "N0", "N1", true),
    make_edge("E1", 3, "N1", "N2", false)};
  return o;
}

vda5050_core::types::State make_state(
  const std::string& order_id, uint32_t order_update_id,
  const std::string& last_node_id, uint32_t last_node_seq)
{
  vda5050_core::types::State s;
  s.order_id = order_id;
  s.order_update_id = order_update_id;
  s.last_node_id = last_node_id;
  s.last_node_sequence_id = last_node_seq;
  return s;
}

}  // namespace

// =============================================================================
// combine_order
// =============================================================================
TEST(CombineOrder, AppendsExtensionPastBaseLastReleased)
{
  auto base = make_base_order();
  // Update extends from the stitch point (N1, seq=2) to a new node N3.
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  update.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  update.edges = {make_edge("E2", 5, "N1", "N3", true)};

  auto res = combine_order(base, update, /*last_node_seq=*/0);
  ASSERT_TRUE(static_cast<bool>(res)) << "errors=" << res.errors.size();
  EXPECT_EQ(res.order.order_id, std::string(kOrderId));
  EXPECT_EQ(res.order.order_update_id, 1u);
  // N1 (preserved base last) + N2 (horizon) + N3 (extension) = 3
  ASSERT_EQ(res.order.nodes.size(), 3u);
  EXPECT_EQ(res.order.nodes[0].node_id, "N1");
  EXPECT_EQ(res.order.nodes[1].node_id, "N2");
  EXPECT_EQ(res.order.nodes[2].node_id, "N3");
}

TEST(CombineOrder, RejectsLowerOrEqualUpdateId)
{
  auto base = make_base_order();
  base.order_update_id = 5;
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 5;  // not greater
  auto res = combine_order(base, update, 0);
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::OrderUpdateError);
}

TEST(CombineOrder, RejectsAlteringReleasedBaseNode)
{
  auto base = make_base_order();
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  // Try to alter N0 (released base, seq=0).
  update.nodes = {make_node("N0_NEW", 0, true)};

  auto res = combine_order(base, update, 0);
  EXPECT_FALSE(static_cast<bool>(res));
}

TEST(CombineOrder, RejectsUnreleaseOfReleasedHorizon)
{
  // Build a base where the horizon contains a released node (degenerate
  // but possible in edge cases). Update with same seq + released=false.
  vda5050_core::types::Order base;
  base.order_id = kOrderId;
  base.order_update_id = 0;
  base.nodes = {
    make_node("N0", 0, true), make_node("N1", 2, true),
    make_node("N2", 4, true)};  // all released

  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  // N1 is the new "stitch node" (seq 2) — must be byte-equal.
  update.nodes = {
    make_node("N1", 2, true),
    make_node("N2", 4, false)};  // attempt to un-release

  auto res = combine_order(base, update, 0);
  EXPECT_FALSE(static_cast<bool>(res));
}

TEST(CombineOrder, RejectsStitchNodeContentChange)
{
  auto base = make_base_order();
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  // Stitch node N1 with different node_id — content change.
  vda5050_core::types::Node stitch_changed = make_node("N1_RENAMED", 2, true);
  update.nodes = {stitch_changed, make_node("N3", 6, true)};

  auto res = combine_order(base, update, 0);
  EXPECT_FALSE(static_cast<bool>(res));
}

TEST(CombineOrder, ReplacesUnreleasedHorizon)
{
  auto base = make_base_order();
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  // Stitch node identical, replace N2 horizon (seq=4) with N2_v2 (still
  // unreleased) — note: identical node_id so it's structurally a content
  // edit; we test "in-place update of horizon" by keeping released=false.
  update.nodes = {
    make_node("N1", 2, true),
    make_node("N2", 4, false)};  // same struct = idempotent replace

  auto res = combine_order(base, update, 0);
  EXPECT_TRUE(static_cast<bool>(res));
  ASSERT_EQ(res.order.nodes.size(), 2u);
  EXPECT_EQ(res.order.nodes[0].node_id, "N1");
  EXPECT_EQ(res.order.nodes[1].node_id, "N2");
}

TEST(CombineOrder, RejectsMismatchedOrderId)
{
  auto base = make_base_order();
  vda5050_core::types::Order update;
  update.order_id = "OTHER";
  update.order_update_id = 1;
  auto res = combine_order(base, update, 0);
  EXPECT_FALSE(static_cast<bool>(res));
}

TEST(CombineOrder, AppendsExtensionEdges)
{
  // Edge-side parallel of AppendsExtensionPastBaseLastReleased. Base has
  // E0 (released, seq=1) + E1 (horizon, seq=3); update introduces E2
  // (seq=5) past the horizon. E0 (last released) is preserved as the
  // stitch edge anchor; E1 stays; E2 appended.
  auto base = make_base_order();
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  update.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  update.edges = {
    make_edge("E0", 1, "N0", "N1", true),  // stitch edge — must equal base
    make_edge("E2", 5, "N1", "N3", true)};

  auto res = combine_order(base, update, 0);
  ASSERT_TRUE(static_cast<bool>(res)) << "errors=" << res.errors.size();
  ASSERT_EQ(res.order.edges.size(), 3u);  // E0 + E1 (horizon) + E2
  EXPECT_EQ(res.order.edges[0].edge_id, "E0");
  EXPECT_EQ(res.order.edges[1].edge_id, "E1");
  EXPECT_EQ(res.order.edges[2].edge_id, "E2");
}

TEST(CombineOrder, RejectsAlteringReleasedBaseEdge)
{
  auto base = make_base_order();  // E0 released at seq=1
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  // Try to redefine E0 with a different end_node — should reject.
  update.edges = {make_edge("E0", 1, "N0", "DIFFERENT", true)};
  // (must also pass the node-side stitch — keep N1 identical)
  update.nodes = {make_node("N1", 2, true)};

  auto res = combine_order(base, update, 0);
  EXPECT_FALSE(static_cast<bool>(res));
}

TEST(CombineOrder, RejectsWhenAGVPassedStitchPoint)
{
  // Base has stitch point at N1 (seq=2). AGV state shows it's at seq=4
  // already → past the stitch point. Combine must reject because the
  // stitch can no longer be applied.
  auto base = make_base_order();
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  update.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};

  auto res = combine_order(base, update, /*last_node_seq=*/4);
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::OrderUpdateError);
}

// =============================================================================
// MismatchCounter
// =============================================================================
TEST(MismatchCounter, IncrementsAndRecoversAtThreshold)
{
  OrderLifecycleManager mgr(kAGV, /*threshold=*/3, /*cap=*/8);
  mgr.record_published(make_base_order());
  ASSERT_TRUE(mgr.has_active_order());

  // Three consecutive states with mismatched order_id triggers recovery.
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  EXPECT_TRUE(mgr.has_active_order()) << "still active after 2 mismatches";
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  EXPECT_FALSE(mgr.has_active_order()) << "should clear after 3rd mismatch";
}

TEST(MismatchCounter, ResetsOnMatch)
{
  OrderLifecycleManager mgr(kAGV, /*threshold=*/3, /*cap=*/8);
  mgr.record_published(make_base_order());

  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  // Match — counter resets.
  mgr.on_state_update(make_state(kOrderId, 0, "", 0));
  // Two more mismatches should NOT trigger recovery (counter restarted).
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  EXPECT_TRUE(mgr.has_active_order());
}

TEST(MismatchCounter, SkippedWhenStateOrderIdEmpty)
{
  OrderLifecycleManager mgr(kAGV, /*threshold=*/2, /*cap=*/8);
  mgr.record_published(make_base_order());

  // §6.10.6: empty state.order_id is benign (initial / post-reset).
  for (int i = 0; i < 5; ++i)
  {
    mgr.on_state_update(make_state("", 0, "", 0));
  }
  EXPECT_TRUE(mgr.has_active_order());
}

// =============================================================================
// Pending queue / drain
// =============================================================================
TEST(PendingQueue, ReleasedWhenStateReachesStitch)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());

  // Queue an update whose first node = the stitch point (seq 2).
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  update.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(update));

  // AGV not yet at stitch — no drain.
  auto ready1 = mgr.on_state_update(make_state(kOrderId, 0, "N0", 0));
  EXPECT_TRUE(ready1.empty());
  EXPECT_EQ(mgr.pending_update_count(), 1u);

  // AGV reaches stitch — drain returns the candidate (#19: spec-strict;
  // adoption is deferred to record_published).
  auto ready2 = mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));
  ASSERT_EQ(ready2.size(), 1u);
  EXPECT_EQ(ready2.front().order_update_id, 1u);
  EXPECT_EQ(mgr.pending_update_count(), 0u);
  // Drain alone does NOT advance active — record_published does.
  EXPECT_EQ(mgr.active_order_update_id().value_or(99), 0u)
    << "drain must not adopt; only record_published advances active";
  // Caller then publishes the candidate and records it.
  mgr.record_published(ready2.front());
  EXPECT_EQ(mgr.active_order_update_id().value_or(0), 1u);
}

// record_published adopts the publisher-supplied merged order rather than
// re-combining, so the internal active route is the full base+horizon view
// (not the sparse wire candidate) even if state advanced after publish.
TEST(RecordPublished, AdoptsSuppliedMergedOrder)
{
  OrderLifecycleManager mgr(kAGV);
  auto base = make_base_order();
  mgr.record_published(base);

  // Sparse wire candidate: stitch node (N1@2) + extension (N3@6).
  vda5050_core::types::Order sparse;
  sparse.order_id = kOrderId;
  sparse.order_update_id = 1;
  sparse.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  sparse.edges = {make_edge("E2", 5, "N1", "N3", true)};

  // The merged route the publisher computes (base horizon + extension).
  auto combined = combine_order(base, sparse, /*last_node_seq=*/2);
  ASSERT_TRUE(combined);
  ASSERT_GT(combined.order.nodes.size(), sparse.nodes.size());

  mgr.record_published(sparse, combined.order);
  EXPECT_EQ(mgr.active_order_update_id().value_or(0), 1u);
  // Internal active route is the merged order, not the 2-node candidate.
  EXPECT_EQ(mgr.snapshot().nodes.size(), combined.order.nodes.size());
}

TEST(PendingQueue, BlockedByOrderIdMismatch)
{
  OrderLifecycleManager mgr(kAGV, /*threshold=*/100, /*cap=*/8);
  mgr.record_published(make_base_order());

  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  update.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(update));

  // AGV reports a different order_id — drain blocked.
  auto ready = mgr.on_state_update(make_state("OTHER_ORDER", 0, "N0", 0));
  EXPECT_TRUE(ready.empty());
  EXPECT_EQ(mgr.pending_update_count(), 1u);
}

TEST(PendingQueue, BlockedUntilPrevConfirmed)
{
  // Bump base.order_update_id to simulate a previously confirmed update.
  OrderLifecycleManager mgr(kAGV);
  auto base = make_base_order();
  base.order_update_id = 5;
  mgr.record_published(base);

  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 7;  // strictly higher
  update.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(update));

  // AGV at stitch but reporting older update_id (4 < 5) — blocked.
  auto ready1 = mgr.on_state_update(make_state(kOrderId, 4, "N1", 2));
  EXPECT_TRUE(ready1.empty());
  EXPECT_EQ(mgr.pending_update_count(), 1u);

  // AGV reports update_id == active (confirmed) — drain.
  auto ready2 = mgr.on_state_update(make_state(kOrderId, 5, "N1", 2));
  EXPECT_EQ(ready2.size(), 1u);
  EXPECT_EQ(mgr.pending_update_count(), 0u);
}

TEST(PendingQueue, CapRejectsNewest)
{
  OrderLifecycleManager mgr(kAGV, /*threshold=*/3, /*cap=*/3);
  vda5050_core::types::Order update;
  update.order_id = kOrderId;
  update.order_update_id = 1;
  EXPECT_TRUE(mgr.enqueue_pending_update(update));
  EXPECT_TRUE(mgr.enqueue_pending_update(update));
  EXPECT_TRUE(mgr.enqueue_pending_update(update));
  EXPECT_FALSE(mgr.enqueue_pending_update(update)) << "cap=3 reached";
  EXPECT_EQ(mgr.pending_update_count(), 3u);
}

TEST(PendingQueue, SequentialDrainAcrossStatesAdvancesActiveUpdateId)
{
  // Realistic flow: queue u1, drain it, queue u2 (stitched against the
  // post-drain active), drain it. This pins down that after a drain the
  // active_order_update_id advances and a follow-up update queued
  // *against the new active* drains correctly.
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());

  vda5050_core::types::Order u1;
  u1.order_id = kOrderId;
  u1.order_update_id = 1;
  // Stitch at base's last released (N1@2); extend with N3@6.
  u1.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(u1));

  // State 1: AGV reaches the original stitch (N1@2) on update_id=0.
  auto ready1 = mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));
  ASSERT_EQ(ready1.size(), 1u);
  EXPECT_EQ(ready1[0].order_update_id, 1u);
  // Caller publishes u1 and records it (simulating successful publish).
  mgr.record_published(ready1.front());
  EXPECT_EQ(mgr.active_order_update_id().value_or(0), 1u);

  // After u1 drains+records: active is u1 itself (per #19 spec-strict
  // semantics — active = last-published-on-wire). u1.nodes are
  // [N1@2 rel, N3@6 rel], so the new "last released" is N3@6 — u2
  // must stitch there.
  vda5050_core::types::Order u2;
  u2.order_id = kOrderId;
  u2.order_update_id = 2;
  u2.nodes = {make_node("N3", 6, true), make_node("N4", 8, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(u2));

  // State 2: AGV confirms update_id=1 and is at N3@6 → u2 drains.
  auto ready2 = mgr.on_state_update(make_state(kOrderId, 1, "N3", 6));
  ASSERT_EQ(ready2.size(), 1u);
  EXPECT_EQ(ready2[0].order_update_id, 2u);
  EXPECT_EQ(mgr.pending_update_count(), 0u);
  // Drain doesn't advance — caller records.
  EXPECT_EQ(mgr.active_order_update_id().value_or(99), 1u);
  mgr.record_published(ready2.front());
  EXPECT_EQ(mgr.active_order_update_id().value_or(0), 2u);
}

// =============================================================================
// Order completion
// =============================================================================
TEST(Completion, DetectedWhenLastNodeReached)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());

  // Final node of base = N2, seq=4.
  EXPECT_FALSE(mgr.is_order_complete());
  mgr.on_state_update(make_state(kOrderId, 0, "N2", 4));
  EXPECT_TRUE(mgr.is_order_complete());

  // Repeated matching state stays complete (no double-log; state stays).
  mgr.on_state_update(make_state(kOrderId, 0, "N2", 4));
  EXPECT_TRUE(mgr.is_order_complete());
}

TEST(Completion, NotSetWhenLastNodeIdMismatch)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());

  // seq matches but node_id doesn't — should NOT mark complete.
  mgr.on_state_update(make_state(kOrderId, 0, "WRONG_ID", 4));
  EXPECT_FALSE(mgr.is_order_complete());
}

// =============================================================================
// newBaseRequest
// =============================================================================
TEST(NewBaseRequest, SetsFlagAndClearsOnExtension)
{
  OrderLifecycleManager mgr(kAGV);
  auto base = make_base_order();
  base.order_update_id = 0;
  mgr.record_published(base);
  EXPECT_FALSE(mgr.active_order_needs_more_base());

  // State asserts new_base_request — flag becomes sticky-true.
  vda5050_core::types::State s = make_state(kOrderId, 0, "N0", 0);
  s.new_base_request = true;
  mgr.on_state_update(s);
  EXPECT_TRUE(mgr.active_order_needs_more_base());

  // No-op republish (same update_id) — flag stays.
  mgr.record_published(base);
  EXPECT_TRUE(mgr.active_order_needs_more_base());

  // Real extension (higher update_id) — flag clears.
  auto ext = base;
  ext.order_update_id = 1;
  mgr.record_published(ext);
  EXPECT_FALSE(mgr.active_order_needs_more_base());
}

// =============================================================================
// #19: drain returns candidate, not combined; adoption deferred to
// record_published. Per VDA5050 v2.0.0 §6.6.2:927 master must not retransmit
// base nodes — only re-send the stitch node + extension. AGV merges
// internally.
// =============================================================================
TEST(PendingQueue, DrainReturnsCandidateNotCombined)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());

  // Candidate = update with stitch node + 1 extension. If drain returned
  // the combined order it would have 3 nodes (preserved base last + horizon
  // + extension); the candidate has only 2 (stitch + extension).
  vda5050_core::types::Order candidate;
  candidate.order_id = kOrderId;
  candidate.order_update_id = 1;
  candidate.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(candidate));

  auto ready = mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));
  ASSERT_EQ(ready.size(), 1u);
  // Drain returns candidate as-sent — exactly 2 nodes, no merging.
  EXPECT_EQ(ready.front().nodes.size(), 2u);
  EXPECT_EQ(ready.front().nodes[0].node_id, "N1");
  EXPECT_EQ(ready.front().nodes[1].node_id, "N3");
}

// =============================================================================
// clear() — direct contract test (called by AGV::restart and 3-strike recovery)
// =============================================================================
TEST(LifecycleClear, ClearResetsAllState)
{
  OrderLifecycleManager mgr(kAGV);
  // Drive every field into a non-default state.
  mgr.record_published(make_base_order());
  mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));
  vda5050_core::types::Order pending;
  pending.order_id = kOrderId;
  pending.order_update_id = 1;
  pending.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(pending));

  vda5050_core::types::State s = make_state(kOrderId, 0, "N0", 0);
  s.new_base_request = true;
  mgr.on_state_update(s);

  ASSERT_TRUE(mgr.has_active_order());
  ASSERT_TRUE(mgr.active_order_needs_more_base());
  ASSERT_GT(mgr.pending_update_count(), 0u);

  mgr.clear();

  EXPECT_FALSE(mgr.has_active_order());
  EXPECT_FALSE(mgr.active_order_id().has_value());
  EXPECT_FALSE(mgr.active_order_update_id().has_value());
  EXPECT_FALSE(mgr.is_order_complete());
  EXPECT_FALSE(mgr.active_order_needs_more_base());
  EXPECT_EQ(mgr.pending_update_count(), 0u);

  auto snap = mgr.snapshot();
  EXPECT_FALSE(snap.has_active);
  EXPECT_EQ(snap.last_node_sequence_id, 0u);
  EXPECT_EQ(snap.state_order_update_id, 0u);
  EXPECT_TRUE(snap.state_order_id.empty());
  EXPECT_FALSE(snap.order_complete);
}

// =============================================================================
// 3-strike mismatch must also clear the pending queue
// =============================================================================
TEST(MismatchCounter, RecoveryAtThresholdAlsoClearsPendingQueue)
{
  OrderLifecycleManager mgr(kAGV, /*threshold=*/3, /*cap=*/8);
  mgr.record_published(make_base_order());

  // Queue 2 pending updates while we still have an active order.
  vda5050_core::types::Order u1;
  u1.order_id = kOrderId;
  u1.order_update_id = 1;
  u1.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(u1));
  ASSERT_TRUE(mgr.enqueue_pending_update(u1));
  ASSERT_EQ(mgr.pending_update_count(), 2u);

  // 3 consecutive states with mismatched order_id → recovery.
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  mgr.on_state_update(make_state("OTHER", 0, "", 0));
  mgr.on_state_update(make_state("OTHER", 0, "", 0));

  EXPECT_FALSE(mgr.has_active_order());
  EXPECT_EQ(mgr.pending_update_count(), 0u)
    << "3-strike recovery must also drop queued pending updates";
}

TEST(PendingQueue, DrainDoesNotAdvanceActiveUntilRecordPublished)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());
  ASSERT_EQ(mgr.active_order_update_id().value_or(99), 0u);

  vda5050_core::types::Order candidate;
  candidate.order_id = kOrderId;
  candidate.order_update_id = 1;
  candidate.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  ASSERT_TRUE(mgr.enqueue_pending_update(candidate));

  auto ready = mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));
  ASSERT_EQ(ready.size(), 1u);

  // Drain does NOT touch active_order_update_id — adoption is the
  // caller's responsibility via record_published after a successful
  // publish. This is what prevents the stitcher from rejecting the
  // drained candidate as a duplicate.
  EXPECT_EQ(mgr.active_order_update_id().value_or(99), 0u);

  // Now caller records — adoption fires.
  mgr.record_published(ready.front());
  EXPECT_EQ(mgr.active_order_update_id().value_or(0), 1u);
}

// =============================================================================
// record_published — internal merged-view adoption
// =============================================================================
//
// Wire is spec-strict per VDA5050 v2.0.0 §6.6.2:927 (only stitch + extension
// transmitted). Internal active_order_ must remain the full merged view
// (stitch anchor + horizon + extension) so that next-update validation,
// completion detection, and snapshot consumers see the true active route.

TEST(RecordPublished, UpdateAdoptsMergedBaseHorizonView)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());

  // AGV reaches the stitch point.
  mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));

  // Wire candidate carries only stitch + extension (per §6.6.2:927).
  vda5050_core::types::Order candidate;
  candidate.order_id = kOrderId;
  candidate.order_update_id = 1;
  candidate.nodes = {make_node("N1", 2, true), make_node("N3", 6, true)};
  candidate.edges = {make_edge("E2", 5, "N1", "N3", true)};

  mgr.record_published(candidate);

  auto snap = mgr.snapshot();
  ASSERT_TRUE(snap.has_active);
  EXPECT_EQ(snap.order_update_id, 1u);
  // Merged view = stitch_anchor (N1) + horizon kept from base (N2)
  // + extension (N3). N0 is intentionally dropped — it is pre-stitch
  // executed base, no longer part of the active route.
  ASSERT_EQ(snap.nodes.size(), 3u);
  EXPECT_EQ(snap.nodes[0].node_id, "N1");
  EXPECT_EQ(snap.nodes[1].node_id, "N2");
  EXPECT_EQ(snap.nodes[2].node_id, "N3");
  // Edges: last-released base edge (E0) + horizon kept (E1) + new (E2).
  // Mirrors nodes-stitch-anchor preservation but for edges.
  ASSERT_EQ(snap.edges.size(), 3u);
  EXPECT_EQ(snap.edges[0].edge_id, "E0");
  EXPECT_EQ(snap.edges[1].edge_id, "E1");
  EXPECT_EQ(snap.edges[2].edge_id, "E2");
}

TEST(RecordPublished, SequentialUpdatesGrowAndPromoteHorizon)
{
  // Custom base: 2 released + 2 horizon nodes so we can exercise both
  // horizon promotion and prior-horizon preservation across two updates.
  vda5050_core::types::Order v0;
  v0.order_id = kOrderId;
  v0.order_update_id = 0;
  v0.nodes = {
    make_node("N0", 0, true), make_node("N1", 2, true),
    make_node("N2", 4, false), make_node("N3", 6, false)};
  v0.edges = {
    make_edge("E0", 1, "N0", "N1", true), make_edge("E1", 3, "N1", "N2", false),
    make_edge("E2", 5, "N2", "N3", false)};

  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(v0);

  // AGV reaches stitch point for update1.
  mgr.on_state_update(make_state(kOrderId, 0, "N1", 2));

  // Update 1: promotes N2 horizon→base, adds N4 horizon.
  vda5050_core::types::Order u1;
  u1.order_id = kOrderId;
  u1.order_update_id = 1;
  u1.nodes = {
    make_node("N1", 2, true), make_node("N2", 4, true),
    make_node("N4", 8, false)};
  u1.edges = {
    make_edge("E1", 3, "N1", "N2", true),
    make_edge("E3", 7, "N3", "N4", false)};

  mgr.record_published(u1);

  auto snap1 = mgr.snapshot();
  // Merged: stitch (N1) + horizon retained N3 + promoted N2 + new N4.
  // (N3 from V0 horizon was NOT in u1's candidate but must survive.)
  ASSERT_EQ(snap1.nodes.size(), 4u);
  EXPECT_EQ(snap1.nodes[0].node_id, "N1");
  EXPECT_EQ(snap1.nodes[1].node_id, "N2");
  EXPECT_TRUE(snap1.nodes[1].released)
    << "N2 should be promoted from horizon to base";
  EXPECT_EQ(snap1.nodes[2].node_id, "N3");
  EXPECT_FALSE(snap1.nodes[2].released)
    << "N3 horizon from V0 must survive the update";
  EXPECT_EQ(snap1.nodes[3].node_id, "N4");
  EXPECT_FALSE(snap1.nodes[3].released);

  // AGV reaches stitch point for update2.
  mgr.on_state_update(make_state(kOrderId, 1, "N2", 4));

  // Update 2: promotes N3 horizon→base. Would fail with the bug since
  // first update's adoption would have lost N3.
  vda5050_core::types::Order u2;
  u2.order_id = kOrderId;
  u2.order_update_id = 2;
  u2.nodes = {
    make_node("N2", 4, true), make_node("N3", 6, true),
    make_node("N5", 10, false)};
  u2.edges = {
    make_edge("E2", 5, "N2", "N3", true),
    make_edge("E4", 9, "N4", "N5", false)};

  mgr.record_published(u2);

  auto snap2 = mgr.snapshot();
  // Merged: stitch (N2) + horizon-retained N4 + promoted N3 + new N5.
  ASSERT_EQ(snap2.nodes.size(), 4u);
  EXPECT_EQ(snap2.nodes[0].node_id, "N2");
  EXPECT_EQ(snap2.nodes[1].node_id, "N3");
  EXPECT_TRUE(snap2.nodes[1].released);
  EXPECT_EQ(snap2.nodes[2].node_id, "N4");
  EXPECT_FALSE(snap2.nodes[2].released);
  EXPECT_EQ(snap2.nodes[3].node_id, "N5");
  EXPECT_FALSE(snap2.nodes[3].released);
}

TEST(RecordPublished, NewOrderIdAdoptsAsIs)
{
  OrderLifecycleManager mgr(kAGV);
  mgr.record_published(make_base_order());
  ASSERT_TRUE(mgr.has_active_order());

  // Different order_id — new order entirely. record_published must
  // adopt as-is (no merging across order_id boundary).
  vda5050_core::types::Order other;
  other.order_id = "ORDER_B";
  other.order_update_id = 0;
  other.nodes = {make_node("M0", 0, true), make_node("M1", 2, true)};
  other.edges = {make_edge("EM0", 1, "M0", "M1", true)};

  mgr.record_published(other);

  auto snap = mgr.snapshot();
  EXPECT_EQ(snap.order_id, std::string("ORDER_B"));
  ASSERT_EQ(snap.nodes.size(), 2u);
  EXPECT_EQ(snap.nodes[0].node_id, "M0");
  EXPECT_EQ(snap.nodes[1].node_id, "M1");
  ASSERT_EQ(snap.edges.size(), 1u);
  EXPECT_EQ(snap.edges[0].edge_id, "EM0");
}

}  // namespace test
}  // namespace vda5050_core::master
