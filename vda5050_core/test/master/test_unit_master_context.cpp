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

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "vda5050_core/master/contexts/master_context.hpp"

namespace vda5050_core::master::test {

namespace {

types::State state_with_last_node(
  const std::string& id, uint32_t seq, uint32_t header_id = 0)
{
  types::State s;
  s.last_node_id = id;
  s.last_node_sequence_id = seq;
  s.header.header_id = header_id;
  return s;
}

types::Connection connection(types::ConnectionState s)
{
  types::Connection c;
  c.connection_state = s;
  return c;
}

}  // namespace

// A node advance on a later State pushes exactly one NodeReachedUpdate, tagged
// with the AGV id. The first State only seeds the baseline.
TEST(MasterContextTest, NodeReachedOnLastNodeAdvance)
{
  MasterContext context;

  std::vector<NodeReachedUpdate> reached;
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  context.on_state("agv1", state_with_last_node("n0", 0));
  EXPECT_TRUE(reached.empty());

  context.on_state("agv1", state_with_last_node("n1", 2));
  ASSERT_EQ(reached.size(), 1u);
  EXPECT_EQ(reached[0].agv_id, "agv1");
  EXPECT_EQ(reached[0].node_id, "n1");
  EXPECT_EQ(reached[0].sequence_id, 2u);

  // Re-sending the same State pushes nothing further.
  context.on_state("agv1", state_with_last_node("n1", 2));
  EXPECT_EQ(reached.size(), 1u);
}

// One context serves every AGV with an independent baseline: agv1 advancing
// does not affect agv2's diff.
TEST(MasterContextTest, TracksBaselinePerAgv)
{
  MasterContext context;

  std::vector<NodeReachedUpdate> reached;
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  context.on_state("agv1", state_with_last_node("n0", 0));
  context.on_state("agv2", state_with_last_node("n0", 0));
  EXPECT_TRUE(reached.empty());

  // Only agv1 advances.
  context.on_state("agv1", state_with_last_node("n1", 2));
  context.on_state("agv2", state_with_last_node("n0", 0));
  ASSERT_EQ(reached.size(), 1u);
  EXPECT_EQ(reached[0].agv_id, "agv1");
  EXPECT_EQ(reached[0].node_id, "n1");
}

// The first Connection message is itself a transition (CONNECTED for ONLINE);
// a sustained state is not.
TEST(MasterContextTest, ConnectionTransitionReport)
{
  MasterContext context;

  std::vector<ConnectionChangedUpdate> updates;
  context.provider()->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate> u) { updates.push_back(*u); });

  context.on_connection("agv1", connection(types::ConnectionState::ONLINE));
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].agv_id, "agv1");
  EXPECT_EQ(updates[0].kind, ConnectionTransition::CONNECTED);

  context.on_connection("agv1", connection(types::ConnectionState::ONLINE));
  EXPECT_EQ(updates.size(), 1u);
}

// Flipping every state flag in one update pushes one value-carrying update per
// concern, and a new error pushes an ErrorsChangedUpdate.
TEST(MasterContextTest, FlagChangesCarryValuesAndErrors)
{
  MasterContext context;

  std::optional<OperatingModeChangedUpdate> mode;
  std::optional<PausedChangedUpdate> paused;
  std::optional<DrivingChangedUpdate> driving;
  std::optional<NewBaseRequestUpdate> new_base;
  std::optional<LoadsChangedUpdate> loads;
  std::optional<ErrorsChangedUpdate> errors;

  context.provider()->on<OperatingModeChangedUpdate>(
    [&](std::shared_ptr<OperatingModeChangedUpdate> u) { mode = *u; });
  context.provider()->on<PausedChangedUpdate>(
    [&](std::shared_ptr<PausedChangedUpdate> u) { paused = *u; });
  context.provider()->on<DrivingChangedUpdate>(
    [&](std::shared_ptr<DrivingChangedUpdate> u) { driving = *u; });
  context.provider()->on<NewBaseRequestUpdate>(
    [&](std::shared_ptr<NewBaseRequestUpdate> u) { new_base = *u; });
  context.provider()->on<LoadsChangedUpdate>(
    [&](std::shared_ptr<LoadsChangedUpdate> u) { loads = *u; });
  context.provider()->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { errors = *u; });

  types::State baseline;  // all defaults
  baseline.operating_mode = types::OperatingMode::AUTOMATIC;
  context.on_state("agv1", baseline);

  types::State s;
  s.new_base_request = true;
  s.operating_mode = types::OperatingMode::SEMIAUTOMATIC;
  s.paused = true;
  s.driving = true;
  types::Load load;
  load.load_id = "L1";
  s.loads = std::vector<types::Load>{load};
  types::Error err;
  err.error_type = "someError";
  s.errors.push_back(err);

  context.on_state("agv1", s);

  ASSERT_TRUE(mode.has_value());
  EXPECT_EQ(mode->agv_id, "agv1");
  EXPECT_EQ(mode->mode, types::OperatingMode::SEMIAUTOMATIC);
  EXPECT_EQ(mode->prev_mode, types::OperatingMode::AUTOMATIC);
  ASSERT_TRUE(paused.has_value());
  EXPECT_EQ(paused->agv_id, "agv1");
  EXPECT_TRUE(paused->paused);
  ASSERT_TRUE(driving.has_value());
  EXPECT_EQ(driving->agv_id, "agv1");
  EXPECT_TRUE(driving->driving);
  ASSERT_TRUE(new_base.has_value());
  EXPECT_EQ(new_base->agv_id, "agv1");
  ASSERT_TRUE(loads.has_value());
  EXPECT_EQ(loads->agv_id, "agv1");
  ASSERT_EQ(loads->loads.size(), 1u);
  EXPECT_EQ(loads->loads[0].load_id, "L1");
  ASSERT_TRUE(errors.has_value());
  EXPECT_EQ(errors->agv_id, "agv1");
  ASSERT_EQ(errors->appeared.size(), 1u);
  EXPECT_EQ(errors->appeared[0].error_type, "someError");
}

// An error present in the prev State but gone from the current one is reported
// as resolved.
TEST(MasterContextTest, ErrorResolvedIsReported)
{
  MasterContext context;

  std::vector<ErrorsChangedUpdate> updates;
  context.provider()->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { updates.push_back(*u); });

  types::State with_error;
  types::Error err;
  err.error_type = "someError";
  with_error.errors.push_back(err);

  context.on_state("agv1", with_error);  // baseline carries the error
  EXPECT_TRUE(updates.empty());

  context.on_state("agv1", types::State{});  // error cleared
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].agv_id, "agv1");
  EXPECT_TRUE(updates[0].appeared.empty());
  ASSERT_EQ(updates[0].resolved.size(), 1u);
  EXPECT_EQ(updates[0].resolved[0].error_type, "someError");
}

// Each connection-state change is reported with its mapped kind.
TEST(MasterContextTest, OfflineAndBrokenTransitions)
{
  MasterContext context;

  std::vector<ConnectionChangedUpdate> updates;
  context.provider()->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate> u) { updates.push_back(*u); });

  context.on_connection("agv1", connection(types::ConnectionState::ONLINE));
  context.on_connection("agv1", connection(types::ConnectionState::OFFLINE));
  context.on_connection(
    "agv1", connection(types::ConnectionState::CONNECTIONBROKEN));

  ASSERT_EQ(updates.size(), 3u);
  EXPECT_EQ(updates[0].kind, ConnectionTransition::CONNECTED);
  EXPECT_EQ(updates[1].kind, ConnectionTransition::OFFLINE);
  EXPECT_EQ(updates[2].kind, ConnectionTransition::CONNECTIONBROKEN);
  for (const auto& u : updates) EXPECT_EQ(u.agv_id, "agv1");
}

// new_base_request is edge-triggered: fires only on false -> true, not on a
// sustained true or a falling edge.
TEST(MasterContextTest, NewBaseRequestRisingEdgeOnly)
{
  MasterContext context;

  int count = 0;
  context.provider()->on<NewBaseRequestUpdate>(
    [&](std::shared_ptr<NewBaseRequestUpdate>) { ++count; });

  types::State off;  // new_base_request unset (false)
  types::State on;
  on.new_base_request = true;

  context.on_state("agv1", off);  // seed
  context.on_state("agv1", on);   // false -> true: fires
  EXPECT_EQ(count, 1);
  context.on_state("agv1", on);  // sustained true: no fire
  EXPECT_EQ(count, 1);
  context.on_state("agv1", off);  // true -> false: no fire
  EXPECT_EQ(count, 1);
}

// A single State that clears an old error and raises a new one yields one
// ErrorsChangedUpdate carrying both lists.
TEST(MasterContextTest, ErrorsAppearedAndResolvedInOneUpdate)
{
  MasterContext context;

  std::optional<ErrorsChangedUpdate> got;
  context.provider()->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { got = *u; });

  types::State first;
  types::Error old_err;
  old_err.error_type = "old";
  first.errors.push_back(old_err);
  context.on_state("agv1", first);  // seed with "old"

  types::State second;
  types::Error new_err;
  new_err.error_type = "new";
  second.errors.push_back(new_err);
  context.on_state("agv1", second);  // "old" resolved, "new" appeared

  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->appeared.size(), 1u);
  EXPECT_EQ(got->appeared[0].error_type, "new");
  ASSERT_EQ(got->resolved.size(), 1u);
  EXPECT_EQ(got->resolved[0].error_type, "old");
}

// Error identity is type + references, not description: a reworded description
// on the same error is not reported as resolved-then-reappeared.
TEST(MasterContextTest, ErrorDescriptionChangeIsNotAnErrorChange)
{
  MasterContext context;

  int count = 0;
  context.provider()->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate>) { ++count; });

  types::State first;
  types::Error e1;
  e1.error_type = "someError";
  e1.error_description = "initial wording";
  first.errors.push_back(e1);
  context.on_state("agv1", first);  // seed

  types::State second;
  types::Error e2;
  e2.error_type = "someError";
  e2.error_description = "reworded, same error";
  second.errors.push_back(e2);
  context.on_state("agv1", second);  // only the description changed

  EXPECT_EQ(count, 0);
}

// Error identity includes references: same type but different references is a
// different error -> the old one resolves and the new one appears.
TEST(MasterContextTest, ErrorReferenceChangeIsANewError)
{
  MasterContext context;

  std::optional<ErrorsChangedUpdate> got;
  context.provider()->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { got = *u; });

  auto error_at = [](const std::string& ref_value) {
    types::Error e;
    e.error_type = "someError";
    types::ErrorReference r;
    r.reference_key = "node";
    r.reference_value = ref_value;
    e.error_references = std::vector<types::ErrorReference>{r};
    return e;
  };

  types::State first;
  first.errors.push_back(error_at("nodeA"));
  context.on_state("agv1", first);  // seed

  types::State second;
  second.errors.push_back(error_at("nodeB"));  // same type, different reference
  context.on_state("agv1", second);

  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->appeared.size(), 1u);
  EXPECT_EQ(got->appeared[0].error_references->at(0).reference_value, "nodeB");
  ASSERT_EQ(got->resolved.size(), 1u);
  EXPECT_EQ(got->resolved[0].error_references->at(0).reference_value, "nodeA");
}

// The staleness guard uses strict `<`: a State whose header_id equals the
// baseline still diffs (only strictly-older is dropped).
TEST(MasterContextTest, EqualHeaderIdPassesGuard)
{
  MasterContext context;

  std::vector<NodeReachedUpdate> reached;
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  context.on_state("agv1", state_with_last_node("n0", 0, 7));  // seed, header 7
  context.on_state(
    "agv1", state_with_last_node("n1", 2, 7));  // equal id: fires
  ASSERT_EQ(reached.size(), 1u);
  context.on_state(
    "agv1", state_with_last_node("n1", 2, 7));  // equal id, no chg
  EXPECT_EQ(reached.size(), 1u);
}

// An out-of-order State (older header_id than the baseline) is dropped: it
// neither fires an update nor overwrites the baseline.
TEST(MasterContextTest, OutOfOrderStateIsDropped)
{
  MasterContext context;

  std::vector<NodeReachedUpdate> reached;
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  context.on_state("agv1", state_with_last_node("n0", 0, 1));  // seed
  context.on_state("agv1", state_with_last_node("n2", 4, 3));  // advance
  ASSERT_EQ(reached.size(), 1u);

  // Older header_id carrying an earlier node: dropped, no fire.
  context.on_state("agv1", state_with_last_node("n1", 2, 2));
  EXPECT_EQ(reached.size(), 1u);

  // Baseline still n2, so a genuinely newer State diffs against n2, not n1.
  context.on_state("agv1", state_with_last_node("n3", 6, 4));
  ASSERT_EQ(reached.size(), 2u);
  EXPECT_EQ(reached[1].node_id, "n3");
}

// After a reconnect (CONNECTED edge) the stale baseline is dropped, so the
// AGV's restarted low header_ids are accepted and re-seed instead of blocked.
TEST(MasterContextTest, ReconnectReseedsStateBaseline)
{
  MasterContext context;

  std::vector<NodeReachedUpdate> reached;
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  context.on_connection("agv1", connection(types::ConnectionState::ONLINE));
  context.on_state("agv1", state_with_last_node("n0", 0, 5000));  // session 1
  context.on_connection("agv1", connection(types::ConnectionState::OFFLINE));
  context.on_connection("agv1", connection(types::ConnectionState::ONLINE));

  // Session 2: fresh low header_id re-seeds (no fire), then advances normally.
  context.on_state("agv1", state_with_last_node("n0", 0, 1));
  EXPECT_TRUE(reached.empty());
  context.on_state("agv1", state_with_last_node("n1", 2, 2));
  ASSERT_EQ(reached.size(), 1u);
  EXPECT_EQ(reached[0].node_id, "n1");
}

// Producer-only: no per-type cache (per-AGV latest lives on the AGV).
TEST(MasterContextTest, GetUpdateIsProducerOnly)
{
  MasterContext context;
  context.on_state("agv1", state_with_last_node("n0", 0));
  context.on_state("agv1", state_with_last_node("n1", 2));
  EXPECT_EQ(context.get_update<NodeReachedUpdate>(), nullptr);
}

// Two AGVs fed concurrently from separate threads exercise the shared maps
// under mutex_ (run under TSan); independent streams keep the count exact.
TEST(MasterContextTest, ConcurrentPerAgvFeedIsThreadSafe)
{
  MasterContext context;

  std::atomic<int> count{0};
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate>) { count.fetch_add(1); });

  constexpr int kIterations = 1000;
  auto feed = [&](const std::string& id) {
    for (int i = 0; i < kIterations; ++i)
    {
      context.on_state(id, state_with_last_node("n" + std::to_string(i), i));
    }
  };
  std::thread t1([&] { feed("agv1"); });
  std::thread t2([&] { feed("agv2"); });
  t1.join();
  t2.join();

  // Each AGV: the first message seeds; every later one advances the node once.
  EXPECT_EQ(count.load(), 2 * (kIterations - 1));
}

}  // namespace vda5050_core::master::test
