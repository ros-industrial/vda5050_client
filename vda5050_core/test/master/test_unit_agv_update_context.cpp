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

#include "vda5050_core/master/contexts/agv_update_context.hpp"

namespace vda5050_core::master::test {

namespace {

types::State state_with_last_node(const std::string& id, uint32_t seq)
{
  types::State s;
  s.last_node_id = id;
  s.last_node_sequence_id = seq;
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
TEST(AGVUpdateContextTest, NodeReachedOnLastNodeAdvance)
{
  AGVUpdateContext context("agv1");

  std::vector<NodeReachedUpdate> reached;
  context.provider()->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  context.on_state(state_with_last_node("n0", 0));
  EXPECT_TRUE(reached.empty());

  context.on_state(state_with_last_node("n1", 2));
  ASSERT_EQ(reached.size(), 1u);
  EXPECT_EQ(reached[0].agv_id, "agv1");
  EXPECT_EQ(reached[0].node.node_id, "n1");
  EXPECT_EQ(reached[0].node.sequence_id, 2u);

  // Re-sending the same State pushes nothing further.
  context.on_state(state_with_last_node("n1", 2));
  EXPECT_EQ(reached.size(), 1u);
}

// The first Connection message is itself a transition (CONNECTED for ONLINE);
// a sustained state is not.
TEST(AGVUpdateContextTest, ConnectionTransitionReport)
{
  AGVUpdateContext context("agv1");

  std::vector<ConnectionChangedUpdate> updates;
  context.provider()->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate> u) { updates.push_back(*u); });

  context.on_connection(connection(types::ConnectionState::ONLINE));
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].agv_id, "agv1");
  EXPECT_EQ(updates[0].kind, ConnectionTransition::CONNECTED);

  context.on_connection(connection(types::ConnectionState::ONLINE));
  EXPECT_EQ(updates.size(), 1u);
}

// Flipping every state flag in one update pushes one value-carrying update per
// concern, and a new error pushes an ErrorsChangedUpdate.
TEST(AGVUpdateContextTest, FlagChangesCarryValuesAndErrors)
{
  AGVUpdateContext context("agv1");

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
  context.on_state(baseline);

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

  context.on_state(s);

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
TEST(AGVUpdateContextTest, ErrorResolvedIsReported)
{
  AGVUpdateContext context("agv1");

  std::vector<ErrorsChangedUpdate> updates;
  context.provider()->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { updates.push_back(*u); });

  types::State with_error;
  types::Error err;
  err.error_type = "someError";
  with_error.errors.push_back(err);

  context.on_state(with_error);  // baseline carries the error
  EXPECT_TRUE(updates.empty());

  context.on_state(types::State{});  // error cleared
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].agv_id, "agv1");
  EXPECT_TRUE(updates[0].appeared.empty());
  ASSERT_EQ(updates[0].resolved.size(), 1u);
  EXPECT_EQ(updates[0].resolved[0].error_type, "someError");
}

// Each connection-state change is reported with its mapped kind.
TEST(AGVUpdateContextTest, OfflineAndBrokenTransitions)
{
  AGVUpdateContext context("agv1");

  std::vector<ConnectionChangedUpdate> updates;
  context.provider()->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate> u) { updates.push_back(*u); });

  context.on_connection(connection(types::ConnectionState::ONLINE));
  context.on_connection(connection(types::ConnectionState::OFFLINE));
  context.on_connection(connection(types::ConnectionState::CONNECTIONBROKEN));

  ASSERT_EQ(updates.size(), 3u);
  EXPECT_EQ(updates[0].kind, ConnectionTransition::CONNECTED);
  EXPECT_EQ(updates[1].kind, ConnectionTransition::OFFLINE);
  EXPECT_EQ(updates[2].kind, ConnectionTransition::CONNECTIONBROKEN);
  for (const auto& u : updates) EXPECT_EQ(u.agv_id, "agv1");
}

// The context caches the latest update of each type for get_update<T>().
TEST(AGVUpdateContextTest, CachesLatestUpdateForGetUpdate)
{
  AGVUpdateContext context("agv1");

  // Nothing produced yet.
  EXPECT_EQ(context.get_update<OperatingModeChangedUpdate>(), nullptr);

  types::State baseline;
  baseline.operating_mode = types::OperatingMode::AUTOMATIC;
  context.on_state(baseline);

  types::State s;
  s.operating_mode = types::OperatingMode::SEMIAUTOMATIC;
  context.on_state(s);

  auto cached = context.get_update<OperatingModeChangedUpdate>();
  ASSERT_NE(cached, nullptr);
  EXPECT_EQ(cached->mode, types::OperatingMode::SEMIAUTOMATIC);
  EXPECT_EQ(cached->prev_mode, types::OperatingMode::AUTOMATIC);
}

// One thread drives on_state (the single writer) while others read the cache
// via get_update. Counts are timing-dependent, so we only assert progress; the
// value is running this under TSan to prove storage_mutex_ serialises the
// cache against concurrent readers.
TEST(AGVUpdateContextTest, ConcurrentCacheAccessIsThreadSafe)
{
  AGVUpdateContext context("agv1");

  std::atomic<bool> stop{false};
  constexpr int kIterations = 1000;
  std::thread writer([&] {
    for (int i = 0; i < kIterations; ++i)
    {
      context.on_state(state_with_last_node("n" + std::to_string(i), i));
    }
    stop.store(true);
  });

  std::atomic<int> reads{0};
  auto reader = [&] {
    while (!stop.load())
    {
      if (context.get_update<NodeReachedUpdate>()) reads.fetch_add(1);
    }
  };
  std::thread r1(reader);
  std::thread r2(reader);

  writer.join();
  r1.join();
  r2.join();

  EXPECT_GT(reads.load(), 0);
}

}  // namespace vda5050_core::master::test
