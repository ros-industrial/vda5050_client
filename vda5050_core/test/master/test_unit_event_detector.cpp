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

#include "vda5050_core/execution/provider.hpp"
#include "vda5050_core/master/event_detector.hpp"

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
TEST(EventDetectorTest, NodeReachedOnLastNodeAdvance)
{
  auto provider = std::make_shared<execution::Provider>();
  EventDetector detector("agv1", provider);

  std::vector<NodeReachedUpdate> reached;
  provider->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate> u) { reached.push_back(*u); });

  detector.on_state(state_with_last_node("n0", 0));
  EXPECT_TRUE(reached.empty());

  detector.on_state(state_with_last_node("n1", 2));
  ASSERT_EQ(reached.size(), 1u);
  EXPECT_EQ(reached[0].agv_id, "agv1");
  EXPECT_EQ(reached[0].node.node_id, "n1");
  EXPECT_EQ(reached[0].node.sequence_id, 2u);

  // Re-sending the same State pushes nothing further.
  detector.on_state(state_with_last_node("n1", 2));
  EXPECT_EQ(reached.size(), 1u);
}

// The first Connection message is itself a transition (CONNECTED for ONLINE);
// a sustained state is not.
TEST(EventDetectorTest, ConnectionTransition)
{
  auto provider = std::make_shared<execution::Provider>();
  EventDetector detector("agv1", provider);

  std::vector<ConnectionChangedUpdate> updates;
  provider->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate> u) { updates.push_back(*u); });

  detector.on_connection(connection(types::ConnectionState::ONLINE));
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].agv_id, "agv1");
  EXPECT_EQ(updates[0].kind, ConnectionEventKind::CONNECTED);

  detector.on_connection(connection(types::ConnectionState::ONLINE));
  EXPECT_EQ(updates.size(), 1u);
}

// Flipping every state flag in one update pushes one value-carrying update per
// concern, and a new error pushes an ErrorsChangedUpdate.
TEST(EventDetectorTest, FlagChangesCarryValuesAndErrors)
{
  auto provider = std::make_shared<execution::Provider>();
  EventDetector detector("agv1", provider);

  std::optional<OperatingModeChangedUpdate> mode;
  std::optional<PausedChangedUpdate> paused;
  std::optional<DrivingChangedUpdate> driving;
  std::optional<NewBaseRequestUpdate> new_base;
  std::optional<LoadsChangedUpdate> loads;
  std::optional<ErrorsChangedUpdate> errors;

  provider->on<OperatingModeChangedUpdate>(
    [&](std::shared_ptr<OperatingModeChangedUpdate> u) { mode = *u; });
  provider->on<PausedChangedUpdate>(
    [&](std::shared_ptr<PausedChangedUpdate> u) { paused = *u; });
  provider->on<DrivingChangedUpdate>(
    [&](std::shared_ptr<DrivingChangedUpdate> u) { driving = *u; });
  provider->on<NewBaseRequestUpdate>(
    [&](std::shared_ptr<NewBaseRequestUpdate> u) { new_base = *u; });
  provider->on<LoadsChangedUpdate>(
    [&](std::shared_ptr<LoadsChangedUpdate> u) { loads = *u; });
  provider->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { errors = *u; });

  detector.on_state(types::State{});  // baseline: all defaults

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

  detector.on_state(s);

  ASSERT_TRUE(mode.has_value());
  EXPECT_EQ(mode->agv_id, "agv1");
  EXPECT_EQ(mode->mode, types::OperatingMode::SEMIAUTOMATIC);
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
TEST(EventDetectorTest, ErrorResolvedIsReported)
{
  auto provider = std::make_shared<execution::Provider>();
  EventDetector detector("agv1", provider);

  std::vector<ErrorsChangedUpdate> updates;
  provider->on<ErrorsChangedUpdate>(
    [&](std::shared_ptr<ErrorsChangedUpdate> u) { updates.push_back(*u); });

  types::State with_error;
  types::Error err;
  err.error_type = "someError";
  with_error.errors.push_back(err);

  detector.on_state(with_error);  // baseline carries the error
  EXPECT_TRUE(updates.empty());

  detector.on_state(types::State{});  // error cleared
  ASSERT_EQ(updates.size(), 1u);
  EXPECT_EQ(updates[0].agv_id, "agv1");
  EXPECT_TRUE(updates[0].appeared.empty());
  ASSERT_EQ(updates[0].resolved.size(), 1u);
  EXPECT_EQ(updates[0].resolved[0].error_type, "someError");
}

// Each connection-state change is reported with its mapped kind.
TEST(EventDetectorTest, OfflineAndBrokenTransitions)
{
  auto provider = std::make_shared<execution::Provider>();
  EventDetector detector("agv1", provider);

  std::vector<ConnectionChangedUpdate> updates;
  provider->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate> u) { updates.push_back(*u); });

  detector.on_connection(connection(types::ConnectionState::ONLINE));
  detector.on_connection(connection(types::ConnectionState::OFFLINE));
  detector.on_connection(connection(types::ConnectionState::CONNECTIONBROKEN));

  ASSERT_EQ(updates.size(), 3u);
  EXPECT_EQ(updates[0].kind, ConnectionEventKind::CONNECTED);
  EXPECT_EQ(updates[1].kind, ConnectionEventKind::OFFLINE);
  EXPECT_EQ(updates[2].kind, ConnectionEventKind::CONNECTIONBROKEN);
  for (const auto& u : updates) EXPECT_EQ(u.agv_id, "agv1");
}

// Two threads both drive on_state (contending on the same prev_state_
// snapshot) while a third drives on_connection. Counts are interleaving-
// dependent, so we only assert progress; the value is running this under TSan
// to prove the snapshot mutex serialises concurrent access.
TEST(EventDetectorTest, ConcurrentCallsAreThreadSafe)
{
  auto provider = std::make_shared<execution::Provider>();
  EventDetector detector("agv1", provider);

  std::atomic<int> node_updates{0};
  std::atomic<int> conn_updates{0};
  provider->on<NodeReachedUpdate>(
    [&](std::shared_ptr<NodeReachedUpdate>) { node_updates.fetch_add(1); });
  provider->on<ConnectionChangedUpdate>(
    [&](std::shared_ptr<ConnectionChangedUpdate>) {
      conn_updates.fetch_add(1);
    });

  constexpr int kIterations = 1000;
  auto push_states = [&](const std::string& prefix) {
    for (int i = 0; i < kIterations; ++i)
    {
      detector.on_state(state_with_last_node(prefix + std::to_string(i), i));
    }
  };
  std::thread state_a([&] { push_states("a"); });
  std::thread state_b([&] { push_states("b"); });
  std::thread conn_thread([&] {
    for (int i = 0; i < kIterations; ++i)
    {
      detector.on_connection(connection(
        i % 2 == 0 ? types::ConnectionState::ONLINE
                   : types::ConnectionState::OFFLINE));
    }
  });
  state_a.join();
  state_b.join();
  conn_thread.join();

  EXPECT_GT(node_updates.load(), 0);
  EXPECT_GT(conn_updates.load(), 0);
}

}  // namespace vda5050_core::master::test
