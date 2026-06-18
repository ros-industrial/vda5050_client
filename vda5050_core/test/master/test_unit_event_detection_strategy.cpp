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

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "vda5050_core/execution/handler.hpp"
#include "vda5050_core/master/event_detection_strategy.hpp"

namespace vda5050_core::master::test {

namespace {
// Minimal context that caches inbound updates by type, mirroring the
// SimpleContext shape from the execution usage docs.
class TestContext : public execution::ContextInterface
{
public:
  void init() override
  {
    provider()->on<StateUpdate>([this](std::shared_ptr<StateUpdate> update) {
      std::lock_guard<std::mutex> lock(mutex_);
      updates_[update->get_type()] = update;
    });
    provider()->on<ConnectionUpdate>(
      [this](std::shared_ptr<ConnectionUpdate> update) {
        std::lock_guard<std::mutex> lock(mutex_);
        updates_[update->get_type()] = update;
      });
  }

protected:
  std::shared_ptr<execution::UpdateBase> get_update_raw(
    std::type_index type) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = updates_.find(type);
    if (it != updates_.end()) return it->second;
    return nullptr;
  }

  std::shared_ptr<execution::ResourceBase> get_resource_raw(
    std::type_index) const override
  {
    return nullptr;
  }

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::type_index, std::shared_ptr<execution::UpdateBase>>
    updates_;
};

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

// A node advance on a later State emits exactly one NodeReachedEvent to a
// consumer registered on the strategy's engine.
TEST(EventDetectionStrategyTest, EmitsNodeReachedOnLastNodeAdvance)
{
  auto context = std::make_shared<TestContext>();
  auto strategy = std::make_shared<EventDetectionStrategy>();
  auto handler = execution::Handler::make(context, {strategy});

  std::vector<event::ReachedNode> reached;
  strategy->engine()->on<NodeReachedEvent>(
    [&](std::shared_ptr<NodeReachedEvent> e) { reached.push_back(e->node); });

  // First State is only a baseline; no transition yet.
  context->provider()->push<StateUpdate>(state_with_last_node("n0", 0));
  handler->spin_once();
  EXPECT_TRUE(reached.empty());

  // Last node advances -> one NodeReachedEvent.
  context->provider()->push<StateUpdate>(state_with_last_node("n1", 2));
  handler->spin_once();

  ASSERT_EQ(reached.size(), 1u);
  EXPECT_EQ(reached[0].node_id, "n1");
  EXPECT_EQ(reached[0].sequence_id, 2u);

  // Re-spinning the same cached State emits nothing further.
  handler->spin_once();
  EXPECT_EQ(reached.size(), 1u);
}

// The first Connection message is itself a transition (CONNECTED for ONLINE).
TEST(EventDetectionStrategyTest, EmitsConnectionTransition)
{
  auto context = std::make_shared<TestContext>();
  auto strategy = std::make_shared<EventDetectionStrategy>();
  auto handler = execution::Handler::make(context, {strategy});

  std::vector<ConnectionEventKind> kinds;
  strategy->engine()->on<ConnectionChangedEvent>(
    [&](std::shared_ptr<ConnectionChangedEvent> e) {
      kinds.push_back(e->kind);
    });

  context->provider()->push<ConnectionUpdate>(
    connection(types::ConnectionState::ONLINE));
  handler->spin_once();

  ASSERT_EQ(kinds.size(), 1u);
  EXPECT_EQ(kinds[0], ConnectionEventKind::CONNECTED);

  // Sustained ONLINE is not a transition.
  context->provider()->push<ConnectionUpdate>(
    connection(types::ConnectionState::ONLINE));
  handler->spin_once();
  EXPECT_EQ(kinds.size(), 1u);
}

// Every state flag transition (new-base, mode, paused, driving, loads) emits a
// StateFlagChangedEvent, and a new error emits an ErrorsChangedEvent.
TEST(EventDetectionStrategyTest, EmitsFlagChangesAndErrors)
{
  auto context = std::make_shared<TestContext>();
  auto strategy = std::make_shared<EventDetectionStrategy>();
  auto handler = execution::Handler::make(context, {strategy});

  std::vector<StateFlagChangedEvent::Flag> flags;
  strategy->engine()->on<StateFlagChangedEvent>(
    [&](std::shared_ptr<StateFlagChangedEvent> e) {
      flags.push_back(e->flag);
    });
  std::vector<types::Error> appeared;
  strategy->engine()->on<ErrorsChangedEvent>(
    [&](std::shared_ptr<ErrorsChangedEvent> e) { appeared = e->appeared; });

  // Baseline: defaults, no transitions.
  context->provider()->push<StateUpdate>(types::State{});
  handler->spin_once();
  EXPECT_TRUE(flags.empty());

  // Flip every flag and add an error in one update.
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

  context->provider()->push<StateUpdate>(s);
  handler->spin_once();

  using Flag = StateFlagChangedEvent::Flag;
  const std::set<Flag> got(flags.begin(), flags.end());
  EXPECT_EQ(got.count(Flag::NEW_BASE_REQUESTED), 1u);
  EXPECT_EQ(got.count(Flag::MODE), 1u);
  EXPECT_EQ(got.count(Flag::PAUSED), 1u);
  EXPECT_EQ(got.count(Flag::DRIVING), 1u);
  EXPECT_EQ(got.count(Flag::LOADS), 1u);

  ASSERT_EQ(appeared.size(), 1u);
  EXPECT_EQ(appeared[0].error_type, "someError");
}

}  // namespace vda5050_core::master::test
