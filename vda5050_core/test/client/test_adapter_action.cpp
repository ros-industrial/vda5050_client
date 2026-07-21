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

#include <gmock/gmock.h>

#include "adapter_test_fixture.hpp"

class AdapterActionTest : public AdapterTest
{};

TEST_F(AdapterActionTest, InstantActionDispatches)
{
  std::atomic_bool called = false;

  adapter->on_action(
    [&](ActionRequest request, std::shared_ptr<ActionExecution> /*execution*/) {
      called = true;

      EXPECT_EQ(request.action_id(), "action_1");
      EXPECT_EQ(request.action_type(), "customAction");
    });

  adapter->start();

  InstantActions actions;
  actions.actions.push_back(make_action("action_1", "customAction"));

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  ASSERT_TRUE(wait_until([&] { return called.load(); }));

  adapter->stop();
}

TEST_F(AdapterActionTest, MultipleActionsProcessedInOrder)
{
  std::mutex mutex;
  std::vector<std::string> ids;

  adapter->on_action(
    [&](ActionRequest request, std::shared_ptr<ActionExecution> execution) {
      std::lock_guard<std::mutex> lock(mutex);
      ids.push_back(request.action_id());
      execution->finished();
    });

  adapter->start();

  InstantActions actions;

  actions.actions.push_back(make_action("action_1", "customAction"));
  actions.actions.push_back(make_action("action_2", "customAction"));
  actions.actions.push_back(make_action("action_3", "customAction"));

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return ids.size() == 3;
  }));

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_THAT(ids, testing::ElementsAre("action_1", "action_2", "action_3"));

  adapter->stop();
}

TEST_F(AdapterActionTest, StartsRunning)
{
  adapter->on_action([&](
                       ActionRequest /*request*/,
                       std::shared_ptr<ActionExecution> /*execution*/) {});

  adapter->start();

  InstantActions actions;
  actions.actions.push_back(make_action("action_1", "customAction"));

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  ASSERT_TRUE(wait_publish(2));

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  ASSERT_EQ(state.action_states.size(), 1);

  EXPECT_EQ(state.action_states.front().action_status, ActionStatus::RUNNING);

  adapter->stop();
}

TEST_F(AdapterActionTest, FinishedUpdatesState)
{
  adapter->on_action(
    [&](ActionRequest /*request*/, std::shared_ptr<ActionExecution> execution) {
      execution->finished();
    });

  adapter->start();

  InstantActions actions;
  actions.actions.push_back(make_action("action_1", "customAction"));

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(publish_mutex);
    if (published.empty()) return false;
    try
    {
      auto state = nlohmann::json::parse(published.back().message).get<State>();
      return !state.action_states.empty() &&
             state.action_states.front().action_status ==
               ActionStatus::FINISHED;
    }
    catch (...)
    {
      return false;
    }
  }));
  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  ASSERT_EQ(state.action_states.size(), 1);

  EXPECT_EQ(state.action_states.front().action_status, ActionStatus::FINISHED);

  adapter->stop();
}

TEST_F(AdapterActionTest, FailedUpdatesState)
{
  adapter->on_action(
    [&](ActionRequest /*request*/, std::shared_ptr<ActionExecution> execution) {
      execution->failed("failed");
    });

  adapter->start();

  InstantActions actions;
  actions.actions.push_back(make_action("action_1", "customAction"));

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(publish_mutex);
    if (published.empty()) return false;
    try
    {
      auto state = nlohmann::json::parse(published.back().message).get<State>();
      return !state.action_states.empty() &&
             state.action_states.front().action_status == ActionStatus::FAILED;
    }
    catch (...)
    {
      return false;
    }
  }));

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  ASSERT_EQ(state.action_states.size(), 1);

  EXPECT_EQ(state.action_states.front().action_status, ActionStatus::FAILED);
  EXPECT_EQ(state.action_states.front().result_description, "failed");

  adapter->stop();
}

TEST_F(AdapterActionTest, CallbackExceptionHandled)
{
  adapter->on_action([&](ActionRequest, std::shared_ptr<ActionExecution>) {
    throw std::runtime_error("boom");
  });

  adapter->start();

  InstantActions actions;
  actions.actions.push_back(make_action("action_1", "customAction"));

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  SUCCEED();

  adapter->stop();
}
