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

class AdapterLocalizationTest : public AdapterTest
{};

TEST_F(AdapterLocalizationTest, InitPositionUsesLocalizationCallback)
{
  std::atomic_bool called = false;

  adapter->on_localize(
    [&](
      LocalizationRequest request, std::shared_ptr<ActionExecution> execution) {
      EXPECT_DOUBLE_EQ(request.x(), 1.0);
      EXPECT_DOUBLE_EQ(request.y(), 2.0);
      EXPECT_DOUBLE_EQ(request.theta(), 0.5);
      EXPECT_EQ(request.map_id(), "map");

      execution->finished();

      called = true;
    });

  adapter->start();

  vda5050_core::types::Action action = make_action("action_1", "initPosition");
  std::vector<ActionParameter> params;
  params.push_back(ActionParameter{"x", "1.0"});
  params.push_back(ActionParameter{"y", "2.0"});
  params.push_back(ActionParameter{"theta", "0.5"});
  params.push_back(ActionParameter{"mapId", "map"});
  params.push_back(ActionParameter{"lastNodeId", "N0"});
  action.action_parameters = params;

  InstantActions actions;
  actions.actions.push_back(action);

  inject_message(
    fmt::format("{}/instantActions", protocol_adapter->get_topic_prefix()),
    nlohmann::json(actions).dump());

  ASSERT_TRUE(wait_until([&] { return called.load(); }));

  adapter->stop();
}

TEST_F(AdapterLocalizationTest, InitPositionWithoutCallbackFails)
{
  auto manager = adapter->state_manager();

  manager->set_position(10.0, 20.0, 0.5, "agv_map");

  adapter->start();

  vda5050_core::types::Action action = make_action("action_1", "initPosition");
  std::vector<ActionParameter> params;
  params.push_back(ActionParameter{"x", "1.0"});
  params.push_back(ActionParameter{"y", "2.0"});
  params.push_back(ActionParameter{"theta", "0.5"});
  params.push_back(ActionParameter{"mapId", "world_map"});
  params.push_back(ActionParameter{"lastNodeId", "N0"});
  action.action_parameters = params;

  InstantActions actions;
  actions.actions.push_back(action);

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

  ASSERT_TRUE(state.agv_position.has_value());
  EXPECT_FALSE(state.agv_position->position_initialized);

  adapter->stop();
}

TEST_F(AdapterLocalizationTest, MissingParametersFail)
{
  auto manager = adapter->state_manager();

  adapter->start();

  auto action = make_action("action_1", "initPosition");

  action.action_parameters = {
    ActionParameter{"x", "1.0"}, ActionParameter{"y", "2.0"}};

  InstantActions actions;
  actions.actions.push_back(action);

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

  adapter->stop();
}

TEST_F(AdapterLocalizationTest, LocalizationUpdatesLastNode)
{
  auto manager = adapter->state_manager();

  adapter->on_localize(
    [&](
      LocalizationRequest request, std::shared_ptr<ActionExecution> execution) {
      manager->initialize_position(
        request.x(), request.y(), request.theta(), request.map_id());
      execution->finished();
    });

  adapter->start();

  auto action = make_action("action_1", "initPosition");

  action.action_parameters = {
    ActionParameter{"x", "1.0"}, ActionParameter{"y", "2.0"},
    ActionParameter{"theta", "0.5"}, ActionParameter{"mapId", "world_map"},
    ActionParameter{"lastNodeId", "N5"}};

  InstantActions actions;
  actions.actions.push_back(action);

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

  ASSERT_TRUE(state.agv_position.has_value());
  EXPECT_TRUE(state.agv_position->position_initialized);

  EXPECT_EQ(state.last_node_id, "N5");

  adapter->stop();
}
