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

#include "vda5050_core/client/adapter/state_manager.hpp"

using namespace ::vda5050_core::types;          // NOLINT
using namespace vda5050_core::client::adapter;  // NOLINT

TEST(StateManagerTest, StartsWithDefaultState)
{
  auto manager = StateManager::make();

  auto state = manager->state();

  EXPECT_FALSE(state.agv_position.has_value());
  EXPECT_FALSE(state.velocity.has_value());
  EXPECT_FALSE(state.driving);
  EXPECT_FALSE(state.paused.has_value());

  EXPECT_FALSE(manager->consume_publish_requested());
}

TEST(StateManagerTest, SetPositionWithoutTransformation)
{
  auto manager = StateManager::make();

  manager->set_position(1.0, 2.0, 3.0, "map_1");

  auto state = manager->state();

  ASSERT_TRUE(state.agv_position.has_value());

  EXPECT_DOUBLE_EQ(state.agv_position->x, 1.0);
  EXPECT_DOUBLE_EQ(state.agv_position->y, 2.0);
  EXPECT_DOUBLE_EQ(state.agv_position->theta, 3.0);
  EXPECT_EQ(state.agv_position->map_id, "map_1");
  EXPECT_FALSE(state.agv_position->position_initialized);
}

TEST(StateManagerTest, SetPositionWithTransformation)
{
  auto manager = StateManager::make();

  auto transformation =
    Transformation::calibrate({10.0, 20.0, 0.0}, {0.0, 0.0, 0.0});

  manager->set_transformation(transformation, "world_map");

  manager->set_position(1.0, 2.0, 0.0, "world_map");

  auto state = manager->state();

  ASSERT_TRUE(state.agv_position.has_value());

  EXPECT_DOUBLE_EQ(state.agv_position->x, 11.0);
  EXPECT_DOUBLE_EQ(state.agv_position->y, 22.0);
}

TEST(StateManagerTest, UnknownTransformationReturnsNullopt)
{
  auto manager = StateManager::make();

  auto result = manager->transformation("unknown");

  EXPECT_FALSE(result.has_value());
}

TEST(StateManagerTest, UpdateMotionState)
{
  auto manager = StateManager::make();

  Velocity velocity;
  velocity.vx = 1.0;
  velocity.vy = 2.0;
  velocity.omega = 3.0;

  manager->set_velocity(velocity);
  manager->set_driving(true);
  manager->set_paused(false);
  manager->set_distance_since_last_node(10.0);

  auto state = manager->state();

  ASSERT_TRUE(state.velocity.has_value());

  EXPECT_EQ(state.velocity.value(), velocity);
  EXPECT_TRUE(state.driving);
  EXPECT_FALSE(state.paused.value());
  EXPECT_DOUBLE_EQ(state.distance_since_last_node.value(), 10.0);
}

TEST(StateManagerTest, UpdateVehicleState)
{
  auto manager = StateManager::make();

  BatteryState battery;
  battery.battery_charge = 50.5;
  battery.charging = false;

  SafetyState safety;
  safety.e_stop = EStop::MANUAL;
  safety.field_violation = false;

  manager->set_battery_state(battery);
  manager->set_safety_state(safety);
  manager->set_operating_mode(OperatingMode::AUTOMATIC);

  auto state = manager->state();

  EXPECT_EQ(state.battery_state, battery);
  EXPECT_EQ(state.safety_state, safety);
  EXPECT_EQ(state.operating_mode, OperatingMode::AUTOMATIC);
}

TEST(StateManagerTest, AddActionStateAppends)
{
  auto manager = StateManager::make();

  ActionState action;
  action.action_id = "action_1";

  manager->add_action_state(action);

  auto state = manager->state();

  ASSERT_EQ(state.action_states.size(), 1);
  EXPECT_EQ(state.action_states[0].action_id, "action_1");
}

TEST(StateManagerTest, ReplaceExistingActionState)
{
  auto manager = StateManager::make();

  ActionState first;
  first.action_id = "action_1";
  first.action_status = ActionStatus::WAITING;

  ActionState second;
  second.action_id = "action_1";
  first.action_status = ActionStatus::INITIALIZING;

  manager->add_action_state(first);
  manager->add_action_state(second);

  auto state = manager->state();

  ASSERT_EQ(state.action_states.size(), 1);
  EXPECT_EQ(state.action_states[0], second);
}

TEST(StateManagerTest, SetActionStatesReplacesExisting)
{
  auto manager = StateManager::make();

  ActionState initial;
  initial.action_id = "old";

  manager->add_action_state(initial);

  ActionState replacement;
  replacement.action_id = "new";

  std::vector<ActionState> action_states;
  action_states.push_back(replacement);
  manager->set_action_states(action_states);

  auto state = manager->state();

  ASSERT_EQ(state.action_states.size(), 1);
  EXPECT_EQ(state.action_states[0].action_id, "new");
}

TEST(StateManagerTest, ClearActionStateRemovesAll)
{
  auto manager = StateManager::make();

  std::vector<ActionState> actions(2);

  manager->set_action_states(actions);
  manager->clear_action_states();

  auto state = manager->state();

  EXPECT_TRUE(state.action_states.empty());
}

TEST(StateManagerTest, AddErrorAppends)
{
  auto manager = StateManager::make();

  Error error_1;
  error_1.error_type = "node_error";

  Error error_2;
  error_2.error_type = "edge_error";

  manager->add_error(error_1);
  manager->add_error(error_2);

  auto state = manager->state();

  ASSERT_EQ(state.errors.size(), 2);

  EXPECT_EQ(state.errors[0].error_type, "node_error");
  EXPECT_EQ(state.errors[1].error_type, "edge_error");
}

TEST(StateManagerTest, SetErrorsReplacesExisting)
{
  auto manager = StateManager::make();

  Error initial;
  initial.error_type = "old";

  manager->add_error(initial);

  Error replacement;
  replacement.error_type = "new";

  std::vector<Error> errors;
  errors.push_back(replacement);
  manager->set_errors(errors);

  auto state = manager->state();

  ASSERT_EQ(state.errors.size(), 1);
  EXPECT_EQ(state.errors[0].error_type, "new");
}

TEST(StateManagerTest, ClearErrorRemovesAll)
{
  auto manager = StateManager::make();

  std::vector<Error> errors(2);

  manager->set_errors(errors);
  manager->clear_errors();

  auto state = manager->state();

  EXPECT_TRUE(state.errors.empty());
}

TEST(StateManagerTest, AddLoadCreatesVector)
{
  auto manager = StateManager::make();

  Load load;

  manager->add_load(load);

  auto state = manager->state();

  ASSERT_TRUE(state.loads.has_value());

  EXPECT_EQ(state.loads->size(), 1);
}

TEST(StateManagerTest, AddLoadAppends)
{
  auto manager = StateManager::make();

  Load load_1;
  load_1.load_id = "load_1";

  Load load_2;
  load_2.load_id = "load_2";

  manager->add_load(load_1);
  manager->add_load(load_2);

  auto state = manager->state();

  ASSERT_TRUE(state.loads.has_value());
  ASSERT_EQ(state.loads->size(), 2);

  EXPECT_EQ((*state.loads)[0].load_id, "load_1");
  EXPECT_EQ((*state.loads)[1].load_id, "load_2");
}

TEST(StateManagerTest, SetLoadsReplacesExisting)
{
  auto manager = StateManager::make();

  Load initial;
  initial.load_id = "old";

  manager->add_load(initial);

  Load replacement;
  replacement.load_id = "new";

  std::vector<Load> loads;
  loads.push_back(replacement);

  manager->set_loads(loads);

  auto state = manager->state();

  ASSERT_TRUE(state.loads.has_value());
  ASSERT_EQ(state.loads->size(), 1);

  EXPECT_EQ((*state.loads)[0].load_id, "new");
}

TEST(StateManagerTest, ClearLoadsRemovesAll)
{
  auto manager = StateManager::make();

  std::vector<Load> loads(2);

  manager->set_loads(loads);
  manager->clear_loads();

  auto state = manager->state();

  ASSERT_TRUE(state.loads.has_value());
  EXPECT_TRUE(state.loads->empty());
}

TEST(StateManagerTest, RemoveLoadsResetsOptional)
{
  auto manager = StateManager::make();

  Load load;
  load.load_id = "load_1";

  manager->add_load(load);

  manager->remove_loads();

  auto state = manager->state();

  EXPECT_FALSE(state.loads.has_value());
}

TEST(StateManagerTest, AddInformationCreatesVector)
{
  auto manager = StateManager::make();

  Info info;
  info.info_type = "test";

  manager->add_information(info);

  auto state = manager->state();

  ASSERT_TRUE(state.information.has_value());
  ASSERT_EQ(state.information->size(), 1);

  EXPECT_EQ((*state.information)[0].info_type, "test");
}

TEST(StateManagerTest, AddInformationAppends)
{
  auto manager = StateManager::make();

  Info info_1;
  info_1.info_type = "info_1";

  Info info_2;
  info_2.info_type = "info_2";

  manager->add_information(info_1);
  manager->add_information(info_2);

  auto state = manager->state();

  ASSERT_TRUE(state.information.has_value());
  ASSERT_EQ(state.information->size(), 2);

  EXPECT_EQ((*state.information)[0].info_type, "info_1");
  EXPECT_EQ((*state.information)[1].info_type, "info_2");
}

TEST(StateManagerTest, SetInformationReplacesExisting)
{
  auto manager = StateManager::make();

  Info old_info;
  old_info.info_type = "old";

  manager->add_information(old_info);

  Info new_info;
  new_info.info_type = "new";

  std::vector<Info> infos;
  infos.push_back(new_info);

  manager->set_information(infos);

  auto state = manager->state();

  ASSERT_TRUE(state.information.has_value());
  ASSERT_EQ(state.information->size(), 1);

  EXPECT_EQ((*state.information)[0].info_type, "new");
}

TEST(StateManagerTest, RemoveInformationResetsOptional)
{
  auto manager = StateManager::make();

  Info info;
  info.info_type = "test";

  manager->add_information(info);

  manager->remove_information();

  auto state = manager->state();

  EXPECT_FALSE(state.information.has_value());
}

TEST(StateManagerTest, MarkAndConsumePublishRequested)
{
  auto manager = StateManager::make();

  EXPECT_FALSE(manager->consume_publish_requested());

  manager->mark_publish_requested();

  EXPECT_TRUE(manager->consume_publish_requested());
  EXPECT_FALSE(manager->consume_publish_requested());
}
