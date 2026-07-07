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

#include <chrono>

#include "test_fixture_agv.hpp"

namespace vda5050_core::master::test {

using AGVPoseViewTestFixture = AGVTestFixture;

namespace {

vda5050_core::types::AGVPosition make_pos(double x, double y, double theta)
{
  vda5050_core::types::AGVPosition p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  p.map_id = "map1";
  p.position_initialized = true;
  return p;
}

vda5050_core::types::Velocity make_vel(double vx)
{
  vda5050_core::types::Velocity v;
  v.vx = vx;
  return v;
}

}  // namespace

TEST_F(AGVPoseViewTestFixture, NoMessagesYieldsSourceNone)
{
  auto& agv = create_agv();
  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::None);
  EXPECT_FALSE(view.agv_position.has_value());
  EXPECT_FALSE(view.velocity.has_value());
}

TEST_F(AGVPoseViewTestFixture, StateWithPositionUsesStateSource)
{
  auto& agv = create_agv();
  auto state = create_state_msg();
  state.driving = true;
  state.agv_position = make_pos(1.0, 2.0, 0.5);
  state.velocity = make_vel(0.7);
  agv->handle_state(state);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::State);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 1.0);
  ASSERT_TRUE(view.velocity.has_value());
  EXPECT_TRUE(view.driving);
  EXPECT_GE(view.data_age.count(), 0);
}

TEST_F(AGVPoseViewTestFixture, VisualizationWithPositionUsesVisualizationSource)
{
  auto& agv = create_agv();
  auto viz = create_visualization_msg();
  viz.agv_position = make_pos(3.0, 4.0, 1.0);
  viz.velocity = make_vel(0.2);
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 3.0);
}

TEST_F(AGVPoseViewTestFixture, FresherVisualizationWinsByHeaderTimestamp)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  auto state = create_state_msg();
  state.header.timestamp = base;
  state.agv_position = make_pos(1.0, 1.0, 0.0);
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  viz.header.timestamp = base + std::chrono::seconds(1);
  viz.agv_position = make_pos(9.0, 9.0, 0.0);
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 9.0);
}

TEST_F(AGVPoseViewTestFixture, FresherStateWinsByHeaderTimestamp)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  auto viz = create_visualization_msg();
  viz.header.timestamp = base;
  viz.agv_position = make_pos(9.0, 9.0, 0.0);
  agv->handle_visualization(viz);

  auto state = create_state_msg();
  state.header.timestamp = base + std::chrono::seconds(1);
  state.driving = true;
  state.agv_position = make_pos(1.0, 1.0, 0.0);
  agv->handle_state(state);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::State);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 1.0);
  EXPECT_TRUE(view.driving);
}

TEST_F(
  AGVPoseViewTestFixture, DrivingRelayedFromStateWhenVisualizationSuppliesPose)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  // State carries driving but no position; freshest position is from viz.
  auto state = create_state_msg();
  state.header.timestamp = base;
  state.driving = true;
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  viz.header.timestamp = base + std::chrono::seconds(1);
  viz.agv_position = make_pos(2.0, 2.0, 0.0);
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  EXPECT_TRUE(view.driving);
}

}  // namespace vda5050_core::master::test
