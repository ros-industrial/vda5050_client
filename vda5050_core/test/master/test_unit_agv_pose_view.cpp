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

TEST_F(AGVPoseViewTestFixture, UninitializedStatePositionYieldsSourceNone)
{
  auto& agv = create_agv();
  auto state = create_state_msg();
  auto pos = make_pos(5.0, 5.0, 0.0);
  pos.position_initialized = false;  // un-localized — not a usable position
  state.agv_position = pos;
  agv->handle_state(state);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::None);
  EXPECT_FALSE(view.agv_position.has_value());
}

TEST_F(
  AGVPoseViewTestFixture, FresherButUninitializedVizLosesToInitializedState)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  auto state = create_state_msg();
  state.header.timestamp = base;
  state.agv_position = make_pos(1.0, 1.0, 0.0);  // initialized
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  viz.header.timestamp = base + std::chrono::seconds(1);  // fresher...
  auto vpos = make_pos(9.0, 9.0, 0.0);
  vpos.position_initialized = false;  // ...but un-localized, so unusable
  viz.agv_position = vpos;
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::State);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 1.0);
}

TEST_F(AGVPoseViewTestFixture, EqualHeaderTimestampPrefersVisualization)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  auto state = create_state_msg();
  state.header.timestamp = base;
  state.agv_position = make_pos(1.0, 1.0, 0.0);
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  viz.header.timestamp = base;  // equal timestamp — viz wins via >=
  viz.agv_position = make_pos(9.0, 9.0, 0.0);
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 9.0);
}

TEST_F(AGVPoseViewTestFixture, StaleVisualizationIsDroppedAndDoesNotWin)
{
  auto& agv = create_agv();

  auto viz1 = create_visualization_msg();
  viz1.header.header_id = 20;
  viz1.agv_position = make_pos(9.0, 9.0, 0.0);
  agv->handle_visualization(viz1);

  // Reordered older Visualization (lower header_id) must be dropped, so the
  // pose keeps reporting the newer sample rather than rolling back.
  auto viz0 = create_visualization_msg();
  viz0.header.header_id = 19;
  viz0.agv_position = make_pos(1.0, 1.0, 0.0);
  agv->handle_visualization(viz0);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 9.0);
}

TEST_F(AGVPoseViewTestFixture, VelocityComesFromWinningSource)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  auto state = create_state_msg();
  state.header.timestamp = base;
  state.agv_position = make_pos(1.0, 1.0, 0.0);
  state.velocity = make_vel(0.5);
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  viz.header.timestamp = base + std::chrono::seconds(1);
  viz.agv_position = make_pos(9.0, 9.0, 0.0);
  viz.velocity = make_vel(0.9);
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.velocity.has_value());
  ASSERT_TRUE(view.velocity->vx.has_value());
  EXPECT_DOUBLE_EQ(*view.velocity->vx, 0.9);  // winning source, not state's 0.5
}

TEST_F(AGVPoseViewTestFixture, UninitializedStateWithInitializedVizUsesViz)
{
  auto& agv = create_agv();
  const auto base = std::chrono::system_clock::now();

  auto state = create_state_msg();
  state.header.timestamp = base + std::chrono::seconds(1);  // fresher...
  auto spos = make_pos(1.0, 1.0, 0.0);
  spos.position_initialized = false;  // ...but un-localized, so unusable
  state.agv_position = spos;
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  viz.header.timestamp = base;
  viz.agv_position = make_pos(9.0, 9.0, 0.0);  // initialized
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 9.0);
}

TEST_F(AGVPoseViewTestFixture, BothPositionsUninitializedYieldsSourceNone)
{
  auto& agv = create_agv();

  auto state = create_state_msg();
  auto spos = make_pos(1.0, 1.0, 0.0);
  spos.position_initialized = false;
  state.agv_position = spos;
  agv->handle_state(state);

  auto viz = create_visualization_msg();
  auto vpos = make_pos(9.0, 9.0, 0.0);
  vpos.position_initialized = false;
  viz.agv_position = vpos;
  agv->handle_visualization(viz);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::None);
  EXPECT_FALSE(view.agv_position.has_value());
}

TEST_F(AGVPoseViewTestFixture, ReconnectResetsVizGateSoLowerHeaderAccepted)
{
  auto& agv = create_agv();
  agv->handle_connection(create_connection_msg("ONLINE"));

  auto viz_hi = create_visualization_msg();
  viz_hi.header.header_id = 20;
  viz_hi.agv_position = make_pos(9.0, 9.0, 0.0);
  agv->handle_visualization(viz_hi);

  // Reconnect edge (OFFLINE -> ONLINE) resets the visualization gate.
  agv->handle_connection(create_connection_msg("OFFLINE"));
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Restarted AGV: a lower header_id must be accepted, not dropped as stale.
  auto viz_lo = create_visualization_msg();
  viz_lo.header.header_id = 3;
  viz_lo.agv_position = make_pos(1.0, 1.0, 0.0);
  agv->handle_visualization(viz_lo);

  const auto view = agv->get_pose_view();
  EXPECT_EQ(view.source, PoseSource::Visualization);
  ASSERT_TRUE(view.agv_position.has_value());
  EXPECT_DOUBLE_EQ(view.agv_position->x, 1.0);
}

}  // namespace vda5050_core::master::test
