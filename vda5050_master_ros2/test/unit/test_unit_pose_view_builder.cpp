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

#include "vda5050_core/master/pose_view.hpp"
#include "vda5050_master_ros2/pose_view_builder.hpp"

namespace vda5050_master_ros2 {
namespace test {
namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using PoseViewMsg = vda5050_master_ros2::msg::PoseView;

vda5050_core::master::PoseView make_view(vda5050_core::master::PoseSource src)
{
  vda5050_core::master::PoseView v;
  v.source = src;
  v.driving = true;
  v.data_age = std::chrono::milliseconds(250);

  vda5050_core::types::AGVPosition p;
  p.x = 1.5;
  p.y = 2.5;
  p.theta = 0.3;
  p.map_id = "m";
  v.agv_position = p;

  vda5050_core::types::Velocity vel;
  vel.vx = 0.4;
  v.velocity = vel;
  return v;
}

}  // namespace

TEST(PoseViewBuilderTest, MapsStateSourceAndPayload)
{
  const auto view = make_view(vda5050_core::master::PoseSource::State);
  const PoseViewMsg msg = build_pose_view_msg(view, kMfg, kSerial);

  EXPECT_EQ(msg.manufacturer, kMfg);
  EXPECT_EQ(msg.serial_number, kSerial);
  EXPECT_EQ(msg.source, PoseViewMsg::SOURCE_STATE);
  EXPECT_TRUE(msg.driving);
  ASSERT_EQ(msg.agv_position.size(), 1u);
  EXPECT_DOUBLE_EQ(msg.agv_position[0].x, 1.5);
  ASSERT_EQ(msg.velocity.size(), 1u);
  EXPECT_EQ(msg.data_age.sec, 0);
  EXPECT_GT(msg.data_age.nanosec, 0u);
}

TEST(PoseViewBuilderTest, MapsVisualizationSource)
{
  const auto view = make_view(vda5050_core::master::PoseSource::Visualization);
  const PoseViewMsg msg = build_pose_view_msg(view, kMfg, kSerial);
  EXPECT_EQ(msg.source, PoseViewMsg::SOURCE_VISUALIZATION);
}

TEST(PoseViewBuilderTest, NoneSourceLeavesPayloadEmpty)
{
  // Default PoseView: source None, no position / velocity.
  const vda5050_core::master::PoseView view;
  const PoseViewMsg msg = build_pose_view_msg(view, kMfg, kSerial);
  EXPECT_EQ(msg.source, PoseViewMsg::SOURCE_NONE);
  EXPECT_TRUE(msg.agv_position.empty());
  EXPECT_TRUE(msg.velocity.empty());
}

}  // namespace test
}  // namespace vda5050_master_ros2
