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

#include "vda5050_core/client/adapter/transformation.hpp"

using namespace vda5050_core::client::adapter;  // NOLINT

TEST(TransformationTest, IdentityTransformation)
{
  auto tf =
    Transformation::calibrate(Pose2D{0.0, 0.0, 0.0}, Pose2D{0.0, 0.0, 0.0});

  auto world = tf.to_world_pose(Pose2D{1.0, 2.0, 0.5});

  EXPECT_DOUBLE_EQ(world.x, 1.0);
  EXPECT_DOUBLE_EQ(world.y, 2.0);
  EXPECT_DOUBLE_EQ(world.theta, 0.5);
}

TEST(TransformationTest, CalibratesTranslation)
{
  auto tf =
    Transformation::calibrate(Pose2D{1.0, 2.0, 0.0}, Pose2D{10.0, 20.0, 0.0});

  auto world = tf.to_world_pose(Pose2D{10.0, 20.0, 0.0});

  EXPECT_DOUBLE_EQ(world.x, 1.0);
  EXPECT_DOUBLE_EQ(world.y, 2.0);
  EXPECT_DOUBLE_EQ(world.theta, 0.0);
}

TEST(TransformationTest, CalibratesRotation)
{
  auto tf = Transformation::calibrate(
    Pose2D{0.0, 0.0, M_PI / 2.0}, Pose2D{0.0, 0.0, 0.0});

  auto world = tf.to_world_pose(Pose2D{1.0, 0.0, 0.0});

  EXPECT_NEAR(world.x, 0.0, 1e-6);
  EXPECT_NEAR(world.y, 1.0, 1e-6);
  EXPECT_NEAR(world.theta, M_PI / 2.0, 1e-6);
}

TEST(TransformationTest, ConvertsWorldPoseBackToAgv)
{
  auto tf =
    Transformation::calibrate(Pose2D{1.0, 2.0, 0.0}, Pose2D{10.0, 20.0, 0.0});

  auto agv = tf.to_agv_pose(Pose2D{1.0, 2.0, 0.0});

  EXPECT_DOUBLE_EQ(agv.x, 10.0);
  EXPECT_DOUBLE_EQ(agv.y, 20.0);
  EXPECT_DOUBLE_EQ(agv.theta, 0.0);
}

TEST(TransformationTest, WorldAgvRoundTrip)
{
  auto tf =
    Transformation::calibrate(Pose2D{5.0, 10.0, 0.3}, Pose2D{20.0, 30.0, -0.2});

  auto original_agv = Pose2D{4.0, 7.0, 1.0};

  auto world = tf.to_world_pose(original_agv);
  auto result = tf.to_agv_pose(world);

  EXPECT_NEAR(result.x, original_agv.x, 1e-6);
  EXPECT_NEAR(result.y, original_agv.y, 1e-6);
  EXPECT_NEAR(result.theta, original_agv.theta, 1e-6);
}

TEST(TransformationTest, NormalizesAngles)
{
  auto tf = Transformation::calibrate(
    Pose2D{0.0, 0.0, 4 * M_PI}, Pose2D{0.0, 0.0, 0.0});

  auto world = tf.to_world_pose(Pose2D{0.0, 0.0, 0.0});

  EXPECT_NEAR(world.theta, 0.0, 1e-6);
}
