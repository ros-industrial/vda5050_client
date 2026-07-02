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

#include "vda5050_master_ros2/internal/ros2_topic_naming.hpp"

namespace vda5050_master_ros2::internal::test {

// A segment that would otherwise start with a digit is prefixed with a letter
// (not '_') so the resulting topic is not a hidden ROS 2 topic.
TEST(Ros2TopicNamingTest, LeadingDigit_IsPrefixedWithLetter)
{
  EXPECT_EQ(to_ros2_topic_segment("001"), "n001");
  EXPECT_EQ(to_ros2_topic_segment("3M"), "n3M");
  EXPECT_EQ(to_ros2_topic_segment("0"), "n0");
}

TEST(Ros2TopicNamingTest, LeadingLetter_PassesThroughUnchanged)
{
  EXPECT_EQ(to_ros2_topic_segment("KION"), "KION");
  EXPECT_EQ(to_ros2_topic_segment("S001"), "S001");
  EXPECT_EQ(to_ros2_topic_segment("a"), "a");
}

// A leading '_' is legal but hidden, so it is also prefixed with a letter.
TEST(Ros2TopicNamingTest, LeadingUnderscore_IsPrefixedWithLetter)
{
  EXPECT_EQ(to_ros2_topic_segment("_internal"), "n_internal");
  EXPECT_EQ(to_ros2_topic_segment("_"), "n_");
  EXPECT_EQ(to_ros2_topic_segment("_001"), "n_001");
}

TEST(Ros2TopicNamingTest, EmptyString_IsPrefixedWithLetter)
{
  EXPECT_EQ(to_ros2_topic_segment(""), "n");
}

TEST(Ros2TopicNamingTest, AllInvalidChars_ReplacedThenPrefixed)
{
  EXPECT_EQ(to_ros2_topic_segment("---"), "n___");
}

TEST(Ros2TopicNamingTest, MixedAlphanumeric_OnlyLeadingMatters)
{
  EXPECT_EQ(to_ros2_topic_segment("SN001"), "SN001");
  EXPECT_EQ(to_ros2_topic_segment("agv_42"), "agv_42");
  EXPECT_EQ(to_ros2_topic_segment("42agv"), "n42agv");
}

TEST(Ros2TopicNamingTest, Hyphen_IsReplacedWithUnderscore)
{
  EXPECT_EQ(to_ros2_topic_segment("KION-001"), "KION_001");
  EXPECT_EQ(to_ros2_topic_segment("JG-12-7"), "JG_12_7");
  EXPECT_EQ(to_ros2_topic_segment("-leading"), "n_leading");
  EXPECT_EQ(to_ros2_topic_segment("trailing-"), "trailing_");
}

TEST(Ros2TopicNamingTest, Period_IsReplacedWithUnderscore)
{
  EXPECT_EQ(to_ros2_topic_segment("agv.42"), "agv_42");
  EXPECT_EQ(to_ros2_topic_segment("a.b.c"), "a_b_c");
  EXPECT_EQ(to_ros2_topic_segment("3.14"), "n3_14");
}

TEST(Ros2TopicNamingTest, Colon_IsReplacedWithUnderscore)
{
  EXPECT_EQ(to_ros2_topic_segment("vendor:robot:7"), "vendor_robot_7");
  EXPECT_EQ(to_ros2_topic_segment(":start"), "n_start");
}

TEST(Ros2TopicNamingTest, OtherSpecialChars_AlsoReplaced)
{
  EXPECT_EQ(to_ros2_topic_segment("Robot Company"), "Robot_Company");
  EXPECT_EQ(to_ros2_topic_segment("a@b"), "a_b");
  EXPECT_EQ(to_ros2_topic_segment("a/b"), "a_b");
}

// Distinct raw identities that sanitize to the same segment (the collision the
// master detects and refuses at registration).
TEST(Ros2TopicNamingTest, DistinctIdentitiesCanCollide)
{
  EXPECT_EQ(
    to_ros2_topic_segment("KION-001"), to_ros2_topic_segment("KION_001"));
  EXPECT_EQ(to_ros2_topic_segment("001"), to_ros2_topic_segment("n001"));
}

TEST(Ros2TopicNamingTest, NeedsSanitization_MatchesTransformation)
{
  // Contract: needs_topic_sanitization(s) == true iff to_ros2_topic_segment(s)
  // returns something different from s.
  EXPECT_TRUE(needs_topic_sanitization("001"));
  EXPECT_TRUE(needs_topic_sanitization("3M"));
  EXPECT_TRUE(needs_topic_sanitization(""));
  EXPECT_TRUE(needs_topic_sanitization("KION-001"));
  EXPECT_TRUE(needs_topic_sanitization("agv.42"));
  EXPECT_TRUE(needs_topic_sanitization("vendor:robot"));
  EXPECT_TRUE(needs_topic_sanitization("_001"));
  EXPECT_FALSE(needs_topic_sanitization("KION"));
  EXPECT_FALSE(needs_topic_sanitization("S001"));
  EXPECT_FALSE(needs_topic_sanitization("agv_42"));
}

// build_per_agv_topic assembles "/<ns>/<mfg>/<serial>/<leaf>", sanitizing only
// the mfg and serial segments; namespace and leaf pass through untouched.
TEST(Ros2TopicNamingTest, BuildPerAgvTopic_WithNamespace)
{
  EXPECT_EQ(
    build_per_agv_topic("vda5050_master", "KION", "S001", "state"),
    "/vda5050_master/KION/S001/state");
}

TEST(Ros2TopicNamingTest, BuildPerAgvTopic_EmptyNamespace)
{
  EXPECT_EQ(
    build_per_agv_topic("", "KION", "S001", "state"), "/KION/S001/state");
}

TEST(Ros2TopicNamingTest, BuildPerAgvTopic_SanitizesOnlyMfgAndSerial)
{
  // Invalid mfg/serial are transformed; namespace and leaf are not.
  EXPECT_EQ(
    build_per_agv_topic("vda5050_master", "KION-001", "001", "order_status"),
    "/vda5050_master/KION_001/n001/order_status");
}

}  // namespace vda5050_master_ros2::internal::test
