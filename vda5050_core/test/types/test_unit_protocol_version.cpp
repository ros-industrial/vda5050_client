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

#include "vda5050_core/types/protocol_version.hpp"

namespace {

using vda5050_core::types::ProtocolVersion;

// Test 1: Parsing a supported version string returns the matching constant.
TEST(ProtocolVersionTest, FromStringParsesSupportedVersion)
{
  EXPECT_EQ(ProtocolVersion::from_string("2.0.0"), ProtocolVersion::V2_0_0);
}

// Test 2: Parsing an unsupported version string returns std::nullopt.
TEST(ProtocolVersionTest, FromStringReturnsNulloptForUnsupportedVersion)
{
  EXPECT_FALSE(ProtocolVersion::from_string("9.9.9").has_value());
}

// Test 3: Parsing an empty string returns std::nullopt.
TEST(ProtocolVersionTest, FromStringReturnsNulloptForEmptyString)
{
  EXPECT_FALSE(ProtocolVersion::from_string("").has_value());
}

// Test 4: to_string() returns the full semantic version.
TEST(ProtocolVersionTest, ToStringReturnsSemanticVersion)
{
  EXPECT_EQ(ProtocolVersion::V2_0_0.to_string(), "2.0.0");
}

// Test 5: to_topic_version() returns the MQTT topic version segment.
TEST(ProtocolVersionTest, TopicVersionReturnsMajorSegment)
{
  EXPECT_EQ(ProtocolVersion::V2_0_0.to_topic_version(), "v2");
}

// Test 6: supported_versions() enumerates every supported version.
TEST(ProtocolVersionTest, SupportedVersionsContainsV2_0_0)
{
  const auto& supported = ProtocolVersion::supported_versions();
  EXPECT_EQ(supported.size(), 1u);
  EXPECT_EQ(supported[0], ProtocolVersion::V2_0_0);
}

// Test 7: Equal ProtocolVersion values compare equal via both operators.
TEST(ProtocolVersionTest, EqualityOperators)
{
  EXPECT_EQ(ProtocolVersion::V2_0_0, ProtocolVersion::from_string("2.0.0"));
  EXPECT_FALSE(
    ProtocolVersion::V2_0_0 != ProtocolVersion::from_string("2.0.0").value());
}

// Test 8: Ordering operators compare major.minor.patch lexicographically.
// There is only one supported version today, so this exercises the operators
// directly against from_string() results rather than a lower/higher constant.
TEST(ProtocolVersionTest, OrderingOperators)
{
  const auto version = ProtocolVersion::from_string("2.0.0").value();

  EXPECT_TRUE(version <= ProtocolVersion::V2_0_0);
  EXPECT_TRUE(version >= ProtocolVersion::V2_0_0);
  EXPECT_FALSE(version < ProtocolVersion::V2_0_0);
  EXPECT_FALSE(version > ProtocolVersion::V2_0_0);
}

}  // namespace
