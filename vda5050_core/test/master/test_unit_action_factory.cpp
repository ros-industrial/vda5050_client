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

#include <set>
#include <string>
#include <vector>

#include "vda5050_core/master/actions/action_factory.hpp"

namespace vda5050_core::master {
namespace test {

TEST(ActionFactory, BuildCustomAllParamsExplicit)
{
  vda5050_core::types::ActionParameter p;
  p.key = "loadType";
  p.value = "EUR1";
  std::vector<vda5050_core::types::ActionParameter> params{p};

  auto a = ActionFactory::build_custom(
    "pick", "act-001", vda5050_core::types::BlockingType::HARD, "pick a load",
    params);

  EXPECT_EQ(a.action_type, "pick");
  EXPECT_EQ(a.action_id, "act-001");
  EXPECT_EQ(a.blocking_type, vda5050_core::types::BlockingType::HARD);
  ASSERT_TRUE(a.action_description.has_value());
  EXPECT_EQ(*a.action_description, "pick a load");
  ASSERT_TRUE(a.action_parameters.has_value());
  ASSERT_EQ(a.action_parameters->size(), 1u);
  EXPECT_EQ(a.action_parameters->front().key, "loadType");
  EXPECT_EQ(a.action_parameters->front().value, "EUR1");
}

TEST(ActionFactory, BuildCustomDefaultBlockingTypeIsNone)
{
  auto a = ActionFactory::build_custom("stateRequest", "act-002");
  EXPECT_EQ(a.blocking_type, vda5050_core::types::BlockingType::NONE);
}

TEST(ActionFactory, BuildCustomEmptyDescriptionLeavesOptionalUnset)
{
  // Per VDA5050 §6.4: omitting optional fields keeps wire payload compact.
  auto a = ActionFactory::build_custom("stateRequest", "act-003");
  EXPECT_FALSE(a.action_description.has_value());
}

TEST(ActionFactory, BuildCustomEmptyParametersLeavesOptionalUnset)
{
  auto a = ActionFactory::build_custom("stateRequest", "act-004");
  EXPECT_FALSE(a.action_parameters.has_value());
}

TEST(ActionFactory, BuildCustomEmptyActionIdStillBuilds)
{
  // Empty action_id is allowed at the factory layer — uniqueness/empty
  // rejection happens at master.assign_instant_actions.
  auto a = ActionFactory::build_custom("stateRequest", "");
  EXPECT_EQ(a.action_id, "");
}

TEST(ActionFactory, GenerateActionIdHasUuidV4Shape)
{
  // RFC 4122 §3 textual layout: 8-4-4-4-12 lowercase hex with dashes,
  // 36 chars total. Version 4 sets nibble 12 to '4' (high nibble of
  // byte 6). Variant 1 sets nibble 16 to '8', '9', 'a', or 'b'.
  const auto id = ActionFactory::generate_action_id();
  ASSERT_EQ(id.size(), 36u);
  EXPECT_EQ(id[8], '-');
  EXPECT_EQ(id[13], '-');
  EXPECT_EQ(id[18], '-');
  EXPECT_EQ(id[23], '-');
  EXPECT_EQ(id[14], '4') << "version-4 nibble must be 4";
  const char variant_nibble = id[19];
  EXPECT_TRUE(
    variant_nibble == '8' || variant_nibble == '9' || variant_nibble == 'a' ||
    variant_nibble == 'b')
    << "variant nibble must be 8/9/a/b, got '" << variant_nibble << "'";
  for (size_t i = 0; i < id.size(); ++i)
  {
    if (i == 8 || i == 13 || i == 18 || i == 23) continue;
    const char c = id[i];
    EXPECT_TRUE((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))
      << "char " << i << " '" << c << "' is not lowercase hex";
  }
}

TEST(ActionFactory, GenerateActionIdBatchOf100AllUnique)
{
  std::set<std::string> seen;
  for (int i = 0; i < 100; ++i)
  {
    auto id = ActionFactory::generate_action_id();
    EXPECT_TRUE(seen.insert(id).second)
      << "duplicate UUID generated at iteration " << i << ": " << id;
  }
  EXPECT_EQ(seen.size(), 100u);
}

// =============================================================================
// V0 BACKLOG predefined-action factories (Task #38)
// =============================================================================

TEST(ActionFactory, BuildStateRequest_HasCanonicalActionType)
{
  auto a = ActionFactory::build_state_request("req-1");
  EXPECT_EQ(a.action_type, "stateRequest");
  EXPECT_EQ(a.action_id, "req-1");
}

TEST(ActionFactory, BuildStateRequest_DefaultsToNoneBlocking)
{
  // §6.8.1: stateRequest is a query; NONE blocking is the safe default
  // (parallel + driving OK).
  auto a = ActionFactory::build_state_request("req-1");
  EXPECT_EQ(a.blocking_type, vda5050_core::types::BlockingType::NONE);
}

TEST(ActionFactory, BuildStateRequest_NoParametersAttached)
{
  // §6.8.1 stateRequest takes no parameters; optional must remain unset.
  auto a = ActionFactory::build_state_request("req-1");
  EXPECT_FALSE(a.action_parameters.has_value());
}

TEST(ActionFactory, BuildStateRequest_DescriptionPropagates)
{
  auto a = ActionFactory::build_state_request("req-1", "for liveness check");
  ASSERT_TRUE(a.action_description.has_value());
  EXPECT_EQ(*a.action_description, "for liveness check");
}

TEST(ActionFactory, BuildStateRequest_EmptyDescriptionLeavesOptionalUnset)
{
  auto a = ActionFactory::build_state_request("req-1");
  EXPECT_FALSE(a.action_description.has_value());
}

TEST(ActionFactory, BuildFactsheetRequest_HasCanonicalActionType)
{
  auto a = ActionFactory::build_factsheet_request("fs-1");
  EXPECT_EQ(a.action_type, "factsheetRequest");
  EXPECT_EQ(a.action_id, "fs-1");
}

TEST(ActionFactory, BuildFactsheetRequest_DefaultsToNoneBlocking)
{
  auto a = ActionFactory::build_factsheet_request("fs-1");
  EXPECT_EQ(a.blocking_type, vda5050_core::types::BlockingType::NONE);
}

TEST(ActionFactory, BuildFactsheetRequest_NoParametersAttached)
{
  // §6.8.1 factsheetRequest takes no parameters; optional must remain unset.
  auto a = ActionFactory::build_factsheet_request("fs-1");
  EXPECT_FALSE(a.action_parameters.has_value());
}

TEST(ActionFactory, BuildFactsheetRequest_DescriptionPropagates)
{
  auto a = ActionFactory::build_factsheet_request("fs-1", "post-firmware");
  ASSERT_TRUE(a.action_description.has_value());
  EXPECT_EQ(*a.action_description, "post-firmware");
}

}  // namespace test
}  // namespace vda5050_core::master
