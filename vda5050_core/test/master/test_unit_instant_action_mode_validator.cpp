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

#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/validation/instant_action_mode_validator.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master {
namespace test {

namespace {

vda5050_core::types::Action make_action(
  const std::string& type, const std::string& id = "act-1")
{
  vda5050_core::types::Action a;
  a.action_type = type;
  a.action_id = id;
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  return a;
}

vda5050_core::types::State make_state(vda5050_core::types::OperatingMode mode)
{
  vda5050_core::types::State s;
  s.operating_mode = mode;
  return s;
}

PreSendContext make_ctx(const vda5050_core::types::State& state)
{
  return PreSendContext{
    vda5050_core::types::ConnectionState::ONLINE, state, std::nullopt,
    AGVState::AVAILABLE, nullptr};
}

vda5050_core::types::InstantActions wrap(
  const std::vector<vda5050_core::types::Action>& actions)
{
  vda5050_core::types::InstantActions msg;
  msg.actions = actions;
  return msg;
}

}  // namespace

// =============================================================================
// Allowlist membership
// =============================================================================

TEST(InstantActionModeValidator, ExemptList_MatchesPredefinedInstantScope)
{
  // Instant-scope predefined actions designed for non-AUTOMATIC use.
  EXPECT_TRUE(is_mode_exempt_action_type("stateRequest"));
  EXPECT_TRUE(is_mode_exempt_action_type("factsheetRequest"));
  EXPECT_TRUE(is_mode_exempt_action_type("logReport"));
  EXPECT_TRUE(is_mode_exempt_action_type("cancelOrder"));
  EXPECT_TRUE(is_mode_exempt_action_type("initPosition"));
  EXPECT_TRUE(is_mode_exempt_action_type("startPause"));
  EXPECT_TRUE(is_mode_exempt_action_type("stopPause"));
  EXPECT_TRUE(is_mode_exempt_action_type("startCharging"));
  EXPECT_TRUE(is_mode_exempt_action_type("stopCharging"));
}

TEST(InstantActionModeValidator, NonExempt_ActionTypes_Rejected)
{
  // Common operational actions that are NOT instant-scope
  // (typically node/edge actions or custom).
  EXPECT_FALSE(is_mode_exempt_action_type("pick"));
  EXPECT_FALSE(is_mode_exempt_action_type("drop"));
  EXPECT_FALSE(is_mode_exempt_action_type("detectObject"));
  EXPECT_FALSE(is_mode_exempt_action_type("finePositioning"));
  EXPECT_FALSE(is_mode_exempt_action_type("waitForTrigger"));
  // Custom action types
  EXPECT_FALSE(is_mode_exempt_action_type("customAction"));
  EXPECT_FALSE(is_mode_exempt_action_type(""));
}

// =============================================================================
// Validator behavior — degraded mode
// =============================================================================

TEST(InstantActionModeValidator, NoStateInContext_Passes)
{
  PreSendContext ctx{
    vda5050_core::types::ConnectionState::ONLINE, std::nullopt, std::nullopt,
    AGVState::AVAILABLE, nullptr};
  auto res =
    validate_instant_action_mode(ctx, wrap({make_action("customAction")}));
  EXPECT_TRUE(static_cast<bool>(res));
}

// =============================================================================
// Validator behavior — AUTOMATIC / SEMIAUTOMATIC always pass
// =============================================================================

TEST(InstantActionModeValidator, Automatic_NonExemptAction_Passes)
{
  auto state = make_state(vda5050_core::types::OperatingMode::AUTOMATIC);
  auto res =
    validate_instant_action_mode(make_ctx(state), wrap({make_action("pick")}));
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(InstantActionModeValidator, Semiautomatic_NonExemptAction_Passes)
{
  // SEMIAUTOMATIC has the master "in control"; treated like AUTOMATIC.
  auto state = make_state(vda5050_core::types::OperatingMode::SEMIAUTOMATIC);
  auto res = validate_instant_action_mode(
    make_ctx(state), wrap({make_action("customAction")}));
  EXPECT_TRUE(static_cast<bool>(res));
}

// =============================================================================
// Validator behavior — non-AUTOMATIC modes enforce allowlist
// =============================================================================

TEST(InstantActionModeValidator, Manual_ExemptAction_Passes)
{
  auto state = make_state(vda5050_core::types::OperatingMode::MANUAL);
  auto res = validate_instant_action_mode(
    make_ctx(state), wrap({make_action("stateRequest")}));
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(InstantActionModeValidator, Manual_NonExemptAction_Rejected)
{
  auto state = make_state(vda5050_core::types::OperatingMode::MANUAL);
  auto res =
    validate_instant_action_mode(make_ctx(state), wrap({make_action("pick")}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::ModeValidationError);
}

TEST(InstantActionModeValidator, Service_NonExemptAction_Rejected)
{
  auto state = make_state(vda5050_core::types::OperatingMode::SERVICE);
  auto res = validate_instant_action_mode(
    make_ctx(state), wrap({make_action("customAction")}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::ModeValidationError);
}

TEST(InstantActionModeValidator, Teachin_ExemptAction_Passes)
{
  // initPosition during TEACHIN is a legitimate use case (mapping teaching
  // includes pose initialization).
  auto state = make_state(vda5050_core::types::OperatingMode::TEACHIN);
  auto res = validate_instant_action_mode(
    make_ctx(state), wrap({make_action("initPosition")}));
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(
  InstantActionModeValidator, Manual_MixedBatch_NonExemptShortCircuitsRejection)
{
  // Batch order: [stateRequest (exempt), pick (non-exempt)]. Validator
  // walks the batch and rejects on the first non-exempt action in
  // non-AUTOMATIC mode.
  auto state = make_state(vda5050_core::types::OperatingMode::MANUAL);
  std::vector<vda5050_core::types::Action> batch = {
    make_action("stateRequest", "ok"),
    make_action("pick", "bad"),
  };
  auto res = validate_instant_action_mode(make_ctx(state), wrap(batch));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_EQ(res.errors.size(), 1u);
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_NE(
    res.errors.front().error_description->find("pick"), std::string::npos);
}

// =============================================================================
// Tagging
// =============================================================================

TEST(InstantActionModeValidator, RejectionTaggedAsModeValidationError)
{
  auto state = make_state(vda5050_core::types::OperatingMode::MANUAL);
  auto res =
    validate_instant_action_mode(make_ctx(state), wrap({make_action("pick")}));
  ASSERT_FALSE(res.errors.empty());
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::ModeValidationError);
}

}  // namespace test
}  // namespace vda5050_core::master
