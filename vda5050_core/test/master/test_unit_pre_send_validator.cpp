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

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/master/validation/pre_send_validator.hpp"

namespace {

// Returns true iff every error in `res` has type PreSendValidationError.
::testing::AssertionResult AllErrorsHavePreSendType(
  const vda5050_core::order_utils::ValidationResult& res)
{
  for (const auto& e : res.errors)
  {
    if (e.error_type != vda5050_core::errors::PreSendValidationError)
    {
      return ::testing::AssertionFailure()
             << "found error with type '" << e.error_type
             << "' (expected PreSendValidationError)";
    }
  }
  return ::testing::AssertionSuccess();
}

// Returns true iff at least one error's description contains `needle`.
::testing::AssertionResult AnyErrorMentions(
  const vda5050_core::order_utils::ValidationResult& res,
  const std::string& needle)
{
  for (const auto& e : res.errors)
  {
    if (
      e.error_description &&
      e.error_description->find(needle) != std::string::npos)
    {
      return ::testing::AssertionSuccess();
    }
  }
  return ::testing::AssertionFailure()
         << "no error description mentions '" << needle << "'";
}

}  // namespace

namespace vda5050_core::master::test {

namespace {

// Builds a State that satisfies all PreSend checks: AUTOMATIC mode +
// initialized position. Tests clobber individual fields to exercise
// specific failure paths.
vda5050_core::types::State make_ready_state()
{
  vda5050_core::types::State s;
  s.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  vda5050_core::types::AGVPosition pos;
  pos.position_initialized = true;
  s.agv_position = pos;
  return s;
}

// Builds a PreSendContext that satisfies all readiness checks.
PreSendContext make_ready_context()
{
  return PreSendContext{
    vda5050_core::types::ConnectionState::ONLINE, make_ready_state(),
    std::nullopt, AGVState::AVAILABLE, nullptr};
}

}  // namespace

// ============================================================================
// Happy path — same gate is shared between Order + InstantActions publishers,
// so a single ctx-passes test locks in the both publishers' outgoing paths.
// ============================================================================

TEST(PreSendValidatorTest, ValidContextPasses)
{
  auto ctx = make_ready_context();
  auto res = validate_pre_send(ctx);
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_TRUE(res.errors.empty());
}

// ============================================================================
// Connection status
// ============================================================================

TEST(PreSendValidatorTest, OfflineConnectionRejected)
{
  auto ctx = make_ready_context();
  ctx.connection_status = vda5050_core::types::ConnectionState::OFFLINE;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "connection_status"));
}

TEST(PreSendValidatorTest, ConnectionBrokenRejected)
{
  auto ctx = make_ready_context();
  ctx.connection_status =
    vda5050_core::types::ConnectionState::CONNECTIONBROKEN;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "connection_status"));
}

// ============================================================================
// Operational state
// ============================================================================

TEST(PreSendValidatorTest, ErrorOperationalStateRejected)
{
  auto ctx = make_ready_context();
  ctx.operational_state = AGVState::ERROR;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "operational_state"));
  EXPECT_TRUE(AnyErrorMentions(res, "ERROR"));
}

// Silent AGV (state heartbeat exceeded 30s) must be rejected
// at pre-send. The PreSendContext snapshot may still carry a valid
// last_state from before the silence, so the existing "no last_state"
// short-circuit doesn't catch this case.
TEST(PreSendValidatorTest, StateUnknownOperationalStateRejected)
{
  auto ctx = make_ready_context();
  ctx.operational_state = AGVState::STATE_UNKNOWN;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "operational_state"));
  EXPECT_TRUE(AnyErrorMentions(res, "STATE_UNKNOWN"));
}

TEST(PreSendValidatorTest, UnavailableOperationalStateRejected)
{
  auto ctx = make_ready_context();
  ctx.operational_state = AGVState::UNAVAILABLE;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "operational_state"));
  EXPECT_TRUE(AnyErrorMentions(res, "UNAVAILABLE"));
}

// ============================================================================
// last_state absent — short-circuit
// ============================================================================

TEST(PreSendValidatorTest, AbsentLastStateRejectedAndShortCircuits)
{
  auto ctx = make_ready_context();
  ctx.last_state.reset();
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  // Only the "no State reported" error should fire — mode and position
  // checks are unreachable without last_state.
  EXPECT_EQ(res.errors.size(), 1u);
  EXPECT_TRUE(AnyErrorMentions(res, "State"));
}

// ============================================================================
// Operating mode (AUTOMATIC and SEMIAUTOMATIC are master-controlled)
// ============================================================================

TEST(PreSendValidatorTest, SemiAutomaticModeAccepted)
{
  auto ctx = make_ready_context();
  ctx.last_state->operating_mode =
    vda5050_core::types::OperatingMode::SEMIAUTOMATIC;
  auto res = validate_pre_send(ctx);
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_TRUE(res.errors.empty());
}

TEST(PreSendValidatorTest, ManualModeRejected)
{
  auto ctx = make_ready_context();
  ctx.last_state->operating_mode = vda5050_core::types::OperatingMode::MANUAL;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "operating_mode"));
}

TEST(PreSendValidatorTest, ServiceModeRejected)
{
  auto ctx = make_ready_context();
  ctx.last_state->operating_mode = vda5050_core::types::OperatingMode::SERVICE;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
}

TEST(PreSendValidatorTest, TeachInModeRejected)
{
  // TEACHIN: AGV being taught (mapping / programming). Per VDA5050, the
  // master is not in control during TEACHIN.
  auto ctx = make_ready_context();
  ctx.last_state->operating_mode = vda5050_core::types::OperatingMode::TEACHIN;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
}

// ============================================================================
// Paused / e-stop
// ============================================================================

TEST(PreSendValidatorTest, PausedAgvRejected)
{
  auto ctx = make_ready_context();
  ctx.last_state->paused = true;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "paused"));
}

TEST(PreSendValidatorTest, EStopEngagedRejected)
{
  auto ctx = make_ready_context();
  ctx.last_state->safety_state.e_stop = vda5050_core::types::EStop::MANUAL;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "e-stop"));
}

// ============================================================================
// Position
// ============================================================================

TEST(PreSendValidatorTest, AbsentAgvPositionRejected)
{
  auto ctx = make_ready_context();
  ctx.last_state->agv_position.reset();
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "position"));
}

TEST(PreSendValidatorTest, UninitializedPositionRejected)
{
  auto ctx = make_ready_context();
  ctx.last_state->agv_position->position_initialized = false;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "position"));
}

// ============================================================================
// Error accumulation — non-fatal preconditions report ALL pending issues
// ============================================================================

TEST(PreSendValidatorTest, MultipleErrorsAccumulate)
{
  auto ctx = make_ready_context();
  ctx.connection_status = vda5050_core::types::ConnectionState::OFFLINE;
  ctx.last_state->operating_mode = vda5050_core::types::OperatingMode::MANUAL;
  ctx.last_state->agv_position->position_initialized = false;
  auto res = validate_pre_send(ctx);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_GE(res.errors.size(), 3u);
  EXPECT_TRUE(AllErrorsHavePreSendType(res));
  EXPECT_TRUE(AnyErrorMentions(res, "connection_status"));
  EXPECT_TRUE(AnyErrorMentions(res, "operating_mode"));
  EXPECT_TRUE(AnyErrorMentions(res, "position"));
}

}  // namespace vda5050_core::master::test
