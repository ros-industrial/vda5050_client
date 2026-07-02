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
#include "vda5050_core/master/validation/action_conflict_validator.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master {
namespace test {

namespace {

vda5050_core::types::Action make_action(
  const std::string& type, vda5050_core::types::BlockingType blocking,
  const std::string& id = "act-1")
{
  vda5050_core::types::Action a;
  a.action_type = type;
  a.action_id = id;
  a.blocking_type = blocking;
  return a;
}

vda5050_core::types::ActionState make_action_state(
  const std::string& id, vda5050_core::types::ActionStatus status)
{
  vda5050_core::types::ActionState s;
  s.action_id = id;
  s.action_status = status;
  return s;
}

vda5050_core::types::State make_state(
  bool driving,
  std::vector<vda5050_core::types::ActionState> action_states = {})
{
  vda5050_core::types::State s;
  s.driving = driving;
  s.action_states = std::move(action_states);
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
// Defensive / no-state cases
// =============================================================================

TEST(ActionConflictValidator, EmptyActionsList_Passes)
{
  PreSendContext ctx{
    vda5050_core::types::ConnectionState::ONLINE, make_state(true),
    std::nullopt, AGVState::AVAILABLE, nullptr};
  auto res = validate_action_conflict(ctx, wrap({}));
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_TRUE(res.errors.empty());
}

TEST(ActionConflictValidator, NoStateInContext_Passes)
{
  // Degraded mode: AGV never reported state. No info on driving or
  // action_states[]; conservative = pass through (matches PreSend pattern).
  PreSendContext ctx{
    vda5050_core::types::ConnectionState::ONLINE, std::nullopt, std::nullopt,
    AGVState::AVAILABLE, nullptr};
  auto res = validate_action_conflict(
    ctx,
    wrap({make_action("anything", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_TRUE(static_cast<bool>(res));
}

// =============================================================================
// NONE blocking — always allowed
// =============================================================================

TEST(ActionConflictValidator, NoneBlockingAction_AlwaysAllowed)
{
  // Even when AGV is driving AND has active actions, NONE is OK.
  auto state = make_state(
    true, {make_action_state(
            "inflight", vda5050_core::types::ActionStatus::RUNNING)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap(
      {make_action("stateRequest", vda5050_core::types::BlockingType::NONE)}));
  EXPECT_TRUE(static_cast<bool>(res));
}

// =============================================================================
// SOFT blocking
// =============================================================================

TEST(ActionConflictValidator, SoftBlockingNotDriving_Allowed)
{
  auto state = make_state(false);
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap(
      {make_action("doSomething", vda5050_core::types::BlockingType::SOFT)}));
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(ActionConflictValidator, SoftBlockingDriving_RejectedWith_BlockedByDriving)
{
  auto state = make_state(true);
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap(
      {make_action("doSomething", vda5050_core::types::BlockingType::SOFT)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::ActionBlockedByDrivingError);
}

// =============================================================================
// HARD blocking
// =============================================================================

TEST(ActionConflictValidator, HardBlockingNoActiveActions_NotDriving_Allowed)
{
  auto state = make_state(false);
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(
  ActionConflictValidator,
  HardBlockingActiveRunningAction_RejectedWith_HardBlocked)
{
  auto state = make_state(
    false, {make_action_state(
             "inflight", vda5050_core::types::ActionStatus::RUNNING)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::HardActionBlockedError);
}

TEST(
  ActionConflictValidator,
  HardBlockingActiveWaitingAction_RejectedWith_HardBlocked)
{
  auto state = make_state(
    false,
    {make_action_state("queued", vda5050_core::types::ActionStatus::WAITING)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::HardActionBlockedError);
}

TEST(
  ActionConflictValidator,
  HardBlockingActivePausedAction_RejectedWith_HardBlocked)
{
  // PAUSED is conservatively considered active — a paused action could
  // resume any time and conflict with the candidate HARD.
  auto state = make_state(
    false,
    {make_action_state("paused", vda5050_core::types::ActionStatus::PAUSED)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::HardActionBlockedError);
}

TEST(
  ActionConflictValidator,
  HardBlockingActiveInitializingAction_RejectedWith_HardBlocked)
{
  // INITIALIZING is one of the four active statuses (alongside WAITING,
  // RUNNING, PAUSED). Without this test, that branch of is_active_status
  // would be uncovered.
  auto state = make_state(
    false, {make_action_state(
             "init", vda5050_core::types::ActionStatus::INITIALIZING)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::HardActionBlockedError);
}

TEST(
  ActionConflictValidator,
  HardBlockingMixOfFinishedAndRunning_RejectedByRunning)
{
  // Mix of active + non-active actions in state: the iteration must find
  // the active one and reject. Order in the array shouldn't matter — try
  // FINISHED first, then RUNNING.
  auto state = make_state(
    false,
    {make_action_state("done", vda5050_core::types::ActionStatus::FINISHED),
     make_action_state(
       "inflight", vda5050_core::types::ActionStatus::RUNNING)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::HardActionBlockedError);
}

TEST(ActionConflictValidator, HardBlockingFinishedActionsOnly_Allowed)
{
  // FINISHED / FAILED actions are non-active — should NOT block HARD.
  auto state = make_state(
    false,
    {make_action_state("done1", vda5050_core::types::ActionStatus::FINISHED),
     make_action_state("done2", vda5050_core::types::ActionStatus::FAILED)});
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST(ActionConflictValidator, HardBlockingDriving_RejectedWith_BlockedByDriving)
{
  // The driving check fires before the active-action check (HARD must
  // not drive). Even with no active actions, driving alone rejects HARD.
  auto state = make_state(true);
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_EQ(
    res.errors.front().error_type,
    vda5050_core::errors::ActionBlockedByDrivingError);
}

// =============================================================================
// Batch semantics + tagging
// =============================================================================

TEST(
  ActionConflictValidator, MultipleActionsBatch_FirstConflict_StopsAndReturns)
{
  // Order in batch: [NONE OK, HARD blocked]. Validator should reach the
  // HARD action and return — single error, focused on the first conflict.
  auto state = make_state(
    false, {make_action_state(
             "inflight", vda5050_core::types::ActionStatus::RUNNING)});
  std::vector<vda5050_core::types::Action> batch = {
    make_action("stateRequest", vda5050_core::types::BlockingType::NONE, "ok"),
    make_action("hardAction", vda5050_core::types::BlockingType::HARD, "bad")};
  auto res = validate_action_conflict(make_ctx(state), wrap(batch));
  EXPECT_FALSE(static_cast<bool>(res));
  ASSERT_EQ(res.errors.size(), 1u);
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  // The error should reference the second action's id ("bad"), confirming
  // we made it past the first NONE-action without rejecting it.
  EXPECT_NE(
    res.errors.front().error_description->find("hardAction"),
    std::string::npos);
}

TEST(ActionConflictValidator, DrivingRejection_TaggedAsBlockedByDriving)
{
  auto state = make_state(true);
  auto res = validate_action_conflict(
    make_ctx(state),
    wrap({make_action("hardAction", vda5050_core::types::BlockingType::HARD)}));
  ASSERT_FALSE(res.errors.empty());
  for (const auto& err : res.errors)
  {
    EXPECT_EQ(
      err.error_type, vda5050_core::errors::ActionBlockedByDrivingError);
  }
}

}  // namespace test
}  // namespace vda5050_core::master
