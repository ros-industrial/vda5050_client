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

#include <atomic>

#include "vda5050_core/client/adapter/action_execution.hpp"
#include "vda5050_core/client/adapter/order_execution.hpp"

using namespace vda5050_core::client::adapter;  // NOLINT

TEST(OrderExecutionTest, StartsActive)
{
  std::atomic_bool finished = false;
  std::string reason;

  auto execution = OrderExecution::make(
    "order_1", 1, [&]() { finished = true; },
    [&](std::string r) { reason = std::move(r); });

  EXPECT_TRUE(execution->okay());
  EXPECT_FALSE(execution->is_finished());
  EXPECT_FALSE(execution->failure_reason().has_value());

  EXPECT_FALSE(finished);
  EXPECT_TRUE(reason.empty());

  EXPECT_EQ(execution->order_id(), "order_1");
  EXPECT_EQ(execution->order_update_id(), 1);
}

TEST(OrderExecutionTest, FinishInvokesCallback)
{
  std::atomic_int finish_count = 0;
  std::string reason;

  auto execution = OrderExecution::make(
    "order_1", 1, [&]() { finish_count++; },
    [&](std::string r) { reason = std::move(r); });

  execution->finished();

  EXPECT_EQ(finish_count, 1);
  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());
  EXPECT_FALSE(execution->failure_reason().has_value());

  EXPECT_TRUE(reason.empty());
}

TEST(OrderExecutionTest, FailedInvokesCallback)
{
  std::atomic_bool finished = false;
  std::string reason;

  auto execution = OrderExecution::make(
    "order_1", 1, [&]() { finished = true; },
    [&](std::string r) { reason = std::move(r); });

  execution->failed("navigation failed");

  EXPECT_FALSE(finished);

  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());

  ASSERT_TRUE(execution->failure_reason().has_value());
  EXPECT_EQ(*execution->failure_reason(), "navigation failed");

  EXPECT_EQ(reason, "navigation failed");
}

TEST(OrderExecutionTest, CannotFinishTwice)
{
  std::atomic_int finish_count = 0;
  std::atomic_int fail_count = 0;

  auto execution = OrderExecution::make(
    "order_1", 1, [&]() { finish_count++; },
    [&](std::string) { fail_count++; });

  execution->finished();
  EXPECT_NO_THROW(execution->finished());
  EXPECT_NO_THROW(execution->failed("error"));

  EXPECT_EQ(finish_count, 1);
  EXPECT_EQ(fail_count, 0);

  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());
}

TEST(OrderExecutionTest, CannotFailTwice)
{
  std::atomic_int finish_count = 0;
  std::atomic_int fail_count = 0;
  std::string reason;

  auto execution = OrderExecution::make(
    "order_1", 1, [&]() { finish_count++; },
    [&](std::string r) {
      reason = std::move(r);
      fail_count++;
    });

  execution->failed("navigation failed");
  EXPECT_NO_THROW(execution->failed("unknown error"));
  EXPECT_NO_THROW(execution->finished());

  EXPECT_EQ(fail_count, 1);
  EXPECT_EQ(finish_count, 0);

  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());

  ASSERT_TRUE(execution->failure_reason().has_value());
  EXPECT_EQ(*execution->failure_reason(), "navigation failed");

  EXPECT_EQ(reason, "navigation failed");
}

TEST(ActionExecutionTest, StartsActive)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  EXPECT_TRUE(execution->okay());
  EXPECT_FALSE(execution->is_finished());
  EXPECT_TRUE(updates.empty());
}

TEST(ActionExecutionTest, InitializingShowsInitializingStatus)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->initializing();

  ASSERT_EQ(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::INITIALIZING);

  EXPECT_FALSE(updates[0].second.has_value());

  EXPECT_TRUE(execution->okay());
  EXPECT_FALSE(execution->is_finished());
}

TEST(ActionExecutionTest, RunningShowsRunningStatus)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->running();

  ASSERT_EQ(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::RUNNING);

  EXPECT_FALSE(updates[0].second.has_value());

  EXPECT_TRUE(execution->okay());
  EXPECT_FALSE(execution->is_finished());
}

TEST(ActionExecutionTest, PausedWithDescription)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->paused("Waiting for operator");

  ASSERT_EQ(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::PAUSED);

  ASSERT_TRUE(updates[0].second.has_value());
  EXPECT_EQ(*updates[0].second, "Waiting for operator");

  EXPECT_TRUE(execution->okay());
  EXPECT_FALSE(execution->is_finished());
}

TEST(ActionExecutionTest, PausedWithoutDescription)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->paused();

  ASSERT_THAT(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::PAUSED);

  EXPECT_FALSE(updates[0].second.has_value());

  EXPECT_TRUE(execution->okay());
  EXPECT_FALSE(execution->is_finished());
}

TEST(ActionExecutionTest, FinishedWithDescription)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->finished("Completed");

  ASSERT_THAT(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::FINISHED);

  ASSERT_TRUE(updates[0].second.has_value());
  EXPECT_EQ(*updates[0].second, "Completed");

  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());
}

TEST(ActionExecutionTest, FinishedWithoutDescription)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->finished();

  ASSERT_THAT(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::FINISHED);

  EXPECT_FALSE(updates[0].second.has_value());

  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());
}

TEST(ActionExecutionTest, FailedInvokesCallback)
{
  std::vector<
    std::pair<vda5050_core::types::ActionStatus, std::optional<std::string>>>
    updates;

  auto execution = ActionExecution::make([&](auto status, auto description) {
    updates.emplace_back(status, description);
  });

  execution->failed("Actuator error");

  ASSERT_THAT(updates.size(), 1);

  EXPECT_EQ(updates[0].first, vda5050_core::types::ActionStatus::FAILED);

  EXPECT_FALSE(execution->okay());
  EXPECT_TRUE(execution->is_finished());

  ASSERT_TRUE(execution->failure_reason().has_value());
  EXPECT_EQ(*execution->failure_reason(), "Actuator error");
}
