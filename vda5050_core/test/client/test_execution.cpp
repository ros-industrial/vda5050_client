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
}

TEST(OrderExecutionTest, FinishInvokesCallback) {}

TEST(OrderExecutionTest, FailedInvokesCallback) {}

TEST(OrderExecutionTest, CannotFinishTwice) {}

TEST(ActionExecutionTest, StartsActive) {}

TEST(ActionExecutionTest, InitializingShowsInitializingStatus) {}

TEST(ActionExecutionTest, RunningShowsRunningStatus) {}

TEST(ActionExecutionTest, PausedWithDescription) {}

TEST(ActionExecutionTest, PausedWithoutDescription) {}

TEST(ActionExecutionTest, FinishedWithDescription) {}

TEST(ActionExecutionTest, FinishedWithoutDescription) {}

TEST(ActionExecutionTest, FailedInvokesCallback) {}
