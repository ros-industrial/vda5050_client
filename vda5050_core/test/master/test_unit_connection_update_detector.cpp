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

#include "vda5050_core/master/connection/connection_update_detector.hpp"

namespace vda5050_core::master::test {

namespace {
vda5050_core::types::Connection make_msg(vda5050_core::types::ConnectionState s)
{
  vda5050_core::types::Connection c;
  c.connection_state = s;
  return c;
}
}  // namespace

// ============================================================================
// First message (no prev) — every state is a transition.
// ============================================================================

TEST(ConnectionUpdateDetectorTest, ConnectedFromAbsentPrev)
{
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(std::nullopt, curr),
    ConnectionTransition::CONNECTED);
}

TEST(ConnectionUpdateDetectorTest, OfflineFromAbsentPrev)
{
  auto curr = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(
    detect_connection_transition(std::nullopt, curr),
    ConnectionTransition::OFFLINE);
}

TEST(ConnectionUpdateDetectorTest, ConnectionBrokenFromAbsentPrev)
{
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(std::nullopt, curr),
    ConnectionTransition::CONNECTIONBROKEN);
}

// ============================================================================
// Real transitions
// ============================================================================

TEST(ConnectionUpdateDetectorTest, OfflineFromOnline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionTransition::OFFLINE);
}

TEST(ConnectionUpdateDetectorTest, ConnectionBrokenFromOnline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(prev, curr),
    ConnectionTransition::CONNECTIONBROKEN);
}

TEST(ConnectionUpdateDetectorTest, ConnectedFromOffline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionTransition::CONNECTED);
}

TEST(ConnectionUpdateDetectorTest, ConnectedFromConnectionBroken)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionTransition::CONNECTED);
}

TEST(ConnectionUpdateDetectorTest, ConnectionBrokenFromOffline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(prev, curr),
    ConnectionTransition::CONNECTIONBROKEN);
}

// ============================================================================
// Sustained states — no event fires
// ============================================================================

TEST(ConnectionUpdateDetectorTest, NoneOnSustainedOnline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionTransition::NONE);
}

TEST(ConnectionUpdateDetectorTest, NoneOnSustainedOffline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionTransition::NONE);
}

TEST(ConnectionUpdateDetectorTest, NoneOnSustainedConnectionBroken)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionTransition::NONE);
}

}  // namespace vda5050_core::master::test
