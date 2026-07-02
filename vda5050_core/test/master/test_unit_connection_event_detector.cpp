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

#include "vda5050_core/master/connection/connection_event_detector.hpp"

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

TEST(ConnectionEventDetectorTest, ConnectedFromAbsentPrev)
{
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(std::nullopt, curr),
    ConnectionEventKind::CONNECTED);
}

TEST(ConnectionEventDetectorTest, OfflineFromAbsentPrev)
{
  auto curr = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(
    detect_connection_transition(std::nullopt, curr),
    ConnectionEventKind::OFFLINE);
}

TEST(ConnectionEventDetectorTest, ConnectionBrokenFromAbsentPrev)
{
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(std::nullopt, curr),
    ConnectionEventKind::CONNECTIONBROKEN);
}

// ============================================================================
// Real transitions
// ============================================================================

TEST(ConnectionEventDetectorTest, OfflineFromOnline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionEventKind::OFFLINE);
}

TEST(ConnectionEventDetectorTest, ConnectionBrokenFromOnline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(prev, curr),
    ConnectionEventKind::CONNECTIONBROKEN);
}

TEST(ConnectionEventDetectorTest, ConnectedFromOffline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionEventKind::CONNECTED);
}

TEST(ConnectionEventDetectorTest, ConnectedFromConnectionBroken)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionEventKind::CONNECTED);
}

TEST(ConnectionEventDetectorTest, ConnectionBrokenFromOffline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(prev, curr),
    ConnectionEventKind::CONNECTIONBROKEN);
}

// ============================================================================
// Sustained states — no event fires
// ============================================================================

TEST(ConnectionEventDetectorTest, NoneOnSustainedOnline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::ONLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionEventKind::NONE);
}

TEST(ConnectionEventDetectorTest, NoneOnSustainedOffline)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  auto curr = make_msg(vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionEventKind::NONE);
}

TEST(ConnectionEventDetectorTest, NoneOnSustainedConnectionBroken)
{
  auto prev = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  auto curr = make_msg(vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_EQ(
    detect_connection_transition(prev, curr), ConnectionEventKind::NONE);
}

}  // namespace vda5050_core::master::test
