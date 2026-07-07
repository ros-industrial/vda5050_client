/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
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

#include <atomic>
#include <thread>

#include "test_fixture_agv.hpp"

namespace vda5050_core::master::test {

// Use the shared AGV test fixture
using AGVConnectionStateTestFixture = AGVTestFixture;

// =============================================================================
// Initial State Tests
// =============================================================================

TEST_F(AGVConnectionStateTestFixture, InitialConnectionStateIsOffline)
{
  auto& agv = create_agv();
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
}

// =============================================================================
// Connection Message Tests
// =============================================================================

TEST_F(
  AGVConnectionStateTestFixture,
  ConnectionStateOnlineAfterReceivingOnlineMessage)
{
  auto& agv = create_agv();

  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);

  agv->handle_connection(create_connection_msg("ONLINE"));

  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);
}

TEST_F(
  AGVConnectionStateTestFixture,
  ConnectionStateOfflineAfterReceivingOfflineMessage)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);
}

TEST_F(
  AGVConnectionStateTestFixture,
  ConnectionStateConnectionBrokenAfterReceivingConnectionBrokenMessage)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
}

// =============================================================================
// State Transition Tests
// =============================================================================

TEST_F(AGVConnectionStateTestFixture, TransitionOnlineToOfflineToOnline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::OFFLINE);

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);
}

TEST_F(
  AGVConnectionStateTestFixture, TransitionOnlineToConnectionBrokenToOnline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(
    agv->get_connection_status(),
    vda5050_core::types::ConnectionState::CONNECTIONBROKEN);

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(
    agv->get_connection_status(), vda5050_core::types::ConnectionState::ONLINE);
}

// =============================================================================
// Cached Message Tests
// =============================================================================

TEST_F(AGVConnectionStateTestFixture, CachedConnectionMessageIsStored)
{
  auto& agv = create_agv();

  // Initially no cached message
  EXPECT_FALSE(agv->get_last_connection().has_value());

  // Handle connection message
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Verify cached message
  auto cached = agv->get_last_connection();
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(
    cached->connection_state, vda5050_core::types::ConnectionState::ONLINE);

  // Verify timestamp was recorded
  EXPECT_TRUE(agv->get_last_connection_time().has_value());
}

// =============================================================================
// is_connected() Tests
// =============================================================================

TEST_F(AGVConnectionStateTestFixture, IsConnectedReturnsTrueWhenOnline)
{
  auto& agv = create_agv();

  EXPECT_FALSE(agv->is_connected());

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_TRUE(agv->is_connected());
}

TEST_F(AGVConnectionStateTestFixture, IsConnectedReturnsFalseWhenOffline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_TRUE(agv->is_connected());

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_FALSE(agv->is_connected());
}

TEST_F(
  AGVConnectionStateTestFixture, IsConnectedReturnsFalseWhenConnectionBroken)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_TRUE(agv->is_connected());

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_FALSE(agv->is_connected());
}

// =============================================================================
// Thread Safety Tests
// =============================================================================

TEST_F(AGVConnectionStateTestFixture, ConcurrentConnectionStateAccessIsSafe)
{
  auto& agv = create_agv();
  std::atomic_bool stop{false};

  std::thread reader([&]() {
    while (!stop.load())
    {
      auto conn_status = agv->get_connection_status();
      auto is_conn = agv->is_connected();
      (void)conn_status;
      (void)is_conn;
    }
  });

  for (int i = 0; i < 100; ++i)
  {
    agv->handle_connection(create_connection_msg("ONLINE"));
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    agv->handle_connection(create_connection_msg("OFFLINE"));
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  stop.store(true);
  reader.join();

  SUCCEED();
}

// =============================================================================
// Enum Value Tests
// =============================================================================

TEST_F(AGVConnectionStateTestFixture, AGVConnectionStateEnumValues)
{
  EXPECT_NE(
    vda5050_core::types::ConnectionState::ONLINE,
    vda5050_core::types::ConnectionState::OFFLINE);
  EXPECT_NE(
    vda5050_core::types::ConnectionState::ONLINE,
    vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
  EXPECT_NE(
    vda5050_core::types::ConnectionState::OFFLINE,
    vda5050_core::types::ConnectionState::CONNECTIONBROKEN);
}

// =============================================================================
// Configurable interface name
// =============================================================================

// The per-AGV interface name threads through the constructor and is reported
// back via get_interface_name(); onboarding without a name uses the default.
TEST(AGVInterfaceName, ReportsConfiguredInterfaceName)
{
  AGV custom(nullptr, "vendorX", "ACME", "SN-IFACE");
  EXPECT_EQ(custom.get_interface_name(), "vendorX");

  AGV defaulted(nullptr, DefaultInterfaceName, "ACME", "SN-IFACE2");
  EXPECT_EQ(defaulted.get_interface_name(), "uagv");
}

}  // namespace vda5050_core::master::test
