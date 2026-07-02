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

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "vda5050_core/master/agv.hpp"

using namespace vda5050_core::master;  // NOLINT

class AGVTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    interface_name_ = DefaultInterfaceName;
    manufacturer_ = "TestManufacturer";
    serial_number_ = "SN001";
    agv_id_ = manufacturer_ + "/" + serial_number_;
  }

  void TearDown() override
  {
    agv_.reset();
  }

  std::unique_ptr<AGV>& create_agv()
  {
    // Use nullptr for ProtocolAdapter in unit tests
    agv_ = std::make_unique<AGV>(
      nullptr, interface_name_, manufacturer_, serial_number_);
    return agv_;
  }

  std::unique_ptr<AGV>& create_agv_with_heartbeat_interval(
    int state_heartbeat_interval)
  {
    // Use nullptr for ProtocolAdapter in unit tests
    agv_ = std::make_unique<AGV>(
      nullptr, interface_name_, manufacturer_, serial_number_, 10, true,
      state_heartbeat_interval);
    return agv_;
  }

  Connection create_connection_msg(const std::string& state)
  {
    Connection msg;
    msg.header.header_id = 1;
    msg.header.timestamp = std::chrono::system_clock::now();
    msg.header.version = "2.0.0";
    msg.header.manufacturer = manufacturer_;
    msg.header.serial_number = serial_number_;

    if (state == "ONLINE")
    {
      msg.connection_state = ConnectionState::ONLINE;
    }
    else if (state == "OFFLINE")
    {
      msg.connection_state = ConnectionState::OFFLINE;
    }
    else
    {
      msg.connection_state = ConnectionState::CONNECTIONBROKEN;
    }

    return msg;
  }

  State create_state_msg()
  {
    State msg;
    msg.header.header_id = 1;
    msg.header.timestamp = std::chrono::system_clock::now();
    msg.header.version = "2.0.0";
    msg.header.manufacturer = manufacturer_;
    msg.header.serial_number = serial_number_;
    msg.order_id = "test_order";
    msg.order_update_id = 0;
    msg.driving = false;
    msg.paused = false;
    msg.new_base_request = false;
    msg.distance_since_last_node = 0.0;
    return msg;
  }

  Order create_test_order(const std::string& order_id)
  {
    Order order;
    order.order_id = order_id;
    order.order_update_id = 0;
    return order;
  }

  InstantActions create_test_instant_actions(uint32_t id)
  {
    InstantActions actions;
    actions.header.header_id = id;
    return actions;
  }

  std::unique_ptr<AGV> agv_;
  std::string interface_name_;
  std::string manufacturer_;
  std::string serial_number_;
  std::string agv_id_;
};

TEST_F(AGVTestFixture, InitialConnectionStateIsOffline)
{
  auto& agv = create_agv();
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
}

TEST_F(AGVTestFixture, ConnectionStateOnlineAfterReceivingOnlineMessage)
{
  auto& agv = create_agv();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);

  agv->handle_connection(create_connection_msg("ONLINE"));

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);
}

TEST_F(AGVTestFixture, ConnectionStateOfflineAfterReceivingOfflineMessage)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
}

TEST_F(
  AGVTestFixture,
  ConnectionStateConnectionBrokenAfterReceivingConnectionBrokenMessage)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::CONNECTIONBROKEN);
}

TEST_F(AGVTestFixture, TransitionOnlineToOfflineToOnline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);
}

TEST_F(AGVTestFixture, TransitionOnlineToConnectionBrokenToOnline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::CONNECTIONBROKEN);

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);
}

TEST_F(AGVTestFixture, CachedConnectionMessageIsStored)
{
  auto& agv = create_agv();

  // Initially no cached message
  EXPECT_FALSE(agv->get_last_connection().has_value());

  // Handle connection message
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Verify cached message
  auto cached = agv->get_last_connection();
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->connection_state, ConnectionState::ONLINE);

  // Verify timestamp was recorded
  EXPECT_TRUE(agv->get_last_connection_time().has_value());
}

TEST_F(AGVTestFixture, IsConnectedReturnsTrueWhenOnline)
{
  auto& agv = create_agv();

  EXPECT_FALSE(agv->is_connected());

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_TRUE(agv->is_connected());
}

TEST_F(AGVTestFixture, IsConnectedReturnsFalseWhenOffline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_TRUE(agv->is_connected());

  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_FALSE(agv->is_connected());
}

TEST_F(AGVTestFixture, IsConnectedReturnsFalseWhenConnectionBroken)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_TRUE(agv->is_connected());

  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_FALSE(agv->is_connected());
}

TEST_F(AGVTestFixture, ConcurrentConnectionStateAccessIsSafe)
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

TEST_F(AGVTestFixture, StopResetsConnectionStatusToOffline)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->stop();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
}

TEST_F(AGVTestFixture, StopResetsOperationalStateToUnknown)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->stop();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, StopPreservesCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->stop();

  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
  EXPECT_TRUE(agv->get_last_connection_time().has_value());
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

TEST_F(AGVTestFixture, StopIsIdempotent)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));

  agv->stop();
  agv->stop();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, StopOnFreshAgvIsNoOp)
{
  auto& agv = create_agv();

  agv->stop();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, RestartClearsAllCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  agv->handle_factsheet(Factsheet{});
  agv->handle_visualization(Visualization{});
  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
  EXPECT_TRUE(agv->get_last_factsheet().has_value());
  EXPECT_TRUE(agv->get_last_visualization().has_value());

  agv->restart();

  EXPECT_FALSE(agv->get_last_connection().has_value());
  EXPECT_FALSE(agv->get_last_state().has_value());
  EXPECT_FALSE(agv->get_last_factsheet().has_value());
  EXPECT_FALSE(agv->get_last_visualization().has_value());
  EXPECT_FALSE(agv->get_last_connection_time().has_value());
  EXPECT_FALSE(agv->get_last_state_time().has_value());
  EXPECT_FALSE(agv->get_last_factsheet_time().has_value());
  EXPECT_FALSE(agv->get_last_visualization_time().has_value());
}

TEST_F(AGVTestFixture, RestartResetsConnectionStatus)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->restart();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
}

TEST_F(AGVTestFixture, RestartResetsOperationalState)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->restart();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, RestartAllowsNewConnectionCycle)
{
  auto& agv = create_agv();

  // First lifecycle
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->restart();

  // Verify initial state after restart
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // Second lifecycle
  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVTestFixture, RestartPreservesIdentity)
{
  auto& agv = create_agv();

  agv->restart();

  EXPECT_EQ(agv->get_interface_name(), interface_name_);
  EXPECT_EQ(agv->get_manufacturer(), manufacturer_);
  EXPECT_EQ(agv->get_serial_number(), serial_number_);
  EXPECT_EQ(agv->get_agv_id(), agv_id_);
}

TEST_F(AGVTestFixture, PauseSetsStatesToOfflineAndUnavailable)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->pause();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(AGVTestFixture, PausePreservesCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();

  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
  EXPECT_TRUE(agv->get_last_connection_time().has_value());
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

TEST_F(AGVTestFixture, PauseIsIdempotent)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));

  agv->pause();
  agv->pause();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(AGVTestFixture, ResumeAfterPauseAllowsStateMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  agv->pause();
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  agv->resume();

  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVTestFixture, ResumePreservesCachedMessages)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();
  agv->resume();

  EXPECT_TRUE(agv->get_last_connection().has_value());
  EXPECT_TRUE(agv->get_last_state().has_value());
}

TEST_F(AGVTestFixture, ResumeOnFreshAgvIsNoOp)
{
  auto& agv = create_agv();

  agv->resume();

  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, MultiplePauseResumeCycles)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  for (int i = 0; i < 3; ++i)
  {
    agv->pause();
    EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
    EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);

    agv->resume();

    agv->handle_state(create_state_msg());
    EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
  }
}

TEST_F(AGVTestFixture, StopAfterPauseResetsOperationalState)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  agv->stop();
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, RestartAfterPauseClearsEverything)
{
  auto& agv = create_agv();

  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  agv->pause();

  agv->restart();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_FALSE(agv->get_last_connection().has_value());
  EXPECT_FALSE(agv->get_last_state().has_value());
}

TEST_F(AGVTestFixture, PausePreservesQueuesWhileStopClears)
{
  auto& agv = create_agv();

  // Queue messages while offline (processor not running)
  agv->send_order(create_test_order("order_1"));
  agv->send_order(create_test_order("order_2"));
  agv->send_instant_actions(create_test_instant_actions(1));

  EXPECT_EQ(agv->get_pending_order_count(), 2u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Pause should not clear the queues
  agv->pause();
  EXPECT_EQ(agv->get_pending_order_count(), 2u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Stop should clear the queues
  agv->stop();
  EXPECT_EQ(agv->get_pending_order_count(), 0u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 0u);
}

TEST_F(AGVTestFixture, ResumeAfterPauseCanProcessRemainingQueue)
{
  auto& agv = create_agv();

  // Queue messages while offline
  agv->send_order(create_test_order("order_1"));
  agv->send_instant_actions(create_test_instant_actions(1));

  EXPECT_EQ(agv->get_pending_order_count(), 1u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Pause (queues preserved)
  agv->pause();
  EXPECT_EQ(agv->get_pending_order_count(), 1u);
  EXPECT_EQ(agv->get_pending_instant_actions_count(), 1u);

  // Add more messages while paused
  agv->send_order(create_test_order("order_2"));
  EXPECT_EQ(agv->get_pending_order_count(), 2u);

  // Resume - queue processor starts and can process remaining items
  agv->resume();

  // After resume, we should still be able to add to queues
  agv->send_order(create_test_order("order_3"));
  EXPECT_GE(agv->get_pending_order_count(), 1u);
}

TEST_F(AGVTestFixture, InitialOperationalStateIsUnknown)
{
  auto& agv = create_agv();
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, OperationalStateAvailableAfterReceivingStateMessage)
{
  auto& agv = create_agv();

  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // First establish connection to enable state heartbeat
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Now receive state message
  agv->handle_state(create_state_msg());

  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVTestFixture, CachedStateMessageIsStored)
{
  auto& agv = create_agv();

  // Initially no cached state
  EXPECT_FALSE(agv->get_last_state().has_value());

  // Establish connection and receive state
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());

  // Verify cached state
  auto cached = agv->get_last_state();
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->order_id, "test_order");

  // Verify timestamp was recorded
  EXPECT_TRUE(agv->get_last_state_time().has_value());
}

TEST_F(AGVTestFixture, StateHeartbeatTimeoutTransitionsToStateUnknown)
{
  // Create AGV with short state heartbeat interval (1 second)
  auto& agv = create_agv_with_heartbeat_interval(1);

  // Establish connection to start heartbeat
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Receive state message to go AVAILABLE
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Wait for the state heartbeat timeout to trigger
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  // Verify state has transitioned to STATE_UNKNOWN due to timeout
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}

TEST_F(AGVTestFixture, StateHeartbeatReceivingMessagesPreventTimeout)
{
  // Create AGV with short state heartbeat interval (2 seconds)
  auto& agv = create_agv_with_heartbeat_interval(2);

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Receive initial state
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Send state messages periodically to keep operational state alive
  for (int i = 0; i < 3; ++i)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    agv->handle_state(create_state_msg());
    EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
  }

  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVTestFixture, TransitionAvailableToUnknownViaTimeoutThenRecover)
{
  // Create AGV with short state heartbeat interval (1 second)
  auto& agv = create_agv_with_heartbeat_interval(1);

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));

  // Initial state is UNKNOWN
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // Receive state message to go AVAILABLE
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Wait for timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);

  // Recover by receiving state message
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVTestFixture, ConnectionOfflineSetsOperationalStateToUnavailable)
{
  auto& agv = create_agv();

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Receive OFFLINE connection message
  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(AGVTestFixture, ConnectionBrokenSetsOperationalStateToUnavailable)
{
  auto& agv = create_agv();

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Receive CONNECTIONBROKEN connection message
  agv->handle_connection(create_connection_msg("CONNECTIONBROKEN"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);
}

TEST_F(AGVTestFixture, RecoverFromUnavailableAfterConnectionRestored)
{
  auto& agv = create_agv();

  // Establish connection
  agv->handle_connection(create_connection_msg("ONLINE"));
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);

  // Go OFFLINE
  agv->handle_connection(create_connection_msg("OFFLINE"));
  EXPECT_EQ(agv->get_operational_state(), AGVState::UNAVAILABLE);

  // Reconnect
  agv->handle_connection(create_connection_msg("ONLINE"));
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::ONLINE);

  // Operational state is still UNAVAILABLE until state message received
  // (because going OFFLINE changes operational state to UNAVAILABLE,
  // but going back ONLINE doesn't automatically restore AVAILABLE)

  // Receive state message to recover to AVAILABLE
  agv->handle_state(create_state_msg());
  EXPECT_EQ(agv->get_operational_state(), AGVState::AVAILABLE);
}

TEST_F(AGVTestFixture, AGVStateEnumValues)
{
  EXPECT_NE(AGVState::STATE_UNKNOWN, AGVState::AVAILABLE);
  EXPECT_NE(AGVState::STATE_UNKNOWN, AGVState::UNAVAILABLE);
  EXPECT_NE(AGVState::STATE_UNKNOWN, AGVState::ERROR);
  EXPECT_NE(AGVState::AVAILABLE, AGVState::UNAVAILABLE);
  EXPECT_NE(AGVState::AVAILABLE, AGVState::ERROR);
  EXPECT_NE(AGVState::UNAVAILABLE, AGVState::ERROR);
}

TEST_F(AGVTestFixture, ConcurrentOperationalStateAccessIsSafe)
{
  auto& agv = create_agv();
  std::atomic_bool stop{false};

  std::thread reader([&]() {
    while (!stop.load())
    {
      auto op_state = agv->get_operational_state();
      (void)op_state;
    }
  });

  for (int i = 0; i < 100; ++i)
  {
    agv->handle_connection(create_connection_msg("ONLINE"));
    agv->handle_state(create_state_msg());
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    agv->handle_connection(create_connection_msg("OFFLINE"));
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  stop.store(true);
  reader.join();

  SUCCEED();
}

TEST_F(AGVTestFixture, InitialStatesBeforeAnyMessages)
{
  auto& agv = create_agv();

  // Both start in their initial states
  EXPECT_EQ(agv->get_connection_status(), ConnectionState::OFFLINE);
  EXPECT_EQ(agv->get_operational_state(), AGVState::STATE_UNKNOWN);
}
