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

/**
 * @file master_broker_status_test.cpp
 * @brief Tests for VDA5050Master::on_broker_disconnected /
 *        on_broker_reconnected virtuals + get_broker_status() snapshot
 *        (Task #70).
 *
 * Uses a fake MqttClientInterface that captures the connection-state
 * handlers registered by VDA5050Master in connect() and exposes a way
 * to fire them on demand. No broker required.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

namespace vda5050_core::master::test {

namespace {

// Minimal fake that records the connection-state callbacks the master
// registers in its ctor. Other interface methods are no-ops (or
// reasonable defaults) — we never publish or subscribe in these tests.
class FakeMqttClient : public vda5050_core::transport::MqttClientInterface
{
public:
  void connect() override {}
  void disconnect() override {}
  bool connected() override
  {
    return false;
  }
  void publish(
    const std::string& /*topic*/, const std::string& /*message*/, int /*qos*/,
    bool /*retain*/) override
  {
  }
  void subscribe(
    const std::string& /*topic*/, MessageHandler /*handler*/,
    int /*qos*/) override
  {
  }
  void unsubscribe(const std::string& /*topic*/) override {}
  void set_will(
    const std::string& /*topic*/, const std::string& /*message*/, int /*qos*/,
    bool /*retain*/ = true) override
  {
  }

  void set_connection_lost_callback(ConnectionStateHandler handler) override
  {
    connection_lost_handler_ = std::move(handler);
  }
  void set_connected_callback(ConnectionStateHandler handler) override
  {
    connected_handler_ = std::move(handler);
  }

  // Test hooks: fire what the underlying transport would fire.
  void fire_connected(const std::string& cause = "test")
  {
    if (connected_handler_) connected_handler_(cause);
  }
  void fire_connection_lost(const std::string& cause = "test")
  {
    if (connection_lost_handler_) connection_lost_handler_(cause);
  }

  bool has_connection_lost_handler() const
  {
    return static_cast<bool>(connection_lost_handler_);
  }
  bool has_connected_handler() const
  {
    return static_cast<bool>(connected_handler_);
  }

private:
  ConnectionStateHandler connection_lost_handler_;
  ConnectionStateHandler connected_handler_;
};

class CallbackTrackingMaster : public VDA5050Master
{
public:
  using VDA5050Master::VDA5050Master;

  void on_broker_disconnected() override
  {
    disconnected_calls.fetch_add(1);
  }

  void on_broker_reconnected() override
  {
    reconnected_calls.fetch_add(1);
  }

  std::atomic<int> disconnected_calls{0};
  std::atomic<int> reconnected_calls{0};
};

class BrokerStatusTest : public ::testing::Test
{
protected:
  std::shared_ptr<FakeMqttClient> fake_;
  std::shared_ptr<CallbackTrackingMaster> master_;

  void SetUp() override
  {
    fake_ = std::make_shared<FakeMqttClient>();
    master_ = std::make_shared<CallbackTrackingMaster>(fake_);
    // Broker callbacks register in connect() (weak_from_this() is not valid
    // during construction).
    master_->connect();
  }

  void TearDown() override
  {
    master_.reset();
    fake_.reset();
  }
};

}  // namespace

TEST_F(BrokerStatusTest, MasterRegistersCallbacksOnConnect)
{
  // connect() ran in SetUp.
  EXPECT_TRUE(fake_->has_connection_lost_handler());
  EXPECT_TRUE(fake_->has_connected_handler());
}

TEST_F(BrokerStatusTest, InitialSnapshotShowsDisconnected)
{
  const auto snap = master_->get_broker_status();
  EXPECT_FALSE(snap.connected);
  EXPECT_EQ(snap.reconnect_count, 0u);
  EXPECT_FALSE(snap.last_disconnect_at.has_value());
}

TEST_F(BrokerStatusTest, ConnectedEventDispatchesVirtualAndIncrementsCount)
{
  fake_->fire_connected("initial");
  EXPECT_EQ(master_->reconnected_calls.load(), 1);
  EXPECT_EQ(master_->disconnected_calls.load(), 0);

  const auto snap = master_->get_broker_status();
  EXPECT_TRUE(snap.connected);
  EXPECT_EQ(snap.reconnect_count, 1u);
}

TEST_F(BrokerStatusTest, DisconnectThenReconnectCountsTwoConnects)
{
  fake_->fire_connected("initial");
  fake_->fire_connection_lost("network drop");
  fake_->fire_connected("recover");

  EXPECT_EQ(master_->reconnected_calls.load(), 2);
  EXPECT_EQ(master_->disconnected_calls.load(), 1);

  const auto snap = master_->get_broker_status();
  EXPECT_TRUE(snap.connected);
  EXPECT_EQ(snap.reconnect_count, 2u);
  ASSERT_TRUE(snap.last_disconnect_at.has_value());
}

TEST_F(BrokerStatusTest, DisconnectStampsTimestampAndFlipsConnected)
{
  fake_->fire_connected();

  const auto before = std::chrono::system_clock::now();
  fake_->fire_connection_lost("server gone");
  const auto after = std::chrono::system_clock::now();

  const auto snap = master_->get_broker_status();
  EXPECT_FALSE(snap.connected);
  ASSERT_TRUE(snap.last_disconnect_at.has_value());
  EXPECT_GE(*snap.last_disconnect_at, before);
  EXPECT_LE(*snap.last_disconnect_at, after);
}

TEST_F(BrokerStatusTest, DestructorClearsHandlersOnUnderlyingClient)
{
  // Sanity: handlers are installed (connect() ran in SetUp).
  EXPECT_TRUE(fake_->has_connection_lost_handler());
  EXPECT_TRUE(fake_->has_connected_handler());

  master_.reset();

  // After master destruction the registered handlers should be cleared
  // — destruction order would otherwise allow the transport thread to
  // invoke a dangling `this` pointer.
  EXPECT_FALSE(fake_->has_connection_lost_handler());
  EXPECT_FALSE(fake_->has_connected_handler());
}

}  // namespace vda5050_core::master::test
