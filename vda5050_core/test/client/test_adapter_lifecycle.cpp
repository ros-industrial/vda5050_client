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

#include <chrono>
#include <thread>

#include "vda5050_core/types/state.hpp"

#include "adapter_test_fixture.hpp"

using namespace vda5050_core::client::adapter;  // NOLINT
using namespace vda5050_core::types;            // NOLINT
using namespace testing;                        // NOLINT

class AdapterLifecycleTest : public AdapterTest
{};

TEST_F(AdapterLifecycleTest, StateManagerIsCreated)
{
  EXPECT_NE(adapter->state_manager(), nullptr);
}

TEST_F(AdapterLifecycleTest, StartConnectsBroker)
{
  EXPECT_CALL(*mqtt, connect()).Times(1);

  adapter->start();

  adapter->stop();
}

TEST_F(AdapterLifecycleTest, StopDisconnectsBroker)
{
  std::atomic_bool disconnect_called = false;

  EXPECT_CALL(*mqtt, connect()).Times(1);
  EXPECT_CALL(*mqtt, disconnect()).Times(1).WillOnce([&] {
    disconnect_called = true;
  });

  adapter->start();
  adapter->stop();

  ASSERT_TRUE(wait_until([&] { return disconnect_called.load(); }));
}

TEST_F(AdapterLifecycleTest, PublishesOnlineOfflineConnection)
{
  InSequence seq;

  EXPECT_CALL(*mqtt, publish(_, _, _, _))
    .WillOnce([](
                const std::string& /*topic*/, const std::string& message,
                int /*qos*/, bool /*retain*/) {
      auto connection = nlohmann::json::parse(message).get<Connection>();

      EXPECT_EQ(connection.connection_state, ConnectionState::ONLINE);
    });

  EXPECT_CALL(*mqtt, publish(_, _, _, _))
    .WillOnce([](
                const std::string& /*topic*/, const std::string& message,
                int /*qos*/, bool /*retain*/) {
      auto connection = nlohmann::json::parse(message).get<Connection>();

      EXPECT_EQ(connection.connection_state, ConnectionState::OFFLINE);
    });

  adapter->start();
  adapter->stop();
}

TEST_F(AdapterLifecycleTest, SetsConnectionWill)
{
  EXPECT_CALL(*mqtt, set_will(_, _, 1, true))
    .WillOnce([](
                const std::string& /*topic*/, const std::string& message,
                int /*qos*/, bool /*retain*/) {
      auto connection = nlohmann::json::parse(message).get<Connection>();

      EXPECT_EQ(connection.connection_state, ConnectionState::CONNECTIONBROKEN);
    });

  adapter->start();
  adapter->stop();
}

TEST_F(AdapterLifecycleTest, SubscribesTopics)
{
  EXPECT_CALL(*mqtt, subscribe(_, _, _)).Times(2);

  adapter->start();
  adapter->stop();
}

TEST_F(AdapterLifecycleTest, UnsubscribesTopics)
{
  EXPECT_CALL(*mqtt, unsubscribe(_)).Times(2);

  adapter->start();
  adapter->stop();
}

TEST_F(AdapterLifecycleTest, StartTwiceDoesNothing)
{
  EXPECT_CALL(*mqtt, connect()).Times(1);

  adapter->start();
  adapter->start();

  adapter->stop();
}

TEST_F(AdapterLifecycleTest, StopBeforeStartDoesNothing)
{
  EXPECT_CALL(*mqtt, disconnect()).Times(0);

  adapter->stop();
}

TEST_F(AdapterLifecycleTest, PublishThreadPublishesState)
{
  adapter->start();

  adapter->state_manager()->set_driving(true);

  ASSERT_TRUE(wait_publish(2));

  State state;
  {
    std::lock_guard<std::mutex> lock(publish_mutex);
    state = nlohmann::json::parse(published.back().message).get<State>();
  }

  EXPECT_TRUE(state.driving);

  adapter->stop();
}
