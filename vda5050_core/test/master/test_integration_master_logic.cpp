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

/**
 * @file master_logic_test.cpp
 * @brief Logic tests for VDA5050Master class
 *
 * These tests verify business logic of VDA5050Master including onboarding,
 * offboarding, and queue management. Tests use a real MQTT client but skip
 * if no broker is available.
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using vda5050_core::types::InstantActions;
using vda5050_core::types::Order;

using vda5050_core::master::DefaultInterfaceName;
using vda5050_core::master::VDA5050Master;

class MasterLogicTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    interface_name_ = DefaultInterfaceName;
    manufacturer_ = "TestManufacturer";
    serial_number_ = "SN001";
    agv_id_ = manufacturer_ + "/" + serial_number_;
  }

  std::shared_ptr<VDA5050Master> create_master()
  {
    std::string broker = "tcp://localhost:1883";
    auto client = vda5050_core::transport::create_default_client_shared(
      broker, "master_logic_test");
    return std::make_shared<VDA5050Master>(client);
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

  std::string interface_name_;
  std::string manufacturer_;
  std::string serial_number_;
  std::string agv_id_;
};

TEST_F(MasterLogicTestFixture, OnboardAGVCreatesInstance)
{
  auto master = create_master();
  master->connect();

  EXPECT_FALSE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->onboard_agv(manufacturer_, serial_number_);

  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, OnboardAGVCreatesInstanceCustomInterfaceName)
{
  std::string custom_interface_name = "amr";
  auto master = create_master();
  master->connect();

  EXPECT_FALSE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->onboard_agv(custom_interface_name, manufacturer_, serial_number_);

  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, OffboardAGVRemovesInstance)
{
  auto master = create_master();
  master->connect();

  master->onboard_agv(manufacturer_, serial_number_);
  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->offboard_agv(manufacturer_, serial_number_);
  EXPECT_FALSE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, PublishOrderToNotOnboardedAGVThrows)
{
  auto master = create_master();
  master->connect();

  EXPECT_THROW(
    master->publish_order(
      manufacturer_, serial_number_, create_test_order("1")),
    std::runtime_error);

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, PublishInstantActionsToNotOnboardedAGVThrows)
{
  auto master = create_master();
  master->connect();

  EXPECT_THROW(
    master->publish_instant_actions(
      manufacturer_, serial_number_, create_test_instant_actions(1)),
    std::runtime_error);

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, MultipleAGVsCanBeOnboarded)
{
  auto master = create_master();
  master->connect();

  master->onboard_agv("Manufacturer1", "SN001");
  master->onboard_agv("Manufacturer1", "SN002");
  master->onboard_agv("Manufacturer2", "SN001");

  EXPECT_TRUE(master->is_agv_onboarded("Manufacturer1", "SN001"));
  EXPECT_TRUE(master->is_agv_onboarded("Manufacturer1", "SN002"));
  EXPECT_TRUE(master->is_agv_onboarded("Manufacturer2", "SN001"));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, DuplicateOnboardingIsIgnored)
{
  auto master = create_master();
  master->connect();

  master->onboard_agv(manufacturer_, serial_number_);
  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  // Second onboarding should be ignored (no crash, no duplicate)
  master->onboard_agv(manufacturer_, serial_number_);
  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, OffboardNonExistentAGVIsIgnored)
{
  auto master = create_master();
  master->connect();

  // Should not throw or crash
  EXPECT_NO_THROW(master->offboard_agv("NonExistent", "AGV"));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, GetAGVReturnsNullptrForNonOnboardedAGV)
{
  auto master = create_master();
  master->connect();

  auto agv = master->get_agv(manufacturer_, serial_number_);
  EXPECT_EQ(agv, nullptr);

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, GetAGVReturnsValidAGVAfterOnboarding)
{
  auto master = create_master();
  master->connect();

  master->onboard_agv(manufacturer_, serial_number_);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);
  EXPECT_EQ(agv->get_interface_name(), interface_name_);
  EXPECT_EQ(agv->get_manufacturer(), manufacturer_);
  EXPECT_EQ(agv->get_serial_number(), serial_number_);
  EXPECT_EQ(agv->get_agv_id(), agv_id_);

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, GetAGVReturnsNullptrAfterOffboarding)
{
  auto master = create_master();
  master->connect();

  master->onboard_agv(manufacturer_, serial_number_);
  auto agv_before = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv_before, nullptr);

  master->offboard_agv(manufacturer_, serial_number_);

  auto agv_after = master->get_agv(manufacturer_, serial_number_);
  EXPECT_EQ(agv_after, nullptr);

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, ReOnboardingAfterOffboardingWorks)
{
  auto master = create_master();
  master->connect();

  // First onboarding
  master->onboard_agv(manufacturer_, serial_number_);
  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  auto agv1 = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv1, nullptr);

  // Offboard
  master->offboard_agv(manufacturer_, serial_number_);
  EXPECT_FALSE(master->is_agv_onboarded(manufacturer_, serial_number_));

  // Re-onboard
  master->onboard_agv(manufacturer_, serial_number_);
  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  auto agv2 = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv2, nullptr);

  // Should be a new instance (different pointer)
  EXPECT_NE(agv1.get(), agv2.get());

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, AGVQueuesOrders)
{
  auto master = create_master();
  master->connect();
  master->onboard_agv(manufacturer_, serial_number_);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);

  // Queue should work
  EXPECT_TRUE(agv->send_order(create_test_order("order_1")));
  EXPECT_TRUE(agv->send_order(create_test_order("order_2")));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, AGVQueuesInstantActions)
{
  auto master = create_master();
  master->connect();
  master->onboard_agv(manufacturer_, serial_number_);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);

  // Queue should work
  EXPECT_TRUE(agv->send_instant_actions(create_test_instant_actions(1)));
  EXPECT_TRUE(agv->send_instant_actions(create_test_instant_actions(2)));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, AGVDropOldestPolicyWorks)
{
  auto master = create_master();
  master->connect();

  // Onboard with small queue size and drop_oldest = true
  size_t queue_size = 2;
  master->onboard_agv(manufacturer_, serial_number_, queue_size, true);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);

  // Fill the queue
  EXPECT_TRUE(agv->send_order(create_test_order("order_1")));
  EXPECT_TRUE(agv->send_order(create_test_order("order_2")));

  // This should succeed (drops oldest)
  EXPECT_TRUE(agv->send_order(create_test_order("order_3")));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, AGVDropNewestPolicyWorks)
{
  auto master = create_master();
  master->connect();

  // Onboard with small queue size and drop_oldest = false (drop newest)
  size_t queue_size = 2;
  master->onboard_agv(manufacturer_, serial_number_, queue_size, false);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);

  // Fill the queue
  EXPECT_TRUE(agv->send_order(create_test_order("order_1")));
  EXPECT_TRUE(agv->send_order(create_test_order("order_2")));

  // This should fail (rejects new)
  EXPECT_FALSE(agv->send_order(create_test_order("order_3")));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, OnboardingWithCustomQueueSettings)
{
  auto master = create_master();
  master->connect();

  // Onboard with custom queue size and drop policy
  size_t custom_queue_size = 5;
  bool drop_oldest = false;
  master->onboard_agv(
    manufacturer_, serial_number_, custom_queue_size, drop_oldest);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);

  // Queue should accept up to custom_queue_size orders
  for (size_t i = 0; i < custom_queue_size; ++i)
  {
    EXPECT_TRUE(
      agv->send_order(create_test_order("order_" + std::to_string(i))));
  }

  // Next order should be rejected (drop_oldest = false means reject new)
  EXPECT_FALSE(agv->send_order(create_test_order("order_rejected")));

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, MasterConnectIsIdempotent)
{
  auto master = create_master();

  // Multiple connects should not cause issues
  master->connect();
  master->connect();
  master->connect();

  EXPECT_TRUE(master->is_connected());

  master->disconnect();
}

TEST_F(MasterLogicTestFixture, MasterDisconnectIsIdempotent)
{
  auto master = create_master();
  master->connect();

  // Multiple disconnects should not cause issues
  master->disconnect();
  master->disconnect();
  master->disconnect();

  EXPECT_FALSE(master->is_connected());
}

TEST_F(MasterLogicTestFixture, AGVsRemainsOnboardedAfterMasterDisconnect)
{
  auto master = create_master();
  master->connect();
  master->onboard_agv(manufacturer_, serial_number_);

  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->disconnect();

  // AGV should still be onboarded even if master is disconnected
  EXPECT_TRUE(master->is_agv_onboarded(manufacturer_, serial_number_));
}

TEST_F(MasterLogicTestFixture, DestructorCompletesWithPendingMessages)
{
  auto start = std::chrono::steady_clock::now();

  {
    auto master = create_master();
    master->connect();
    master->onboard_agv(manufacturer_, serial_number_);

    auto agv = master->get_agv(manufacturer_, serial_number_);
    ASSERT_NE(agv, nullptr);

    // Queue many messages
    for (int i = 0; i < 50; ++i)
    {
      agv->send_order(create_test_order("pending_" + std::to_string(i)));
    }

    // Master destructor should complete gracefully
  }

  auto elapsed = std::chrono::steady_clock::now() - start;

  // Should complete in reasonable time
  EXPECT_LT(elapsed, std::chrono::seconds(10))
    << "Destructor took too long with pending messages";
}

TEST_F(MasterLogicTestFixture, OffboardWithPendingMessages)
{
  auto master = create_master();
  master->connect();
  master->onboard_agv(manufacturer_, serial_number_);

  auto agv = master->get_agv(manufacturer_, serial_number_);
  ASSERT_NE(agv, nullptr);

  // Queue some messages
  for (int i = 0; i < 10; ++i)
  {
    agv->send_order(create_test_order("pending_" + std::to_string(i)));
  }

  // Offboard should complete gracefully
  auto start = std::chrono::steady_clock::now();
  master->offboard_agv(manufacturer_, serial_number_);
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_LT(elapsed, std::chrono::seconds(5)) << "Offboard took too long";

  EXPECT_FALSE(master->is_agv_onboarded(manufacturer_, serial_number_));

  master->disconnect();
}
