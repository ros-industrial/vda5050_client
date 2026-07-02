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

#include <functional>
#include <memory>
#include <string>

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

using vda5050_core::master::VDA5050Master;
using vda5050_core::transport::MqttClientInterface;

class MockMqttClient : public MqttClientInterface
{
public:
  MOCK_METHOD(void, connect, (), (override));
  MOCK_METHOD(void, disconnect, (), (override));
  MOCK_METHOD(bool, connected, (), (override));
  MOCK_METHOD(
    void, publish, (const std::string&, const std::string&, int, bool),
    (override));
  MOCK_METHOD(
    void, subscribe,
    (const std::string&,
     std::function<void(const std::string&, const std::string&)>, int),
    (override));
  MOCK_METHOD(void, unsubscribe, (const std::string&), (override));
  MOCK_METHOD(
    void, set_will, (const std::string&, const std::string&, int, bool),
    (override));
};

class MasterTeardownTest : public ::testing::Test
{
protected:
  std::shared_ptr<MockMqttClient> mock_;

  void SetUp() override
  {
    mock_ = std::make_shared<MockMqttClient>();
    // Allow connect() / connected() / subscribe() / disconnect() during
    // the test setup phase without explicit per-call expectations.
    EXPECT_CALL(*mock_, connect()).Times(::testing::AnyNumber());
    EXPECT_CALL(*mock_, disconnect()).Times(::testing::AnyNumber());
    EXPECT_CALL(*mock_, connected())
      .Times(::testing::AnyNumber())
      .WillRepeatedly(::testing::Return(true));
    EXPECT_CALL(*mock_, subscribe(::testing::_, ::testing::_, ::testing::_))
      .Times(::testing::AnyNumber());
  }
};

TEST_F(MasterTeardownTest, OffboardAgvUnsubscribesAllPerAgvTopics)
{
  auto master = std::make_shared<VDA5050Master>(mock_);
  master->connect();
  master->onboard_agv("acme", "agv-001");

  EXPECT_CALL(
    *mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/connection")))
    .Times(1);
  EXPECT_CALL(*mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/state")))
    .Times(1);
  EXPECT_CALL(
    *mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/factsheet")))
    .Times(1);
  EXPECT_CALL(
    *mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/visualization")))
    .Times(1);

  master->offboard_agv("acme", "agv-001");
}

TEST_F(MasterTeardownTest, OffboardAgvUnsubscribesAllPerAgvTopicsInterfaceName)
{
  auto master = std::make_shared<VDA5050Master>(mock_);
  master->connect();
  {  // default interface name
    master->onboard_agv("acme", "agv-001");

    EXPECT_CALL(
      *mock_,
      unsubscribe(::testing::HasSubstr("uagv/v2/acme/agv-001/connection")))
      .Times(1);
    EXPECT_CALL(
      *mock_, unsubscribe(::testing::HasSubstr("uagv/v2/acme/agv-001/state")))
      .Times(1);
    EXPECT_CALL(
      *mock_,
      unsubscribe(::testing::HasSubstr("uagv/v2/acme/agv-001/factsheet")))
      .Times(1);
    EXPECT_CALL(
      *mock_,
      unsubscribe(::testing::HasSubstr("uagv/v2/acme/agv-001/visualization")))
      .Times(1);

    master->offboard_agv("acme", "agv-001");
  }

  {  // custom interface name
    std::string custom_interface_name = "amr";
    master->onboard_agv(custom_interface_name, "acme", "agv-001");

    EXPECT_CALL(
      *mock_,
      unsubscribe(::testing::HasSubstr("amr/v2/acme/agv-001/connection")))
      .Times(1);
    EXPECT_CALL(
      *mock_, unsubscribe(::testing::HasSubstr("amr/v2/acme/agv-001/state")))
      .Times(1);
    EXPECT_CALL(
      *mock_,
      unsubscribe(::testing::HasSubstr("amr/v2/acme/agv-001/factsheet")))
      .Times(1);
    EXPECT_CALL(
      *mock_,
      unsubscribe(::testing::HasSubstr("amr/v2/acme/agv-001/visualization")))
      .Times(1);

    master->offboard_agv("acme", "agv-001");
  }
}

TEST_F(MasterTeardownTest, MasterDestructionUnsubscribesAllPerAgvTopics)
{
  EXPECT_CALL(
    *mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/connection")))
    .Times(1);
  EXPECT_CALL(*mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/state")))
    .Times(1);
  EXPECT_CALL(
    *mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/factsheet")))
    .Times(1);
  EXPECT_CALL(
    *mock_, unsubscribe(::testing::HasSubstr("acme/agv-001/visualization")))
    .Times(1);

  {
    auto master = std::make_shared<VDA5050Master>(mock_);
    master->connect();
    master->onboard_agv("acme", "agv-001");
    // master leaves scope → AGV destructor runs → unsubscribe chain
  }
}

TEST_F(MasterTeardownTest, MultipleAgvsAllUnsubscribeOnOffboard)
{
  auto master = std::make_shared<VDA5050Master>(mock_);
  master->connect();
  master->onboard_agv("mfg1", "001");
  master->onboard_agv("mfg2", "002");

  EXPECT_CALL(*mock_, unsubscribe(::testing::HasSubstr("mfg1/001/"))).Times(4);
  EXPECT_CALL(*mock_, unsubscribe(::testing::HasSubstr("mfg2/002/"))).Times(4);

  master->offboard_agv("mfg1", "001");
  master->offboard_agv("mfg2", "002");
}
