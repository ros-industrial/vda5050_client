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
#include <memory>
#include <string>

#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/types/agv_class.hpp"
#include "vda5050_core/types/agv_kinematic.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

#include "vda5050_core/execution/protocol_adapter.hpp"

using vda5050_core::execution::ProtocolAdapter;
using vda5050_core::transport::MqttClientInterface;

using vda5050_core::types::AGVClass;
using vda5050_core::types::AGVKinematic;
using vda5050_core::types::Connection;
using vda5050_core::types::Error;
using vda5050_core::types::Factsheet;
using vda5050_core::types::InstantActions;
using vda5050_core::types::Order;
using vda5050_core::types::State;
using vda5050_core::types::Visualization;

using MessageTypes = testing::Types<
  Connection, Factsheet, InstantActions, Order, State, Visualization>;

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

template <typename T>
class ProtocolAdapterTest : public testing::Test
{
protected:
  std::shared_ptr<MockMqttClient> mock_;
  std::shared_ptr<ProtocolAdapter> adapter_;

  std::string interface_;
  std::string version_;
  std::string manufacturer_;
  std::string serial_number_;

  std::string topic_prefix_;

  int qos_;
  bool retained_;

  void SetUp()
  {
    interface_ = "uagv";
    version_ = "v2";
    manufacturer_ = "ROS-I";
    serial_number_ = "S001";

    mock_ = std::make_shared<MockMqttClient>();

    adapter_ = ProtocolAdapter::make(
      mock_, interface_, version_, manufacturer_, serial_number_);

    topic_prefix_ = fmt::format(
      "{}/{}/{}/{}/", interface_, version_, manufacturer_, serial_number_);

    qos_ = 0;
    retained_ = false;
  }

  void TearDown()
  {
    mock_.reset();
    adapter_.reset();
  }
};

template <typename T>
T make_valid_message();

template <>
Factsheet make_valid_message()
{
  Factsheet msg;
  msg.type_specification.agv_kinematic = AGVKinematic::DIFF;
  msg.type_specification.agv_class = AGVClass::CARRIER;
  return msg;
}

template <typename T>
T make_valid_message()
{
  return T{};
}

TYPED_TEST_SUITE(ProtocolAdapterTest, MessageTypes);

TYPED_TEST(ProtocolAdapterTest, Connect)
{
  EXPECT_CALL(*this->mock_, connect()).Times(1);
  this->adapter_->connect();
}

TYPED_TEST(ProtocolAdapterTest, Disconnect)
{
  EXPECT_CALL(*this->mock_, disconnect()).Times(1);
  this->adapter_->disconnect();
}

TYPED_TEST(ProtocolAdapterTest, Connected)
{
  EXPECT_CALL(*this->mock_, connected()).Times(1);
  this->adapter_->connected();
}

TYPED_TEST(ProtocolAdapterTest, PublishMessage)
{
  TypeParam msg = make_valid_message<TypeParam>();

  EXPECT_CALL(
    *this->mock_, publish(
                    testing::StartsWith(this->topic_prefix_), testing::_,
                    this->qos_, this->retained_))
    .WillOnce([&](
                const std::string& /*topic*/, const std::string& message,
                int /*qos*/, bool /*retained*/) {
      auto j = nlohmann::json::parse(message);

      EXPECT_EQ(j["headerId"], 0);
      EXPECT_EQ(j["version"], this->version_);
      EXPECT_EQ(j["manufacturer"], this->manufacturer_);
      EXPECT_EQ(j["serialNumber"], this->serial_number_);
    });

  this->adapter_->template publish<TypeParam>(msg, this->qos_, this->retained_);
}

TYPED_TEST(ProtocolAdapterTest, SubscribeMessage)
{
  MqttClientInterface::MessageHandler captured_handler;

  EXPECT_CALL(
    *this->mock_,
    subscribe(testing::StartsWith(this->topic_prefix_), testing::_, this->qos_))
    .WillOnce(testing::SaveArg<1>(&captured_handler));

  std::atomic_bool success = false;

  this->adapter_->template subscribe<TypeParam>(
    [&](TypeParam /*msg*/, std::optional<Error> err) {
      if (!err.has_value()) success = true;
    },
    this->qos_);

  TypeParam msg = make_valid_message<TypeParam>();
  nlohmann::json j = msg;

  captured_handler(this->topic_prefix_, j.dump());
  EXPECT_TRUE(success);
}

TYPED_TEST(ProtocolAdapterTest, UnsubscribeMessage)
{
  MqttClientInterface::MessageHandler captured_handler;

  EXPECT_CALL(
    *this->mock_,
    subscribe(testing::StartsWith(this->topic_prefix_), testing::_, this->qos_))
    .WillOnce(testing::SaveArg<1>(&captured_handler));

  std::atomic_bool callback_invoked = false;
  this->adapter_->template subscribe<TypeParam>(
    [&](TypeParam /*msg*/, std::optional<vda5050_core::types::Error> err) {
      if (!err.has_value()) callback_invoked = true;
    },
    this->qos_);

  // While subscribed, the captured wrapper should dispatch to the
  // user callback.
  TypeParam msg = make_valid_message<TypeParam>();
  nlohmann::json j = msg;
  captured_handler(this->topic_prefix_, j.dump());
  EXPECT_TRUE(callback_invoked);

  // Unsubscribe.
  EXPECT_CALL(
    *this->mock_, unsubscribe(testing::StartsWith(this->topic_prefix_)))
    .Times(1);
  this->adapter_->template unsubscribe<TypeParam>();

  // Invoking the captured wrapper after unsubscribe should NOT
  // dispatch — the wrapper is now inert.
  callback_invoked = false;
  captured_handler(this->topic_prefix_, j.dump());
  EXPECT_FALSE(callback_invoked);
}

TYPED_TEST(ProtocolAdapterTest, UnsubscribeWithoutSubscription)
{
  EXPECT_CALL(
    *this->mock_, unsubscribe(testing::StartsWith(this->topic_prefix_)))
    .Times(1);

  this->adapter_->template unsubscribe<TypeParam>();
}

TYPED_TEST(ProtocolAdapterTest, HeaderIncrement)
{
  TypeParam msg = make_valid_message<TypeParam>();

  std::atomic_uint32_t header_count = 0;

  EXPECT_CALL(
    *this->mock_, publish(
                    testing::StartsWith(this->topic_prefix_), testing::_,
                    this->qos_, this->retained_))
    .WillRepeatedly([&](
                      const std::string& /*topic*/, const std::string& message,
                      int /*qos*/, bool /*retained*/) {
      auto j = nlohmann::json::parse(message);

      EXPECT_EQ(j["headerId"].get<uint32_t>(), header_count);
      header_count++;
    });

  this->adapter_->template publish<TypeParam>(msg, this->qos_, this->retained_);
  this->adapter_->template publish<TypeParam>(msg, this->qos_, this->retained_);
}

TYPED_TEST(ProtocolAdapterTest, UnsubscribeAllOnlyUnsubscribesActiveTopics)
{
  // Nothing subscribed — unsubscribe_all() must not touch mqtt_client_.
  EXPECT_CALL(*this->mock_, unsubscribe(testing::_)).Times(0);

  this->adapter_->unsubscribe_all();
}

TYPED_TEST(ProtocolAdapterTest, UnsubscribeAllSilencesActiveCallbacks)
{
  MqttClientInterface::MessageHandler captured_handler;

  EXPECT_CALL(
    *this->mock_,
    subscribe(testing::StartsWith(this->topic_prefix_), testing::_, this->qos_))
    .WillOnce(testing::SaveArg<1>(&captured_handler));

  std::atomic_bool callback_invoked = false;
  this->adapter_->template subscribe<TypeParam>(
    [&](TypeParam /*msg*/, std::optional<Error> err) {
      if (!err.has_value()) callback_invoked = true;
    },
    this->qos_);

  // While subscribed, the captured wrapper should dispatch to the
  // user callback.
  TypeParam msg = make_valid_message<TypeParam>();
  nlohmann::json j = msg;
  captured_handler(this->topic_prefix_, j.dump());
  EXPECT_TRUE(callback_invoked);

  // After unsubscribe_all, the wrapper should be inert.
  EXPECT_CALL(
    *this->mock_, unsubscribe(testing::StartsWith(this->topic_prefix_)))
    .Times(1);
  this->adapter_->unsubscribe_all();

  callback_invoked = false;
  captured_handler(this->topic_prefix_, j.dump());
  EXPECT_FALSE(callback_invoked);
}
