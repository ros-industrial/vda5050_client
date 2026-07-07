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

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

namespace vda5050_core::master::test {

namespace {

class MockMqttClient : public vda5050_core::transport::MqttClientInterface
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

class OnboardBatchTest : public ::testing::Test
{
protected:
  std::shared_ptr<MockMqttClient> mock_;
  std::shared_ptr<VDA5050Master> master_;

  void SetUp() override
  {
    mock_ = std::make_shared<MockMqttClient>();
    EXPECT_CALL(*mock_, connect()).Times(::testing::AnyNumber());
    EXPECT_CALL(*mock_, disconnect()).Times(::testing::AnyNumber());
    EXPECT_CALL(*mock_, connected())
      .Times(::testing::AnyNumber())
      .WillRepeatedly(::testing::Return(true));
    EXPECT_CALL(*mock_, subscribe(::testing::_, ::testing::_, ::testing::_))
      .Times(::testing::AnyNumber());
    EXPECT_CALL(*mock_, unsubscribe(::testing::_))
      .Times(::testing::AnyNumber());
    EXPECT_CALL(
      *mock_, set_will(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(::testing::AnyNumber());
    EXPECT_CALL(
      *mock_, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(::testing::AnyNumber());

    master_ = std::make_shared<VDA5050Master>(mock_);
  }

  static VDA5050Master::OnboardSpec spec(
    const std::string& mfg, const std::string& sn)
  {
    return VDA5050Master::OnboardSpec{mfg, sn, 10, true};
  }

  bool roster_contains(
    const std::vector<std::pair<std::string, std::string>>& v,
    const std::string& mfg, const std::string& sn)
  {
    return std::find(v.begin(), v.end(), std::make_pair(mfg, sn)) != v.end();
  }
};

}  // namespace

TEST_F(OnboardBatchTest, OnboardBatch_EmptyInputProducesEmptyResult)
{
  auto r = master_->onboard_agv_batch({});
  EXPECT_EQ(r.onboarded.size(), 0u);
  EXPECT_EQ(r.skipped_already_onboarded.size(), 0u);
  EXPECT_EQ(r.failed.size(), 0u);
  EXPECT_TRUE(r.failed.empty());
}

TEST_F(OnboardBatchTest, OnboardBatch_OnboardsAllFromEmptyMaster)
{
  std::vector<VDA5050Master::OnboardSpec> specs = {
    spec("ACME", "AGV01"), spec("ACME", "AGV02"), spec("OTHER", "BOT01")};
  auto r = master_->onboard_agv_batch(specs);

  EXPECT_EQ(r.onboarded.size(), 3u);
  EXPECT_EQ(r.skipped_already_onboarded.size(), 0u);
  EXPECT_EQ(r.failed.size(), 0u);

  auto roster = master_->get_onboarded_agvs();
  EXPECT_EQ(roster.size(), 3u);
  EXPECT_TRUE(roster_contains(roster, "ACME", "AGV01"));
  EXPECT_TRUE(roster_contains(roster, "ACME", "AGV02"));
  EXPECT_TRUE(roster_contains(roster, "OTHER", "BOT01"));
}

TEST_F(OnboardBatchTest, OnboardBatch_SkipsAlreadyOnboarded)
{
  master_->onboard_agv("ACME", "AGV01");

  std::vector<VDA5050Master::OnboardSpec> specs = {
    spec("ACME", "AGV01"),  // already onboarded
    spec("ACME", "AGV02")};
  auto r = master_->onboard_agv_batch(specs);

  EXPECT_EQ(r.onboarded.size(), 1u);
  EXPECT_EQ(r.skipped_already_onboarded.size(), 1u);
  EXPECT_EQ(r.failed.size(), 0u);
  EXPECT_EQ(master_->get_onboarded_agvs().size(), 2u);
}

TEST_F(OnboardBatchTest, OnboardBatch_RejectsEmptyKeys)
{
  std::vector<VDA5050Master::OnboardSpec> specs = {
    spec("", "AGV01"), spec("ACME", "")};
  auto r = master_->onboard_agv_batch(specs);

  EXPECT_EQ(r.onboarded.size(), 0u);
  EXPECT_EQ(r.skipped_already_onboarded.size(), 0u);
  EXPECT_EQ(r.failed.size(), 2u);
  EXPECT_TRUE(master_->get_onboarded_agvs().empty());
}

TEST_F(OnboardBatchTest, OnboardBatch_MixedSpecs)
{
  master_->onboard_agv("ACME", "AGV01");

  std::vector<VDA5050Master::OnboardSpec> specs = {
    spec("ACME", "AGV01"),  // already onboarded -> skip
    spec("ACME", "AGV02"),  // new -> onboard
    spec("", "AGV03"),      // empty mfg -> fail
    spec("OTHER", "BOT01")  // new -> onboard
  };
  auto r = master_->onboard_agv_batch(specs);

  EXPECT_EQ(r.onboarded.size(), 2u);
  EXPECT_EQ(r.skipped_already_onboarded.size(), 1u);
  EXPECT_EQ(r.failed.size(), 1u);
  EXPECT_EQ(master_->get_onboarded_agvs().size(), 3u);
}

TEST_F(OnboardBatchTest, OnboardBatch_RespectsPerSpecQueueConfig)
{
  std::vector<VDA5050Master::OnboardSpec> specs = {
    VDA5050Master::OnboardSpec{"ACME", "AGV01", 5, false},
    VDA5050Master::OnboardSpec{"ACME", "AGV02", 20, true}};
  auto r = master_->onboard_agv_batch(specs);
  EXPECT_EQ(r.onboarded.size(), 2u);
  // We can't directly observe queue config from outside, but both
  // AGVs should be reachable via get_agv.
  EXPECT_NE(master_->get_agv("ACME", "AGV01"), nullptr);
  EXPECT_NE(master_->get_agv("ACME", "AGV02"), nullptr);
}

TEST_F(OnboardBatchTest, OffboardBatch_OffboardsAll)
{
  master_->onboard_agv_batch(
    {spec("ACME", "AGV01"), spec("ACME", "AGV02"), spec("OTHER", "BOT01")});

  std::vector<std::pair<std::string, std::string>> keys = {
    {"ACME", "AGV01"}, {"OTHER", "BOT01"}};
  std::size_t removed = master_->offboard_agv_batch(keys);

  EXPECT_EQ(removed, 2u);
  auto roster = master_->get_onboarded_agvs();
  EXPECT_EQ(roster.size(), 1u);
  EXPECT_TRUE(roster_contains(roster, "ACME", "AGV02"));
}

TEST_F(OnboardBatchTest, OffboardBatch_IgnoresMissingKeys)
{
  master_->onboard_agv("ACME", "AGV01");

  std::vector<std::pair<std::string, std::string>> keys = {
    {"ACME", "AGV01"},  // present
    {"GHOST", "X999"},  // missing
    {"", "AGV02"}       // empty mfg
  };
  std::size_t removed = master_->offboard_agv_batch(keys);

  EXPECT_EQ(removed, 1u);
  EXPECT_TRUE(master_->get_onboarded_agvs().empty());
}

TEST_F(OnboardBatchTest, OffboardBatch_EmptyInputIsNoOp)
{
  master_->onboard_agv("ACME", "AGV01");
  EXPECT_EQ(master_->offboard_agv_batch({}), 0u);
  EXPECT_EQ(master_->get_onboarded_agvs().size(), 1u);
}

TEST_F(OnboardBatchTest, GetOnboardedAgvs_EmptyOnFreshMaster)
{
  EXPECT_TRUE(master_->get_onboarded_agvs().empty());
}

TEST_F(OnboardBatchTest, GetOnboardedAgvs_ReflectsSingleAndBatchOnboard)
{
  master_->onboard_agv("ACME", "AGV01");
  master_->onboard_agv_batch({spec("ACME", "AGV02"), spec("OTHER", "BOT01")});

  auto roster = master_->get_onboarded_agvs();
  EXPECT_EQ(roster.size(), 3u);
  EXPECT_TRUE(roster_contains(roster, "ACME", "AGV01"));
  EXPECT_TRUE(roster_contains(roster, "ACME", "AGV02"));
  EXPECT_TRUE(roster_contains(roster, "OTHER", "BOT01"));
}

}  // namespace vda5050_core::master::test
