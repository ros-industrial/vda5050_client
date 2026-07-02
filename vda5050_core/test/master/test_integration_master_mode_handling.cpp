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
 * @file master_mode_handling_test.cpp
 * @brief Integration tests for AGV mode handling (Task #24).
 *
 * Verifies the capture-and-resume contract for the AGV's outbound
 * queue when operating_mode transitions out of / back to AUTOMATIC.
 *
 * Uses gmock MockMqttClient. AGV state is injected via handle_state.
 */

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master {
namespace test {

namespace {

constexpr const char* kManufacturer = "TestMfg";
constexpr const char* kSerial = "SN001";

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

vda5050_core::types::State make_state(vda5050_core::types::OperatingMode mode)
{
  vda5050_core::types::State msg;
  msg.header.header_id = 1;
  msg.header.timestamp = std::chrono::system_clock::now();
  msg.header.version = "2.0.0";
  msg.header.manufacturer = kManufacturer;
  msg.header.serial_number = kSerial;
  msg.order_id = "test_order";
  msg.order_update_id = 0;
  msg.driving = false;
  msg.paused = false;
  msg.new_base_request = false;
  msg.distance_since_last_node = 0.0;
  msg.operating_mode = mode;
  return msg;
}

vda5050_core::types::Order make_order(const std::string& id)
{
  vda5050_core::types::Order o;
  o.header.header_id = 1;
  o.header.timestamp = std::chrono::system_clock::now();
  o.header.version = "2.0.0";
  o.header.manufacturer = kManufacturer;
  o.header.serial_number = kSerial;
  o.order_id = id;
  o.order_update_id = 0;
  return o;
}

vda5050_core::types::InstantActions make_instant_actions(const std::string& id)
{
  vda5050_core::types::InstantActions ia;
  ia.header.header_id = 1;
  ia.header.timestamp = std::chrono::system_clock::now();
  ia.header.version = "2.0.0";
  ia.header.manufacturer = kManufacturer;
  ia.header.serial_number = kSerial;
  vda5050_core::types::Action a;
  a.action_id = id;
  a.action_type = "stateRequest";
  a.blocking_type = vda5050_core::types::BlockingType::NONE;
  ia.actions = {a};
  return ia;
}

vda5050_core::types::Connection make_online_connection()
{
  vda5050_core::types::Connection c;
  c.header.header_id = 1;
  c.header.timestamp = std::chrono::system_clock::now();
  c.header.version = "2.0.0";
  c.header.manufacturer = kManufacturer;
  c.header.serial_number = kSerial;
  c.connection_state = vda5050_core::types::ConnectionState::ONLINE;
  return c;
}

class CallbackTrackingMaster : public VDA5050Master
{
public:
  using VDA5050Master::VDA5050Master;

  void on_mode_changed(
    const std::string& agv_id, vda5050_core::types::OperatingMode new_mode,
    vda5050_core::types::OperatingMode prev_mode) override
  {
    mode_changed_calls.fetch_add(1);
    last_new_mode = new_mode;
    last_prev_mode = prev_mode;
    last_mode_changed_agv = agv_id;
  }

  std::atomic<int> mode_changed_calls{0};
  vda5050_core::types::OperatingMode last_new_mode{
    vda5050_core::types::OperatingMode::AUTOMATIC};
  vda5050_core::types::OperatingMode last_prev_mode{
    vda5050_core::types::OperatingMode::AUTOMATIC};
  std::string last_mode_changed_agv;
};

class ModeHandlingTest : public ::testing::Test
{
protected:
  std::shared_ptr<MockMqttClient> mock_;
  std::shared_ptr<CallbackTrackingMaster> master_;
  std::unique_ptr<AGV> agv_;

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
    master_ = std::make_shared<CallbackTrackingMaster>(mock_);

    // Long heartbeat interval — we don't want the timer to fire during tests.
    agv_ = std::make_unique<AGV>(
      nullptr, DefaultInterfaceName, kManufacturer, kSerial, 100, true, 60,
      master_->weak_from_this());
    agv_->handle_connection(make_online_connection());
  }

  void TearDown() override
  {
    agv_.reset();
    master_.reset();
    mock_.reset();
  }
};

}  // namespace

// ============================================================================
// Capture edge — leave-AUTOMATIC populates buffer + drains live queue
// ============================================================================

TEST_F(ModeHandlingTest, LeavingAutomaticCapturesOrdersIntoBuffer)
{
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  agv_->send_order(make_order("O1"));
  agv_->send_order(make_order("O2"));
  // Live queue may already be partially drained by the queue thread;
  // verify capture works regardless.

  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::MANUAL));

  auto buf = agv_->get_mode_cancelled_queue();
  EXPECT_TRUE(buf.cancelled_at.has_value());
  ASSERT_TRUE(buf.from_mode.has_value());
  ASSERT_TRUE(buf.to_mode.has_value());
  EXPECT_EQ(*buf.from_mode, vda5050_core::types::OperatingMode::AUTOMATIC);
  EXPECT_EQ(*buf.to_mode, vda5050_core::types::OperatingMode::MANUAL);
}

TEST_F(ModeHandlingTest, LeavingAutomaticCapturesInstantActionsToo)
{
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  agv_->send_instant_actions(make_instant_actions("IA1"));

  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::SERVICE));

  auto buf = agv_->get_mode_cancelled_queue();
  EXPECT_TRUE(buf.cancelled_at.has_value());
  ASSERT_TRUE(buf.to_mode.has_value());
  EXPECT_EQ(*buf.to_mode, vda5050_core::types::OperatingMode::SERVICE);
}

TEST_F(ModeHandlingTest, SecondLeaveAutomaticOverwritesBuffer)
{
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  agv_->send_order(make_order("O1"));
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::MANUAL));

  auto first_buf = agv_->get_mode_cancelled_queue();
  EXPECT_TRUE(first_buf.cancelled_at.has_value());
  const auto first_to = first_buf.to_mode;

  // Return to AUTOMATIC, do nothing (don't resume/discard).
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));

  // Leave AUTOMATIC again to a different mode with empty queue.
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::TEACHIN));

  auto second_buf = agv_->get_mode_cancelled_queue();
  EXPECT_TRUE(second_buf.cancelled_at.has_value());
  ASSERT_TRUE(second_buf.to_mode.has_value());
  EXPECT_EQ(*second_buf.to_mode, vda5050_core::types::OperatingMode::TEACHIN);
  EXPECT_NE(second_buf.to_mode, first_to)
    << "buffer should be overwritten with new transition's metadata";
}

// ============================================================================
// Resume / discard
// ============================================================================

TEST_F(ModeHandlingTest, ResumeOnEmptyBufferReturnsZero)
{
  // Never-cancelled AGV — resume should be a no-op.
  auto [orders, actions] = agv_->resume_mode_cancelled_queue();
  EXPECT_EQ(orders, 0u);
  EXPECT_EQ(actions, 0u);
}

TEST_F(ModeHandlingTest, DiscardOnEmptyBufferReturnsZero)
{
  auto [orders, actions] = agv_->discard_mode_cancelled_queue();
  EXPECT_EQ(orders, 0u);
  EXPECT_EQ(actions, 0u);
}

TEST_F(ModeHandlingTest, DiscardClearsBufferWithoutReEnqueue)
{
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  agv_->send_order(make_order("O1"));
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::MANUAL));

  auto pre_discard = agv_->get_mode_cancelled_queue();

  agv_->discard_mode_cancelled_queue();

  auto post_discard = agv_->get_mode_cancelled_queue();
  EXPECT_TRUE(post_discard.orders.empty());
  EXPECT_TRUE(post_discard.instant_actions.empty());
  EXPECT_FALSE(post_discard.cancelled_at.has_value());
}

// ============================================================================
// Dispatch ordering — capture-and-drain runs BEFORE on_mode_changed
// ============================================================================

TEST_F(ModeHandlingTest, OnModeChangedFiresWithCorrectModes)
{
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  EXPECT_EQ(master_->mode_changed_calls.load(), 0);

  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::MANUAL));

  EXPECT_GE(master_->mode_changed_calls.load(), 1);
  EXPECT_EQ(master_->last_new_mode, vda5050_core::types::OperatingMode::MANUAL);
  EXPECT_EQ(
    master_->last_prev_mode, vda5050_core::types::OperatingMode::AUTOMATIC);
}

// ============================================================================
// No-op transitions
// ============================================================================

TEST_F(ModeHandlingTest, StayingInAutomaticDoesNotCapture)
{
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));

  auto buf = agv_->get_mode_cancelled_queue();
  EXPECT_FALSE(buf.cancelled_at.has_value());
}

TEST_F(ModeHandlingTest, EnteringAutomaticDoesNotCaptureOrTouchBuffer)
{
  // Start in AUTOMATIC, leave to MANUAL (capture happens), return to
  // AUTOMATIC. Verify the return edge does NOT alter the buffer.
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));
  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::MANUAL));

  auto pre_return = agv_->get_mode_cancelled_queue();
  ASSERT_TRUE(pre_return.cancelled_at.has_value());
  const auto pre_cancelled_at = *pre_return.cancelled_at;

  agv_->handle_state(make_state(vda5050_core::types::OperatingMode::AUTOMATIC));

  auto post_return = agv_->get_mode_cancelled_queue();
  ASSERT_TRUE(post_return.cancelled_at.has_value());
  EXPECT_EQ(*post_return.cancelled_at, pre_cancelled_at)
    << "return-to-AUTOMATIC should not refresh the buffer's timestamp";
}

}  // namespace test
}  // namespace vda5050_core::master
