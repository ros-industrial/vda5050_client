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
 * @file master_state_timeout_test.cpp
 * @brief Integration tests for VDA5050Master::on_state_timeout and
 *        on_state_resumed virtuals (Task #28).
 *
 * Verifies that the named state-heartbeat edges fire on the AGV's
 * background timer thread (timeout) and on the next state-message
 * arrival (recovery), with the expected once-per-edge contract.
 *
 * Uses gmock MockMqttClient so tests run without a broker. AGV state
 * is injected by calling AGV::handle_state directly.
 */

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master::test {

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

// Subclass that records every #28 edge dispatch. Counters use atomics
// because on_state_timeout fires from the HeartbeatListener's monitor
// thread.
class CallbackTrackingMaster : public VDA5050Master
{
public:
  using VDA5050Master::VDA5050Master;

  void on_state_timeout(const std::string& agv_id) override
  {
    timeout_calls.fetch_add(1);
    std::lock_guard<std::mutex> lock(str_mu_);
    last_timeout_agv_ = agv_id;
  }

  void on_state_resumed(const std::string& agv_id) override
  {
    resumed_calls.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(str_mu_);
      last_resumed_agv_ = agv_id;
    }
    // Record dispatch order vs on_state — used to verify on_state_resumed
    // fires BEFORE on_state.
    if (state_calls.load() == 0)
    {
      resumed_before_first_state = true;
    }
  }

  void on_state(
    const std::string& agv_id,
    const vda5050_core::types::State& /*state*/) override
  {
    state_calls.fetch_add(1);
    std::lock_guard<std::mutex> lock(str_mu_);
    last_state_agv_ = agv_id;
  }

  std::string last_timeout_agv() const
  {
    std::lock_guard<std::mutex> lock(str_mu_);
    return last_timeout_agv_;
  }
  std::string last_resumed_agv() const
  {
    std::lock_guard<std::mutex> lock(str_mu_);
    return last_resumed_agv_;
  }
  std::string last_state_agv() const
  {
    std::lock_guard<std::mutex> lock(str_mu_);
    return last_state_agv_;
  }

  std::atomic<int> timeout_calls{0};
  std::atomic<int> resumed_calls{0};
  std::atomic<int> state_calls{0};
  std::atomic<bool> resumed_before_first_state{false};

private:
  // Last-seen agv_id strings are written from the heartbeat monitor
  // thread (on_state_timeout) and the MQTT/handle thread (on_state*)
  // while test assertions read from the main thread. Protect them
  // behind a mutex — std::string isn't safe for cross-thread plain
  // reads/writes even when the writer "finishes before" the reader,
  // since TSan rightly flags the unsynchronized handoff.
  mutable std::mutex str_mu_;
  std::string last_timeout_agv_;
  std::string last_resumed_agv_;
  std::string last_state_agv_;
};

vda5050_core::types::State make_state_msg()
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
  return msg;
}

class StateTimeoutCallbackTest : public ::testing::Test
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
  }

  void TearDown() override
  {
    agv_.reset();
    master_.reset();
    mock_.reset();
  }

  // Construct AGV with master back-pointer + fast heartbeat. Callers
  // must call setup_subscriptions() if they need MQTT routing — for
  // these tests we only inject state directly, so we skip that.
  void create_agv_with_heartbeat(int interval_seconds)
  {
    agv_ = std::make_unique<AGV>(
      nullptr, DefaultInterfaceName, kManufacturer, kSerial, 10, true,
      interval_seconds, master_->weak_from_this());
  }

  void inject_online_connection()
  {
    vda5050_core::types::Connection c;
    c.header.header_id = 1;
    c.header.timestamp = std::chrono::system_clock::now();
    c.header.version = "2.0.0";
    c.header.manufacturer = kManufacturer;
    c.header.serial_number = kSerial;
    c.connection_state = vda5050_core::types::ConnectionState::ONLINE;
    agv_->handle_connection(c);
  }

  // Poll until the predicate is true or the deadline elapses. Used in
  // place of fixed sleeps so heartbeat-timer assertions remain robust
  // under TSan/ASan instrumentation, which can slow timer-thread wake-
  // ups well past the 1s nominal heartbeat interval.
  template <typename Pred>
  bool wait_for(Pred pred, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (pred()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return pred();
  }
};

}  // namespace

// ============================================================================
// Timeout edge — on_state_timeout fires when 30s window violated
// ============================================================================

TEST_F(StateTimeoutCallbackTest, OnStateTimeoutFiresWhenHeartbeatExceeded)
{
  create_agv_with_heartbeat(1);
  inject_online_connection();

  // First state arrival flips operational_state to AVAILABLE.
  agv_->handle_state(make_state_msg());
  EXPECT_EQ(agv_->get_operational_state(), AGVState::AVAILABLE);
  EXPECT_EQ(master_->timeout_calls.load(), 0);

  // Wait past the heartbeat interval — timer fires on its monitor thread.
  // Poll rather than fixed-sleep so the test stays robust under sanitizer
  // slowdown (TSan can stretch a 1s timer well beyond 2.5s wall time).
  ASSERT_TRUE(wait_for(
    [&] { return master_->timeout_calls.load() >= 1; },
    std::chrono::milliseconds(8000)));

  EXPECT_EQ(agv_->get_operational_state(), AGVState::STATE_UNKNOWN);
  EXPECT_GE(master_->timeout_calls.load(), 1);
  EXPECT_EQ(
    master_->last_timeout_agv(), std::string(kManufacturer) + "/" + kSerial);
}

TEST_F(StateTimeoutCallbackTest, OnStateTimeoutFiresOncePerSilenceEpisode)
{
  create_agv_with_heartbeat(1);
  inject_online_connection();
  agv_->handle_state(make_state_msg());

  // Wait for the first fire, then sleep well past additional heartbeat
  // intervals — verifies the HeartbeatListener's one-shot guard prevents
  // re-fire while silent. Polling for the first fire is robust to
  // sanitizer slowdown; the trailing sleep is the actual no-re-fire test.
  ASSERT_TRUE(wait_for(
    [&] { return master_->timeout_calls.load() >= 1; },
    std::chrono::milliseconds(8000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  EXPECT_EQ(master_->timeout_calls.load(), 1)
    << "timeout dispatch should fire exactly once per silence episode";
}

// ============================================================================
// Recovery edge — on_state_resumed fires on next state after timeout
// ============================================================================

TEST_F(StateTimeoutCallbackTest, OnStateResumedFiresOnRecoveryAfterTimeout)
{
  create_agv_with_heartbeat(1);
  inject_online_connection();
  agv_->handle_state(make_state_msg());

  // First state already fired one resumed (initial UNKNOWN→AVAILABLE).
  // Snapshot counter, then drive a timeout + recovery cycle.
  const int resumed_after_first = master_->resumed_calls.load();
  EXPECT_GE(resumed_after_first, 1);

  ASSERT_TRUE(wait_for(
    [&] { return master_->timeout_calls.load() >= 1; },
    std::chrono::milliseconds(8000)));
  EXPECT_EQ(agv_->get_operational_state(), AGVState::STATE_UNKNOWN);
  EXPECT_GE(master_->timeout_calls.load(), 1);

  // Recovery: next state arrival
  agv_->handle_state(make_state_msg());
  EXPECT_EQ(agv_->get_operational_state(), AGVState::AVAILABLE);
  EXPECT_EQ(master_->resumed_calls.load(), resumed_after_first + 1);
}

TEST_F(StateTimeoutCallbackTest, OnStateResumedFiresOnFirstStateFromUnknown)
{
  // Documents the contract: every STATE_UNKNOWN→AVAILABLE edge fires
  // recovery, including the AGV's very first State message (initial
  // STATE_UNKNOWN at construction → AVAILABLE).
  create_agv_with_heartbeat(60);  // long interval — no timer firing
  inject_online_connection();

  EXPECT_EQ(agv_->get_operational_state(), AGVState::STATE_UNKNOWN);
  EXPECT_EQ(master_->resumed_calls.load(), 0);

  agv_->handle_state(make_state_msg());

  EXPECT_EQ(agv_->get_operational_state(), AGVState::AVAILABLE);
  EXPECT_EQ(master_->resumed_calls.load(), 1);
  EXPECT_EQ(
    master_->last_resumed_agv(), std::string(kManufacturer) + "/" + kSerial);
}

TEST_F(StateTimeoutCallbackTest, OnStateResumedDoesNotFireWhenAlreadyAvailable)
{
  // Once AVAILABLE, subsequent State messages should NOT re-fire
  // recovery — only the STATE_UNKNOWN→AVAILABLE edge counts.
  create_agv_with_heartbeat(60);
  inject_online_connection();

  agv_->handle_state(make_state_msg());  // resumed_calls = 1 (initial)
  EXPECT_EQ(master_->resumed_calls.load(), 1);

  agv_->handle_state(make_state_msg());
  agv_->handle_state(make_state_msg());

  EXPECT_EQ(master_->resumed_calls.load(), 1)
    << "recovery should fire only on the unknown→available edge";
}

TEST_F(StateTimeoutCallbackTest, OnStateResumedFiresBeforeOnState)
{
  // Verifies dispatch order: on_state_resumed runs BEFORE on_state on
  // the recovery edge, so observers see the named edge first.
  create_agv_with_heartbeat(60);
  inject_online_connection();
  agv_->handle_state(make_state_msg());  // first state — counters bump

  EXPECT_TRUE(master_->resumed_before_first_state.load());
}

}  // namespace vda5050_core::master::test
