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
 * @file master_assign_instant_actions_test.cpp
 * @brief Integration tests for VDA5050Master::assign_instant_actions
 *        (Task #20).
 *
 * Verifies that the synchronous pre-flight (onboarded, connection
 * ONLINE, action_id uniqueness) returns the right
 * InstantActionDecision and rich error feedback. Mirrors the pattern
 * from master_assign_order_test.cpp (#15) but with a deliberately
 * lighter pre-flight: instantActions are designed to function in
 * degraded states (cancelOrder during ERROR, factsheetRequest before
 * any state report, initPosition before position is initialized).
 */

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/layout/lif.hpp"
#include "vda5050_core/master/actions/action_factory.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_state.hpp"
#include "vda5050_core/types/action_status.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master::test {

namespace {

constexpr const char* kManufacturer = "ACME";
constexpr const char* kSerial = "AGV001";

// Minimal map used by all assign_instant_actions tests — satisfies the
// no-map gate (Task #39); the IA tests don't reference specific nodes
// or edges so a single-node map suffices.
vda5050_core::layout::Graph::ConstPtr make_test_graph()
{
  vda5050_core::layout::LIF lif;
  vda5050_core::layout::Layout layout;
  layout.layout_id = "L1";
  vda5050_core::layout::Node n;
  n.node_id = "N0";
  n.map_id = "L1";
  n.node_position = {0.0, 0.0};
  n.vehicle_type_node_properties.push_back({"v1", std::nullopt, std::nullopt});
  layout.nodes.push_back(n);
  lif.layouts.push_back(std::move(layout));
  return vda5050_core::layout::Graph::from_lif(std::move(lif));
}

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

vda5050_core::types::State make_ready_state(
  vda5050_core::types::OperatingMode mode =
    vda5050_core::types::OperatingMode::AUTOMATIC,
  bool position_initialized = true, const std::string& last_node_id = "N0")
{
  vda5050_core::types::State s;
  s.header.header_id = 1;
  s.header.timestamp = std::chrono::system_clock::now();
  s.header.version = "2.0.0";
  s.header.manufacturer = kManufacturer;
  s.header.serial_number = kSerial;
  s.last_node_id = last_node_id;
  s.last_node_sequence_id = 0;
  s.driving = false;
  s.paused = false;
  s.new_base_request = false;
  s.distance_since_last_node = 0.0;
  s.operating_mode = mode;

  vda5050_core::types::AGVPosition pos;
  pos.position_initialized = position_initialized;
  pos.x = 0.0;
  pos.y = 0.0;
  pos.theta = 0.0;
  pos.map_id = "test_map";
  s.agv_position = pos;
  return s;
}

// Build an order with one released node N0@0 carrying one action with
// the given action_id. Used to test that assign_instant_actions's
// uniqueness check sees the active order's node actions.
vda5050_core::types::Order make_order_with_node_action(
  const std::string& action_id, const std::string& order_id = "ORDER-1")
{
  vda5050_core::types::Order o;
  o.header.header_id = 1;
  o.header.timestamp = std::chrono::system_clock::now();
  o.header.version = "2.0.0";
  o.header.manufacturer = kManufacturer;
  o.header.serial_number = kSerial;
  o.order_id = order_id;
  o.order_update_id = 0;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  vda5050_core::types::Action node_action;
  node_action.action_type = "pick";
  node_action.action_id = action_id;
  node_action.blocking_type = vda5050_core::types::BlockingType::NONE;
  n.actions = {node_action};
  o.nodes = {n};
  return o;
}

template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

vda5050_core::types::InstantActions wrap(
  const std::vector<vda5050_core::types::Action>& actions)
{
  vda5050_core::types::InstantActions msg;
  msg.header.header_id = 1;
  msg.header.timestamp = std::chrono::system_clock::now();
  msg.header.version = "2.0.0";
  msg.header.manufacturer = kManufacturer;
  msg.header.serial_number = kSerial;
  msg.actions = actions;
  return msg;
}

class MasterAssignInstantActionsTest : public ::testing::Test
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
    master_->set_graph(make_test_graph());
    master_->onboard_agv(kManufacturer, kSerial);
  }

  void inject_online_and_state(
    vda5050_core::types::OperatingMode mode =
      vda5050_core::types::OperatingMode::AUTOMATIC,
    bool position_initialized = true)
  {
    auto agv = master_->get_agv(kManufacturer, kSerial);
    ASSERT_NE(agv, nullptr);
    agv->handle_connection(make_online_connection());
    agv->handle_state(make_ready_state(mode, position_initialized));
  }
};

// =============================================================================
// Pre-flight checks
// =============================================================================

TEST_F(MasterAssignInstantActionsTest, AgvNotOnboarded_Returns_AgvNotOnboarded)
{
  auto act = ActionFactory::build_custom("stateRequest", "act-1");
  auto res = master_->assign_instant_actions("OTHER", "UNKNOWN", wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::AGV_NOT_ONBOARDED);
  ASSERT_FALSE(res.errors.empty());
}

TEST_F(MasterAssignInstantActionsTest, AgvOffline_Returns_AgvOffline)
{
  // No connection injected — connection_status defaults to OFFLINE.
  auto act = ActionFactory::build_custom("stateRequest", "act-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::AGV_OFFLINE);
  ASSERT_FALSE(res.errors.empty());
}

TEST_F(MasterAssignInstantActionsTest, HappyPath_ReturnsAssigned_AndQueues)
{
  inject_online_and_state();

  auto act = ActionFactory::build_custom("stateRequest", "act-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::ASSIGNED);
  EXPECT_TRUE(res.errors.empty());
  EXPECT_TRUE(static_cast<bool>(res));
}

TEST_F(
  MasterAssignInstantActionsTest, OperatingModeManual_ExemptAction_StillAssigns)
{
  // Per §6.10.6 + §6.8.1 mode-gate: instant-scope predefined actions
  // (stateRequest etc.) are exempt from the AUTOMATIC requirement and
  // can be sent in MANUAL/SERVICE/TEACHIN — they're designed for
  // diagnostic + recovery use cases.
  inject_online_and_state(vda5050_core::types::OperatingMode::MANUAL);

  auto act = ActionFactory::build_state_request("act-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::ASSIGNED);
}

TEST_F(
  MasterAssignInstantActionsTest,
  OperatingModeManual_NonExemptAction_Returns_AgvModeNotAutoForAction)
{
  // The master must not send non-recovery actions in MANUAL. A custom
  // (non-allowlist) action_type in MANUAL should be rejected.
  inject_online_and_state(vda5050_core::types::OperatingMode::MANUAL);

  auto act = ActionFactory::build_custom(
    "customAction", "act-1", vda5050_core::types::BlockingType::NONE);
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::AGV_MODE_NOT_AUTO_FOR_ACTION);
  ASSERT_FALSE(res.errors.empty());
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::ModeValidationError);
}

TEST_F(MasterAssignInstantActionsTest, PositionNotInitialized_StillAssigns)
{
  // initPosition is itself an instant action and runs BEFORE position is
  // initialized. The skip is deliberate per Task #20 design.
  inject_online_and_state(
    vda5050_core::types::OperatingMode::AUTOMATIC,
    /*position_initialized=*/false);

  auto act = ActionFactory::build_custom("initPosition", "act-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::ASSIGNED);
}

TEST_F(MasterAssignInstantActionsTest, NoStateYet_StillAssigns)
{
  // factsheetRequest is run against AGVs the master knows nothing about
  // yet. Assigning before the AGV has reported any state must be allowed.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  // Note: NO handle_state injection.

  auto act = ActionFactory::build_custom("factsheetRequest", "act-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::ASSIGNED);
}

// =============================================================================
// action_id uniqueness checks
// =============================================================================

TEST_F(MasterAssignInstantActionsTest, EmptyActionId_Returns_DuplicateActionId)
{
  inject_online_and_state();

  auto act = ActionFactory::build_custom("stateRequest", "");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));

  EXPECT_EQ(res.decision, InstantActionDecision::DUPLICATE_ACTION_ID);
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_NE(
    res.errors.front().error_description->find("empty action_id"),
    std::string::npos)
    << "error message should call out empty action_id";
}

TEST_F(
  MasterAssignInstantActionsTest,
  DuplicateActionIdInBatch_Returns_DuplicateActionId)
{
  inject_online_and_state();

  auto a1 = ActionFactory::build_custom("stateRequest", "act-DUP");
  auto a2 = ActionFactory::build_custom("factsheetRequest", "act-DUP");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({a1, a2}));

  EXPECT_EQ(res.decision, InstantActionDecision::DUPLICATE_ACTION_ID);
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_NE(
    res.errors.front().error_description->find("act-DUP"), std::string::npos);
}

TEST_F(
  MasterAssignInstantActionsTest,
  ActionIdInFlightInState_Returns_DuplicateActionId)
{
  // Inject a state with an in-flight action_state[].action_id that the
  // candidate is about to reuse — must be rejected as duplicate.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());

  auto state = make_ready_state();
  vda5050_core::types::ActionState as;
  as.action_id = "act-INFLIGHT";
  as.action_type = "pick";
  as.action_status = vda5050_core::types::ActionStatus::RUNNING;
  state.action_states = {as};
  agv->handle_state(state);

  auto candidate = ActionFactory::build_custom("cancelOrder", "act-INFLIGHT");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({candidate}));

  EXPECT_EQ(res.decision, InstantActionDecision::DUPLICATE_ACTION_ID);
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_NE(
    res.errors.front().error_description->find("act-INFLIGHT"),
    std::string::npos);
}

TEST_F(
  MasterAssignInstantActionsTest, GeneratedUniqueActionId_PassesUniquenessCheck)
{
  inject_online_and_state();

  // The factory's UUID generator must produce IDs that pass uniqueness
  // (smoke test that combination works end-to-end).
  auto a1 = ActionFactory::build_custom(
    "stateRequest", ActionFactory::generate_action_id());
  auto a2 = ActionFactory::build_custom(
    "factsheetRequest", ActionFactory::generate_action_id());
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({a1, a2}));

  EXPECT_EQ(res.decision, InstantActionDecision::ASSIGNED);
}

// =============================================================================
// Action conflict checks (Task #22)
// =============================================================================

TEST_F(
  MasterAssignInstantActionsTest,
  HardActionWhileActiveAction_Returns_HardActionBlocked)
{
  // Inject a state that has a RUNNING action; candidate is HARD.
  // §6.12 mandates HARD must not be parallel — sync path should reject.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  auto state = make_ready_state();
  vda5050_core::types::ActionState as;
  as.action_id = "running-act";
  as.action_status = vda5050_core::types::ActionStatus::RUNNING;
  state.action_states = {as};
  agv->handle_state(state);

  auto candidate = ActionFactory::build_custom(
    "hardAction", "act-1", vda5050_core::types::BlockingType::HARD);
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({candidate}));

  EXPECT_EQ(res.decision, InstantActionDecision::HARD_ACTION_BLOCKED);
  ASSERT_FALSE(res.errors.empty());
}

TEST_F(
  MasterAssignInstantActionsTest,
  SoftActionWhileDriving_Returns_ActionBlockedByDriving)
{
  // Inject a state with driving=true; candidate is SOFT.
  // §6.12 mandates SOFT must not drive — sync path should reject.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  auto state = make_ready_state();
  state.driving = true;
  agv->handle_state(state);

  auto candidate = ActionFactory::build_custom(
    "softAction", "act-1", vda5050_core::types::BlockingType::SOFT);
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({candidate}));

  EXPECT_EQ(res.decision, InstantActionDecision::ACTION_BLOCKED_BY_DRIVING);
}

TEST_F(
  MasterAssignInstantActionsTest,
  HardActionWhileDriving_Returns_ActionBlockedByDriving)
{
  // Driving rule fires before the active-action rule when both apply
  // (HARD candidate with driving=true should classify as
  // ACTION_BLOCKED_BY_DRIVING, not HARD_ACTION_BLOCKED).
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  auto state = make_ready_state();
  state.driving = true;
  agv->handle_state(state);

  auto candidate = ActionFactory::build_custom(
    "hardAction", "act-1", vda5050_core::types::BlockingType::HARD);
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({candidate}));

  EXPECT_EQ(res.decision, InstantActionDecision::ACTION_BLOCKED_BY_DRIVING);
}

TEST_F(
  MasterAssignInstantActionsTest,
  ActionIdInActiveOrderNode_Returns_DuplicateActionId)
{
  // The action_id uniqueness check must scan the active order's
  // node/edge actions (not just state.action_states[]). This test
  // covers the active_order_snapshot() branch of the iteration.
  inject_online_and_state();

  // Publish an order whose first node carries an action with id "ORDER-N0-A".
  auto order = make_order_with_node_action("ORDER-N0-A");
  auto order_res = master_->assign_order(kManufacturer, kSerial, order);
  ASSERT_EQ(order_res.decision, AssignmentDecision::ASSIGNED);

  // Wait for the queue thread to publish + record_published to populate
  // active_order_snapshot.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  ASSERT_TRUE(wait_for(
    [&] { return agv->has_active_order(); }, std::chrono::milliseconds(500)))
    << "active order never adopted; lifecycle didn't see record_published";

  // Now try to send an instant action that reuses the order's action_id.
  auto candidate = ActionFactory::build_custom("stateRequest", "ORDER-N0-A");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({candidate}));

  EXPECT_EQ(res.decision, InstantActionDecision::DUPLICATE_ACTION_ID);
  ASSERT_FALSE(res.errors.empty());
  ASSERT_TRUE(res.errors.front().error_description.has_value());
  EXPECT_NE(
    res.errors.front().error_description->find("ORDER-N0-A"),
    std::string::npos);
}

TEST_F(MasterAssignInstantActionsTest, MultipleValidActions_AllAssigned)
{
  // Happy-path batch: send 3 unique-id NONE-blocking actions; all should
  // pass uniqueness + conflict checks and queue as one batch.
  inject_online_and_state();

  auto a1 = ActionFactory::build_state_request("req-a");
  auto a2 = ActionFactory::build_factsheet_request("req-b");
  auto a3 = ActionFactory::build_custom(
    "customAction", "req-c", vda5050_core::types::BlockingType::NONE);

  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({a1, a2, a3}));

  EXPECT_EQ(res.decision, InstantActionDecision::ASSIGNED);
  EXPECT_TRUE(res.errors.empty());
}

// =============================================================================
// Async chain regression: instant actions must REACH the wire (publish call)
// even in degraded states that would block an order. Guards against
// PreSend-validator-rejecting-instant-actions in the async path.
// =============================================================================

class MasterInstantActionsPublishesInDegradedTest : public ::testing::Test
{
protected:
  std::shared_ptr<MockMqttClient> mock_;
  std::shared_ptr<VDA5050Master> master_;
  std::atomic<int> instant_publishes_{0};

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

    // Count publishes whose topic ends with "/instantActions" — the
    // signal that the action actually reached the wire.
    EXPECT_CALL(
      *mock_, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .Times(::testing::AnyNumber())
      .WillRepeatedly(
        [this](const std::string& topic, const std::string&, int, bool) {
          if (topic.find("/instantActions") != std::string::npos)
          {
            instant_publishes_.fetch_add(1);
          }
        });

    master_ = std::make_shared<VDA5050Master>(mock_);
    master_->set_graph(make_test_graph());
    master_->onboard_agv(kManufacturer, kSerial);
  }
};

TEST_F(
  MasterInstantActionsPublishesInDegradedTest,
  ManualMode_InstantActionStillReachesWire)
{
  // operating_mode=MANUAL: assign_instant_actions returns ASSIGNED, and
  // the async chain MUST also let the action through to mqtt.publish.
  // Before this fix, pre_send_validator silently dropped these.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  agv->handle_state(
    make_ready_state(vda5050_core::types::OperatingMode::MANUAL));

  auto act = ActionFactory::build_state_request("req-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));
  ASSERT_EQ(res.decision, InstantActionDecision::ASSIGNED);

  EXPECT_TRUE(wait_for(
    [&] { return instant_publishes_.load() >= 1; },
    std::chrono::milliseconds(500)))
    << "instant action never reached mqtt.publish in MANUAL mode";
}

TEST_F(
  MasterInstantActionsPublishesInDegradedTest,
  PositionNotInitialized_InstantActionStillReachesWire)
{
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  agv->handle_state(make_ready_state(
    vda5050_core::types::OperatingMode::AUTOMATIC,
    /*position_initialized=*/false));

  auto act = ActionFactory::build_custom(
    "initPosition", "req-1", vda5050_core::types::BlockingType::NONE);
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));
  ASSERT_EQ(res.decision, InstantActionDecision::ASSIGNED);

  EXPECT_TRUE(wait_for(
    [&] { return instant_publishes_.load() >= 1; },
    std::chrono::milliseconds(500)))
    << "initPosition never reached mqtt.publish with "
       "position_initialized=false";
}

TEST_F(
  MasterInstantActionsPublishesInDegradedTest,
  NoStateYet_InstantActionStillReachesWire)
{
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  // No handle_state — factsheetRequest valid use case.

  auto act = ActionFactory::build_factsheet_request("req-1");
  auto res =
    master_->assign_instant_actions(kManufacturer, kSerial, wrap({act}));
  ASSERT_EQ(res.decision, InstantActionDecision::ASSIGNED);

  EXPECT_TRUE(wait_for(
    [&] { return instant_publishes_.load() >= 1; },
    std::chrono::milliseconds(500)))
    << "factsheetRequest never reached mqtt.publish without prior state";
}

}  // namespace

}  // namespace vda5050_core::master::test
