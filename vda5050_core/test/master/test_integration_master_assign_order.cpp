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
 * @file master_assign_order_test.cpp
 * @brief Integration tests for VDA5050Master::assign_order (Task #15).
 *
 * Verifies that the synchronous pre-flight readiness checks
 * (onboarding, connection, operational state, mode, position
 * initialization, stitch pre-flight) return the right
 * AssignmentDecision and rich error feedback.
 *
 * Uses gmock MockMqttClient so tests run without a broker. AGV state
 * is injected by calling AGV::handle_connection / handle_state
 * directly, mirroring what the MQTT subscriber would do.
 */

#include <gmock/gmock.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/layout/lif.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/agv_position.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/operating_mode.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master::test {

namespace {

constexpr const char* kManufacturer = "ACME";
constexpr const char* kSerial = "AGV001";
constexpr const char* kOrderId = "ORDER_A";

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
  bool position_initialized = true, const std::string& last_node_id = "N0",
  uint32_t last_node_seq = 0, const std::string& order_id = "",
  uint32_t order_update_id = 0)
{
  vda5050_core::types::State s;
  s.header.header_id = 1;
  s.header.timestamp = std::chrono::system_clock::now();
  s.header.version = "2.0.0";
  s.header.manufacturer = kManufacturer;
  s.header.serial_number = kSerial;
  s.order_id = order_id;
  s.order_update_id = order_update_id;
  s.last_node_id = last_node_id;
  s.last_node_sequence_id = last_node_seq;
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

vda5050_core::types::Node mk_node(
  const std::string& id, uint32_t seq, bool released)
{
  vda5050_core::types::Node n;
  n.node_id = id;
  n.sequence_id = seq;
  n.released = released;
  return n;
}

vda5050_core::types::Edge mk_edge(
  const std::string& id, uint32_t seq, const std::string& from,
  const std::string& to, bool released)
{
  vda5050_core::types::Edge e;
  e.edge_id = id;
  e.sequence_id = seq;
  e.start_node_id = from;
  e.end_node_id = to;
  e.released = released;
  return e;
}

// V0 = N0@0(released) -> E0@1 -> N1@2(released) -> E1@3 -> N2@4(horizon).
// last released base = N1@2 → stitch point seq=2.
vda5050_core::types::Order make_multi_node_v0()
{
  vda5050_core::types::Order o;
  o.header.header_id = 1;
  o.header.timestamp = std::chrono::system_clock::now();
  o.header.version = "2.0.0";
  o.header.manufacturer = kManufacturer;
  o.header.serial_number = kSerial;
  o.order_id = kOrderId;
  o.order_update_id = 0;
  o.nodes = {
    mk_node("N0", 0, true), mk_node("N1", 2, true), mk_node("N2", 4, false)};
  o.edges = {
    mk_edge("E0", 1, "N0", "N1", true), mk_edge("E1", 3, "N1", "N2", false)};
  return o;
}

// Poll a predicate until true or timeout. Returns true if predicate passed.
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

vda5050_core::types::Order make_minimal_order(uint32_t update_id = 0)
{
  vda5050_core::types::Order o;
  o.header.header_id = 1;
  o.header.timestamp = std::chrono::system_clock::now();
  o.header.version = "2.0.0";
  o.header.manufacturer = kManufacturer;
  o.header.serial_number = kSerial;
  o.order_id = kOrderId;
  o.order_update_id = update_id;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  o.nodes = {n};
  return o;
}

class MasterAssignOrderTest : public ::testing::Test
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

  // Build the topology that the test orders run against. Mirrors
  // make_v0() / make_active_v0() — nodes N0/N1/N2, edges E0/E1.
  static vda5050_core::layout::Graph::ConstPtr make_test_graph()
  {
    vda5050_core::layout::LIF lif;
    vda5050_core::layout::Layout layout;
    layout.layout_id = "L1";
    for (const auto& id : {"N0", "N1", "N2"})
    {
      vda5050_core::layout::Node n;
      n.node_id = id;
      n.map_id = "L1";
      n.node_position = {0.0, 0.0};
      n.vehicle_type_node_properties.push_back(
        {"v1", std::nullopt, std::nullopt});
      layout.nodes.push_back(n);
    }
    for (auto pair :
         {std::make_pair("E0", std::make_pair("N0", "N1")),
          std::make_pair("E1", std::make_pair("N1", "N2"))})
    {
      vda5050_core::layout::Edge e;
      e.edge_id = pair.first;
      e.start_node_id = pair.second.first;
      e.end_node_id = pair.second.second;
      vda5050_core::layout::VehicleTypeEdgeProperty p;
      p.vehicle_type_id = "v1";
      e.vehicle_type_edge_properties.push_back(p);
      layout.edges.push_back(e);
    }
    lif.layouts.push_back(std::move(layout));
    return vda5050_core::layout::Graph::from_lif(std::move(lif));
  }

  // Drive the AGV into a "ready to receive orders" state by injecting
  // an ONLINE connection and an AUTOMATIC State with position
  // initialized.
  void make_agv_ready()
  {
    auto agv = master_->get_agv(kManufacturer, kSerial);
    ASSERT_NE(agv, nullptr);
    agv->handle_connection(make_online_connection());
    agv->handle_state(make_ready_state());
  }
};

}  // namespace

// =============================================================================
// AGV onboarded / not-onboarded
// =============================================================================
TEST_F(MasterAssignOrderTest, AgvNotOnboarded_Returns_AgvNotOnboarded)
{
  auto res =
    master_->assign_order("OTHER_MFG", "OTHER_SN", make_minimal_order());
  EXPECT_EQ(res.decision, AssignmentDecision::AGV_NOT_ONBOARDED);
  EXPECT_FALSE(static_cast<bool>(res));
  EXPECT_FALSE(res.errors.empty());
}

// =============================================================================
// Connection / operational-state guards
// =============================================================================
TEST_F(MasterAssignOrderTest, AgvOffline_Returns_AgvOffline)
{
  // No connection injected — connection_status defaults to OFFLINE.
  auto res =
    master_->assign_order(kManufacturer, kSerial, make_minimal_order());
  EXPECT_EQ(res.decision, AssignmentDecision::AGV_OFFLINE);
}

TEST_F(MasterAssignOrderTest, NoStateYet_Returns_AgvNoStateYet)
{
  // ONLINE connection but no State message yet.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());

  auto res =
    master_->assign_order(kManufacturer, kSerial, make_minimal_order());
  // Operational state is still STATE_UNKNOWN until a State arrives, so
  // this surfaces as AGV_NOT_READY (operational state check fires first).
  EXPECT_EQ(res.decision, AssignmentDecision::AGV_NOT_READY);
}

// =============================================================================
// Mode / position
// =============================================================================
TEST_F(MasterAssignOrderTest, ManualMode_Returns_AgvModeNotAuto)
{
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  agv->handle_state(
    make_ready_state(vda5050_core::types::OperatingMode::MANUAL));

  auto res =
    master_->assign_order(kManufacturer, kSerial, make_minimal_order());
  EXPECT_EQ(res.decision, AssignmentDecision::AGV_MODE_NOT_AUTO);
}

TEST_F(MasterAssignOrderTest, PositionNotInitialized_Returns_PosNotInit)
{
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  agv->handle_connection(make_online_connection());
  agv->handle_state(make_ready_state(
    vda5050_core::types::OperatingMode::AUTOMATIC,
    /*position_initialized=*/false));

  auto res =
    master_->assign_order(kManufacturer, kSerial, make_minimal_order());
  EXPECT_EQ(res.decision, AssignmentDecision::AGV_POSITION_NOT_INITIALIZED);
}

// =============================================================================
// Happy path
// =============================================================================
TEST_F(MasterAssignOrderTest, HappyPath_ReturnsAssigned_AndQueues)
{
  make_agv_ready();
  auto agv = master_->get_agv(kManufacturer, kSerial);

  EXPECT_EQ(agv->get_pending_order_count(), 0u);
  auto res =
    master_->assign_order(kManufacturer, kSerial, make_minimal_order());
  EXPECT_EQ(res.decision, AssignmentDecision::ASSIGNED);
  EXPECT_TRUE(static_cast<bool>(res));
  EXPECT_TRUE(res.errors.empty());

  // The queue thread runs concurrently and may already have drained
  // the order by the time we assert. Eventual consistency: the order
  // is either still queued OR has been published (record_published
  // ran). Either way the FMS got "ASSIGNED" feedback. We assert the
  // visible end-state: lifecycle records the published order.
  ASSERT_TRUE(wait_for(
    [&] { return agv->has_active_order(); }, std::chrono::milliseconds(500)))
    << "queue thread did not publish + record within 500ms";
}

// =============================================================================
// End-to-end + stitcher branch coverage
// =============================================================================
TEST_F(MasterAssignOrderTest, EndToEnd_DrainPublishRecord_Cycle)
{
  // Verify the full async path: assign_order → queue thread → publisher
  // chain → adapter.publish → record_published advances active.
  make_agv_ready();
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);
  ASSERT_FALSE(agv->has_active_order());

  auto v0 = make_minimal_order(0);
  auto res = master_->assign_order(kManufacturer, kSerial, v0);
  ASSERT_EQ(res.decision, AssignmentDecision::ASSIGNED);

  // Wait for queue thread to publish + record. has_active_order() flips
  // true only after AGV::publish_order completes record_published.
  ASSERT_TRUE(wait_for(
    [&] { return agv->has_active_order(); }, std::chrono::milliseconds(500)))
    << "queue thread did not advance active within 500ms";

  EXPECT_EQ(agv->active_order_id().value_or(""), kOrderId);
  EXPECT_EQ(agv->active_order_update_id().value_or(99), 0u);
}

TEST_F(MasterAssignOrderTest, AssignOrder_StitchedUpdate_Queued)
{
  // V0 is multi-node; AGV at N0 (not at stitch anchor N1@2). U1 must
  // QUEUE_PENDING because cond 3 (AGV reached stitch) fails.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);

  // Drive AGV ready, parked at N0.
  agv->handle_connection(make_online_connection());
  agv->handle_state(make_ready_state(
    vda5050_core::types::OperatingMode::AUTOMATIC, true, "N0", 0));

  // Publish V0 and wait for adoption.
  auto res0 =
    master_->assign_order(kManufacturer, kSerial, make_multi_node_v0());
  ASSERT_EQ(res0.decision, AssignmentDecision::ASSIGNED);
  ASSERT_TRUE(wait_for(
    [&] { return agv->has_active_order(); }, std::chrono::milliseconds(500)));

  // Simulate AGV acking V0 by sending a state with order_id=ORDER_A.
  // Without this, snapshot.state_order_id is empty and stitcher's
  // pre-flight returns REJECT instead of evaluating cond 3.
  agv->handle_state(make_ready_state(
    vda5050_core::types::OperatingMode::AUTOMATIC, true, "N0", 0,
    /*order_id=*/std::string(kOrderId), /*order_update_id=*/0));

  // U1 is a stitched update with first node at the stitch anchor (N1@2).
  vda5050_core::types::Order u1;
  u1.header.header_id = 1;
  u1.header.timestamp = std::chrono::system_clock::now();
  u1.header.version = "2.0.0";
  u1.header.manufacturer = kManufacturer;
  u1.header.serial_number = kSerial;
  u1.order_id = kOrderId;
  u1.order_update_id = 1;
  u1.nodes = {mk_node("N1", 2, true), mk_node("N3", 6, true)};
  u1.edges = {mk_edge("E2", 5, "N1", "N3", true)};

  // AGV is still at N0 — stitcher cond 3 (reached stitch) fails.
  auto res = master_->assign_order(kManufacturer, kSerial, u1);
  EXPECT_EQ(res.decision, AssignmentDecision::STITCH_QUEUED)
    << "expected STITCH_QUEUED when AGV is at N0 but stitch is N1";
}

TEST_F(MasterAssignOrderTest, AssignOrder_StitchedUpdate_Rejected)
{
  // V0 active. Send a candidate with backward order_update_id (same
  // order_id, lower id) → stitcher pre-flight returns REJECT.
  auto agv = master_->get_agv(kManufacturer, kSerial);
  ASSERT_NE(agv, nullptr);

  agv->handle_connection(make_online_connection());
  agv->handle_state(make_ready_state(
    vda5050_core::types::OperatingMode::AUTOMATIC, true, "N0", 0));

  auto v0 = make_multi_node_v0();
  v0.order_update_id = 5;  // simulate an already-active update
  auto res0 = master_->assign_order(kManufacturer, kSerial, v0);
  ASSERT_EQ(res0.decision, AssignmentDecision::ASSIGNED);
  ASSERT_TRUE(wait_for(
    [&] {
      auto u = agv->active_order_update_id();
      return u.has_value() && *u == 5u;
    },
    std::chrono::milliseconds(500)));

  // Backward update (same id, lower update_id) → stitcher rejects.
  auto bad = make_multi_node_v0();
  bad.order_update_id = 3;
  auto res = master_->assign_order(kManufacturer, kSerial, bad);
  EXPECT_EQ(res.decision, AssignmentDecision::STITCH_REJECTED);
  EXPECT_FALSE(res.errors.empty());
}

// =============================================================================
// Returned errors carry diagnostic descriptions
// =============================================================================
TEST_F(MasterAssignOrderTest, RejectionErrorsAreDescriptive)
{
  // Drive AGV into a rejection (no connection injected → OFFLINE).
  auto res =
    master_->assign_order(kManufacturer, kSerial, make_minimal_order());
  ASSERT_EQ(res.decision, AssignmentDecision::AGV_OFFLINE);
  ASSERT_FALSE(res.errors.empty());
  bool found_online = false;
  for (const auto& e : res.errors)
  {
    if (
      e.error_description &&
      e.error_description->find("ONLINE") != std::string::npos)
    {
      found_online = true;
      break;
    }
  }
  EXPECT_TRUE(found_online) << "expected error description mentioning 'ONLINE'";
}

}  // namespace vda5050_core::master::test
