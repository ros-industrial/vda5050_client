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

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "vda5050_core/master/agv.hpp"
#include "vda5050_master_ros2/order_status_builder.hpp"

namespace vda5050_master_ros2 {
namespace test {

namespace {

constexpr const char* kMfg = "ACME";
constexpr const char* kSerial = "AGV01";

using OrderStatus = vda5050_master_ros2::msg::OrderStatus;

vda5050_core::types::State make_state_with_order(
  const std::string& order_id, uint32_t order_update_id, uint32_t last_node_seq,
  const std::string& last_node_id = "")
{
  vda5050_core::types::State s;
  s.header.version = "2.0.0";
  s.header.manufacturer = kMfg;
  s.header.serial_number = kSerial;
  s.order_id = order_id;
  s.order_update_id = order_update_id;
  s.last_node_id = last_node_id;
  s.last_node_sequence_id = last_node_seq;
  s.driving = false;
  s.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  return s;
}

vda5050_core::types::Node make_node(
  uint32_t seq, const std::string& id, bool released)
{
  vda5050_core::types::Node n;
  n.node_id = id;
  n.sequence_id = seq;
  n.released = released;
  return n;
}

vda5050_core::master::AGV::OrderStatusBundle make_bundle_no_active(
  std::optional<vda5050_core::types::State> state)
{
  vda5050_core::master::AGV::OrderStatusBundle b;
  b.state = std::move(state);
  if (b.state.has_value())
  {
    b.state_received_at = std::chrono::system_clock::now();
  }
  b.active_order_snapshot.has_active = false;
  b.pending_stitch_count = 0;
  return b;
}

vda5050_core::master::AGV::OrderStatusBundle make_bundle_active(
  vda5050_core::types::State state, const std::string& order_id,
  uint32_t order_update_id, std::vector<vda5050_core::types::Node> nodes,
  bool order_complete = false)
{
  vda5050_core::master::AGV::OrderStatusBundle b;
  b.state = std::move(state);
  b.state_received_at = std::chrono::system_clock::now();
  b.active_order_snapshot.has_active = true;
  b.active_order_snapshot.order_id = order_id;
  b.active_order_snapshot.order_update_id = order_update_id;
  b.active_order_snapshot.nodes = std::move(nodes);
  b.active_order_snapshot.order_complete = order_complete;
  b.pending_stitch_count = 0;
  return b;
}

}  // namespace

// =============================================================================
// Phase derivation
// =============================================================================

TEST(OrderStatusBuilderTest, EmptyState_NoActiveOrder_PhaseNoOrder)
{
  vda5050_core::master::AGV::OrderStatusBundle b;
  b.active_order_snapshot.has_active = false;
  b.pending_stitch_count = 0;

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_NO_ORDER);
  EXPECT_EQ(msg.manufacturer, kMfg);
  EXPECT_EQ(msg.serial_number, kSerial);
  EXPECT_EQ(msg.order_id, "");
  EXPECT_EQ(msg.order_update_id, 0u);
  EXPECT_EQ(msg.nodes_in_base, 0u);
  EXPECT_EQ(msg.nodes_in_horizon, 0u);
  EXPECT_FALSE(msg.driving);
  EXPECT_FALSE(msg.paused);
  EXPECT_FALSE(msg.new_base_request);
  EXPECT_TRUE(msg.action_states.empty());
  EXPECT_TRUE(msg.errors.empty());
}

TEST(OrderStatusBuilderTest, StateButNoActiveOrder_PhaseNoOrder)
{
  // AGV reported State but master is not tracking an order (e.g., AGV
  // reported empty order_id post-reset). Should be SUCCESS-friendly
  // but phase=NO_ORDER.
  auto state = make_state_with_order("", 0, 0);
  auto b = make_bundle_no_active(state);

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_NO_ORDER);
  // State fields echoed.
  EXPECT_EQ(msg.last_node_id, "");
  EXPECT_EQ(msg.last_node_sequence_id, 0u);
  EXPECT_FALSE(msg.driving);
}

TEST(OrderStatusBuilderTest, ActiveAccepted_LastNodeSeqZero)
{
  auto state = make_state_with_order("ORD1", 0, /*last_node_seq=*/0);
  auto b = make_bundle_active(
    state, "ORD1", 0, {make_node(0, "N0", true), make_node(1, "N1", true)});

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_ACCEPTED);
  EXPECT_EQ(msg.order_id, "ORD1");
  EXPECT_EQ(msg.order_update_id, 0u);
  EXPECT_EQ(msg.nodes_in_base, 2u);
  EXPECT_EQ(msg.nodes_in_horizon, 0u);
}

TEST(OrderStatusBuilderTest, ActiveRunning_LastNodeSeqAdvanced)
{
  auto state = make_state_with_order("ORD1", 0, /*last_node_seq=*/2, "N2");
  state.driving = true;
  auto b = make_bundle_active(
    state, "ORD1", 0,
    {make_node(0, "N0", true), make_node(2, "N2", true),
     make_node(4, "N4", true)});

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_RUNNING);
  EXPECT_EQ(msg.last_node_id, "N2");
  EXPECT_EQ(msg.last_node_sequence_id, 2u);
  EXPECT_TRUE(msg.driving);
}

TEST(OrderStatusBuilderTest, ActiveOrderComplete_PhaseCompleted)
{
  auto state = make_state_with_order("ORD1", 0, /*last_node_seq=*/4, "N4");
  auto b = make_bundle_active(
    state, "ORD1", 0,
    {make_node(0, "N0", true), make_node(2, "N2", true),
     make_node(4, "N4", true)},
    /*order_complete=*/true);

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_COMPLETED);
}

TEST(OrderStatusBuilderTest, ActiveWithFatalError_PhaseFailed)
{
  auto state = make_state_with_order("ORD1", 0, /*last_node_seq=*/2);
  vda5050_core::types::Error err;
  err.error_type = "BatteryDepleted";
  err.error_level = vda5050_core::types::ErrorLevel::FATAL;
  state.errors.push_back(err);
  auto b = make_bundle_active(state, "ORD1", 0, {make_node(0, "N0", true)});

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_FAILED);
  ASSERT_EQ(msg.errors.size(), 1u);
  EXPECT_EQ(msg.errors[0].error_type, "BatteryDepleted");
}

// =============================================================================
// Field mapping detail
// =============================================================================

TEST(
  OrderStatusBuilderTest, BaseAndHorizon_StableCounts_ActionStatesCopiedThrough)
{
  auto state = make_state_with_order("ORD1", 0, /*last_node_seq=*/1, "N1");
  state.paused = true;
  state.new_base_request = true;
  // Two action states — both should be copied through (no filtering).
  vda5050_core::types::ActionState as1;
  as1.action_id = "A1";
  as1.action_status = vda5050_core::types::ActionStatus::FINISHED;
  vda5050_core::types::ActionState as2;
  as2.action_id = "A2";
  as2.action_status = vda5050_core::types::ActionStatus::RUNNING;
  state.action_states = {as1, as2};

  // 3 base + 2 horizon nodes.
  auto b = make_bundle_active(
    state, "ORD1", 0,
    {make_node(0, "N0", true), make_node(1, "N1", true),
     make_node(2, "N2", true), make_node(3, "N3", false),
     make_node(4, "N4", false)});
  b.pending_stitch_count = 2;

  auto msg = build_order_status_msg(b, kMfg, kSerial);

  EXPECT_EQ(msg.phase, OrderStatus::PHASE_RUNNING);
  EXPECT_EQ(msg.nodes_in_base, 3u);
  EXPECT_EQ(msg.nodes_in_horizon, 2u);
  EXPECT_EQ(msg.pending_stitch_count, 2u);
  EXPECT_TRUE(msg.paused);
  EXPECT_TRUE(msg.new_base_request);
  ASSERT_EQ(msg.action_states.size(), 2u);
  EXPECT_EQ(msg.action_states[0].action_id, "A1");
  EXPECT_EQ(msg.action_states[1].action_id, "A2");
}

}  // namespace test
}  // namespace vda5050_master_ros2
