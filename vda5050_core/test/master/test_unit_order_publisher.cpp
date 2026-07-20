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

// Unit tests for the OrderPublisher validator chain (gmock MockMqttClient,
// no broker).

#include <gmock/gmock.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/layout/lif.hpp"
#include "vda5050_core/master/order/order_publisher.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/validation/pre_send_validator.hpp"

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

// Layout with the node/edge ids used across these fixtures (N0-E0-N1-E1-N2
// plus a stitch extension N1-E2-N3), matched by traversability's integrity
// check.
vda5050_core::layout::Graph::ConstPtr make_minimal_graph()
{
  vda5050_core::layout::LIF lif;
  vda5050_core::layout::Layout layout;
  layout.layout_id = "L1";
  for (const auto& id : {"N0", "N1", "N2", "N3"})
  {
    vda5050_core::layout::Node n;
    n.node_id = id;
    n.map_id = "L1";
    n.node_position = {0.0, 0.0};
    n.vehicle_type_node_properties.push_back(
      {"v1", std::nullopt, std::nullopt});
    layout.nodes.push_back(n);
  }
  struct EdgeSpec
  {
    const char* id;
    const char* from;
    const char* to;
  };
  for (const auto& spec :
       {EdgeSpec{"E0", "N0", "N1"}, EdgeSpec{"E1", "N1", "N2"},
        EdgeSpec{"E2", "N1", "N3"}, EdgeSpec{"E3", "N2", "N3"}})
  {
    vda5050_core::layout::Edge e;
    e.edge_id = spec.id;
    e.start_node_id = spec.from;
    e.end_node_id = spec.to;
    vda5050_core::layout::VehicleTypeEdgeProperty p;
    p.vehicle_type_id = "v1";
    e.vehicle_type_edge_properties.push_back(p);
    layout.edges.push_back(e);
  }
  lif.layouts.push_back(std::move(layout));
  return vda5050_core::layout::Graph::from_lif(std::move(lif));
}

// Passes all pre-send readiness checks. last_node_id parks the AGV on a node so
// traversability's reachability passes (fixtures lack node_position).
validation::PreSendContext make_ready_context(
  const std::string& last_node_id = "")
{
  vda5050_core::types::State s;
  s.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  vda5050_core::types::AGVPosition pos;
  pos.position_initialized = true;
  s.agv_position = pos;
  s.last_node_id = last_node_id;
  return validation::PreSendContext{
    vda5050_core::types::ConnectionState::ONLINE, s,
    std::nullopt /* last_factsheet — graph step doesn't use it */,
    AGVState::AVAILABLE, make_minimal_graph()};
}

// Build a schema-valid header so the schema gate passes and the chain
// reaches the graph step.
void fill_schema_valid_header(vda5050_core::types::Header& h)
{
  h.version = "2.0.0";
  h.manufacturer = "ACME";
  h.serial_number = "AGV001";
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

// Active V0: released base [N0(0), N1(2)] + horizon [N2(4)] with edges.
vda5050_core::types::Order make_active_v0()
{
  vda5050_core::types::Order o;
  fill_schema_valid_header(o.header);
  o.order_id = "ORDER_A";
  o.order_update_id = 0;
  o.nodes = {
    mk_node("N0", 0, true), mk_node("N1", 2, true), mk_node("N2", 4, false)};
  o.edges = {
    mk_edge("E0", 1, "N0", "N1", true), mk_edge("E1", 3, "N1", "N2", false)};
  return o;
}

}  // namespace

// =============================================================================
// Each validator stage must short-circuit independently (regression guard
// against a refactor dropping a stage).
// =============================================================================

TEST(OrderPublisherTest, MalformedOrderRejectedAtSchema)
{
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;

  vda5050_core::types::Order malformed;
  // header.version intentionally left empty
  malformed.order_id = "ORDER_A";
  malformed.order_update_id = 0;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  malformed.nodes = {n};

  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(0);

  auto ctx = make_ready_context();
  auto result = publisher.publish(*adapter, ctx, malformed, std::nullopt);

  EXPECT_FALSE(static_cast<bool>(result));
  ASSERT_FALSE(result.fatal_errors().empty());
  bool found_schema_error = false;
  for (const auto& e : result.fatal_errors())
  {
    if (e.error_type == vda5050_core::errors::ContentValidationError)
    {
      found_schema_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_schema_error)
    << "expected ContentValidationError; got first error type: "
    << result.fatal_errors().front().error_type;
}

TEST(OrderPublisherTest, NotReadyAGVRejectedAtPreSend)
{
  // PreSendContext with OFFLINE connection → PreSend validator rejects.
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;

  vda5050_core::types::Order order;
  fill_schema_valid_header(order.header);
  order.order_id = "ORDER_A";
  order.order_update_id = 0;
  vda5050_core::types::Node n;
  n.node_id = "N0";
  n.sequence_id = 0;
  n.released = true;
  order.nodes = {n};

  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(0);

  // Override the ready context's connection status to OFFLINE.
  auto ctx = make_ready_context();
  ctx.connection_status = vda5050_core::types::ConnectionState::OFFLINE;

  auto result = publisher.publish(*adapter, ctx, order, std::nullopt);

  EXPECT_FALSE(static_cast<bool>(result));
  ASSERT_FALSE(result.fatal_errors().empty());
  bool found_pre_send_error = false;
  for (const auto& e : result.fatal_errors())
  {
    if (e.error_type == vda5050_core::errors::PreSendValidationError)
    {
      found_pre_send_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_pre_send_error)
    << "expected PreSendValidationError; got first error type: "
    << result.fatal_errors().front().error_type;
}

// =============================================================================
// Publisher branches on update vs new order: no/different active order takes
// the is_valid_graph path; same order_id takes combine_order (sparse seqs OK).
// =============================================================================
namespace {

// Stitched update U1: stitch anchor N1@2 then a sparse extension. Rejected by
// is_valid_graph standalone; accepted by combine_order.
vda5050_core::types::Order make_stitched_update_v1()
{
  vda5050_core::types::Order o;
  fill_schema_valid_header(o.header);
  o.order_id = "ORDER_A";
  o.order_update_id = 1;
  // Stitch node N1, release the horizon N2, extend to N3 from the tail so the
  // merged order is a valid graph.
  o.nodes = {
    mk_node("N1", 2, true), mk_node("N2", 4, true), mk_node("N3", 6, true)};
  o.edges = {
    mk_edge("E1", 3, "N1", "N2", true), mk_edge("E3", 5, "N2", "N3", true)};
  return o;
}

}  // namespace

TEST(OrderPublisherTest, FreshOrderNoActiveTakesGraphPath)
{
  // No active → is_valid_graph branch runs on the full V0 graph (which is
  // a valid standalone graph). Publish succeeds.
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  auto v0 = make_active_v0();

  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(1);

  auto ctx = make_ready_context("N0");  // active_order = nullopt
  auto result = publisher.publish(*adapter, ctx, v0, std::nullopt);
  EXPECT_TRUE(static_cast<bool>(result))
    << "errors=" << result.fatal_errors().size();
}

TEST(OrderPublisherTest, NoGraphLoadedSkipsGraphIntegrity)
{
  // No layout loaded: traversability skips graph-integrity, but a valid fresh
  // order still publishes (is_valid_graph + reachability still run).
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  auto v0 = make_active_v0();

  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(1);

  auto ctx = make_ready_context("N0");
  ctx.loaded_graph = nullptr;  // no layout loaded
  auto result = publisher.publish(*adapter, ctx, v0, std::nullopt);
  EXPECT_TRUE(static_cast<bool>(result))
    << "errors=" << result.fatal_errors().size();
}

TEST(OrderPublisherTest, StitchedUpdateValidatedViaCombineOrder)
{
  // Same order_id with an active order takes combine_order, which accepts the
  // sparse-seq candidate that is_valid_graph would reject standalone.
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;

  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(1);

  auto ctx = make_ready_context("N1");
  auto u1 = make_stitched_update_v1();
  auto result = publisher.publish(*adapter, ctx, u1, make_active_v0());
  EXPECT_TRUE(static_cast<bool>(result))
    << "errors=" << result.fatal_errors().size();
}

TEST(OrderPublisherTest, MergedGraphInvalidRejected)
{
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(0);

  // Combine succeeds but the merge is graph-invalid (E2 branches N1->N3 while
  // horizon N2 is kept) → publisher rejects rather than adopt it.
  auto u = make_stitched_update_v1();
  u.nodes = {mk_node("N1", 2, true), mk_node("N3", 6, true)};
  u.edges = {mk_edge("E2", 5, "N1", "N3", true)};
  auto result =
    publisher.publish(*adapter, make_ready_context("N1"), u, make_active_v0());
  EXPECT_FALSE(static_cast<bool>(result));
  ASSERT_FALSE(result.fatal_errors().empty());
  EXPECT_NE(
    result.fatal_errors().front().error_description.value_or("").find("graph"),
    std::string::npos)
    << "must reject on the merged-graph check, not an earlier one";
}

TEST(OrderPublisherTest, StitchedUpdateBackwardUpdateIdRejected)
{
  // combine_order rejects update_update_id <= active.order_update_id.
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(0);

  auto active = make_active_v0();
  active.order_update_id = 5;
  auto ctx = make_ready_context("N1");

  auto candidate = make_stitched_update_v1();
  candidate.order_update_id = 3;  // backward
  auto result = publisher.publish(*adapter, ctx, candidate, active);
  EXPECT_FALSE(static_cast<bool>(result));
}

TEST(OrderPublisherTest, OrderExceedingFactsheetNodeLimitRejected)
{
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(0);

  vda5050_core::types::Factsheet fs;
  fs.protocol_limits.max_array_lens.order_nodes = 1;

  auto ctx = make_ready_context("N0");
  ctx.last_factsheet = fs;

  // 3 nodes against a limit of 1.
  auto order = make_active_v0();
  auto result = publisher.publish(*adapter, ctx, order, std::nullopt);

  EXPECT_FALSE(static_cast<bool>(result));
  ASSERT_EQ(result.fatal_errors().size(), 1u);
  EXPECT_EQ(
    result.fatal_errors().front().error_type,
    vda5050_core::errors::ProtocolLimitError);
}

TEST(OrderPublisherTest, StitchedUpdateUnderLimitButMergedOrderOverIsRejected)
{
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(0);

  // Base keeps a two-node horizon, so the merge preserves N3 on top of the
  // update and the merged order outgrows the fragment.
  vda5050_core::types::Order base;
  fill_schema_valid_header(base.header);
  base.order_id = "ORDER_A";
  base.order_update_id = 0;
  base.nodes = {
    mk_node("N0", 0, true), mk_node("N1", 2, true), mk_node("N2", 4, false),
    mk_node("N3", 6, false)};
  base.edges = {
    mk_edge("E0", 1, "N0", "N1", true), mk_edge("E1", 3, "N1", "N2", false),
    mk_edge("E2", 5, "N2", "N3", false)};

  vda5050_core::types::Order update;
  fill_schema_valid_header(update.header);
  update.order_id = "ORDER_A";
  update.order_update_id = 1;
  update.nodes = {mk_node("N1", 2, true), mk_node("N2", 4, true)};
  update.edges = {mk_edge("E1", 3, "N1", "N2", true)};

  vda5050_core::types::Factsheet fs;
  fs.protocol_limits.max_array_lens.order_nodes = 2;

  auto ctx = make_ready_context("N1");
  ctx.last_factsheet = fs;

  auto result = publisher.publish(*adapter, ctx, update, base);

  EXPECT_FALSE(static_cast<bool>(result));
  ASSERT_EQ(result.fatal_errors().size(), 1u);
  EXPECT_EQ(
    result.fatal_errors().front().error_type,
    vda5050_core::errors::ProtocolLimitError)
    << "merged order must be the subject of the limits check";
}

TEST(OrderPublisherTest, DifferentOrderIdTakesGraphPath)
{
  // active.order_id != candidate.order_id → graph path (treated as new
  // order). The candidate is structurally valid as a standalone graph.
  auto mock = std::make_shared<MockMqttClient>();
  ON_CALL(*mock, connected()).WillByDefault(::testing::Return(true));
  auto adapter = vda5050_core::execution::ProtocolAdapter::make(
    mock, "uagv", "2.0.0", "ACME", "AGV001");
  vda5050_core::master::OrderPublisher publisher;
  EXPECT_CALL(
    *mock, publish(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(1);

  auto active = make_active_v0();
  active.order_id = "OTHER_ORDER";  // different id → graph path
  // AGV parked on N0 — traversability reachability check passes for V0.
  auto ctx = make_ready_context("N0");

  // For graph path, candidate must be a valid standalone graph (V0 is).
  auto v0_as_new = make_active_v0();
  auto result = publisher.publish(*adapter, ctx, v0_as_new, active);
  EXPECT_TRUE(static_cast<bool>(result))
    << "errors=" << result.fatal_errors().size();
}

}  // namespace vda5050_core::master::test
