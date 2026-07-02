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

#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/validation/order_graph_validator.hpp"

class OrderValidationTest : public testing::Test
{
protected:
  vda5050_core::types::Order create_order(
    const std::string& order_id, uint32_t order_update_id)
  {
    return vda5050_core::types::Order{vda5050_core::types::Header{},
                                      order_id,
                                      order_update_id,
                                      {},
                                      {},
                                      std::nullopt};
  }

  vda5050_core::types::Node create_node(
    const std::string& node_id, uint32_t sequence_id, bool released)
  {
    return vda5050_core::types::Node{node_id, sequence_id,  released,
                                     {},      std::nullopt, std::nullopt};
  }

  vda5050_core::types::Edge create_edge(
    const std::string& edge_id, uint32_t sequence_id,
    const std::string& start_node_id, const std::string& end_node_id,
    bool released)
  {
    return vda5050_core::types::Edge{
      edge_id,      sequence_id,  start_node_id, end_node_id,  released,
      {},           std::nullopt, std::nullopt,  std::nullopt, std::nullopt,
      std::nullopt, std::nullopt, std::nullopt,  std::nullopt, std::nullopt,
      std::nullopt, std::nullopt};
  }
};

TEST_F(OrderValidationTest, EmptyNodeList)
{
  auto order = create_order("order_0", 0);
  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(), "Order contains no nodes.");
}

TEST_F(OrderValidationTest, UnequalNodesEdges)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, true),
    create_node("node_2", 4, true)};

  order.edges = {create_edge("edge_0", 1, "node_0", "node_1", true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Graph mismatch: Order contains 3 node(s) but 1 edge(s).");
}

TEST_F(OrderValidationTest, FirstSequenceOfUpdate)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 2, true), create_node("node_1", 4, true),
    create_node("node_2", 6, true)};

  order.edges = {
    create_edge("edge_0", 3, "node_0", "node_1", true),
    create_edge("edge_1", 5, "node_1", "node_2", true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Initial order (update 0) must start at sequence 0.");
}

TEST_F(OrderValidationTest, UnreleasedFirstNode)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, false), create_node("node_1", 2, false),
    create_node("node_2", 4, false)};

  order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", false),
    create_edge("edge_1", 3, "node_1", "node_2", false)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "First node of the order must always be released.");
}

TEST_F(OrderValidationTest, EvenNodeSequences)
{
  auto order = create_order("order_0", 1);

  order.nodes = {create_node("node_0", 1, true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Node sequences must be even.");
}

TEST_F(OrderValidationTest, NodeBaseHorizonSeparation)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, false),
    create_node("node_2", 4, true)};

  order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", true),
    create_edge("edge_1", 3, "node_1", "node_2", false)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Horizon cannot start at a node. "
    "The preceeding edge must also be released.");
}

TEST_F(OrderValidationTest, OddEdgeSequences)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 4, true)};

  order.edges = {create_edge("edge_0", 2, "node_0", "node_1", true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Edge sequences must be odd.");
}

TEST_F(OrderValidationTest, DisconnectedGraph)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 4, true)};

  order.edges = {create_edge("edge_0", 1, "node_0", "node_1", true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(), "Sequence jump detected.");
}

TEST_F(OrderValidationTest, DisconnectedEdge)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, true)};

  order.edges = {create_edge("edge_0", 1, "node_0", "node_2", true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Edge connectivity mismatch.");
}

TEST_F(OrderValidationTest, EdgeBaseHorizonSeparation)
{
  auto order = create_order("order_0", 0);

  order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, false),
    create_node("node_2", 4, false)};

  order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", false),
    create_edge("edge_1", 3, "node_1", "node_2", true)};

  auto res = vda5050_core::validation::is_valid_graph(order);

  EXPECT_FALSE(res);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::GraphValidationError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Released edge found within horizon.");
}

TEST_F(OrderValidationTest, OrderUpdateAgainstCurrentBase)
{
  auto base_order = create_order("order_0", 1);

  base_order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, true),
    create_node("node_2", 4, true), create_node("node_3", 6, false)};

  base_order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", true),
    create_edge("edge_1", 3, "node_1", "node_2", true),
    create_edge("edge_2", 5, "node_2", "node_3", false)};

  auto next_order = create_order("order_0", 1);

  next_order.nodes = {
    create_node("node_1", 2, true), create_node("node_2", 4, true),
    create_node("node_4", 6, true)};

  next_order.edges = {
    create_edge("edge_1", 3, "node_1", "node_2", true),
    create_edge("edge_3", 5, "node_2", "node_4", true),
  };

  auto res = vda5050_core::validation::is_valid_update(base_order, next_order);

  EXPECT_FALSE(res);
  EXPECT_EQ(res.errors.size(), 1);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::OrderUpdateError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Order update must be strictly increasing.");
}

TEST_F(OrderValidationTest, DisconnectedOrderUpdate)
{
  auto base_order = create_order("order_0", 1);

  base_order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, true),
    create_node("node_2", 4, true), create_node("node_3", 6, false)};

  base_order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", true),
    create_edge("edge_1", 3, "node_1", "node_2", true),
    create_edge("edge_2", 5, "node_2", "node_3", false)};

  auto next_order = create_order("order_0", 2);

  next_order.nodes = {create_node("node_4", 6, true)};

  auto res = vda5050_core::validation::is_valid_update(base_order, next_order);

  EXPECT_FALSE(res);
  EXPECT_EQ(res.errors.size(), 1);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::OrderUpdateError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Stitching failure. The update must start exactly at the "
    "last released node of the base");
}

TEST_F(OrderValidationTest, StitchingNewOrder)
{
  auto base_order = create_order("order_0", 1);

  base_order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, true),
    create_node("node_2", 4, true), create_node("node_3", 6, false)};

  base_order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", true),
    create_edge("edge_1", 3, "node_1", "node_2", true),
    create_edge("edge_2", 5, "node_2", "node_3", false)};

  auto next_order = create_order("order_0", 2);

  next_order.nodes = {create_node("node_4", 4, true)};

  auto res = vda5050_core::validation::is_valid_update(base_order, next_order);

  EXPECT_FALSE(res);
  EXPECT_EQ(res.errors.size(), 1);
  EXPECT_EQ(
    res.errors.front().error_type, vda5050_core::errors::OrderUpdateError);
  EXPECT_EQ(
    res.errors.front().error_description.value(),
    "Stitching failure. The update must start exactly at the "
    "last released node of the base");
}

TEST_F(OrderValidationTest, OrderValidationSuccess)
{
  auto base_order = create_order("order_0", 0);

  base_order.nodes = {
    create_node("node_0", 0, true), create_node("node_1", 2, true),
    create_node("node_2", 4, true), create_node("node_3", 6, false)};

  base_order.edges = {
    create_edge("edge_0", 1, "node_0", "node_1", true),
    create_edge("edge_1", 3, "node_1", "node_2", true),
    create_edge("edge_2", 5, "node_2", "node_3", false)};

  auto next_order = create_order("order_0", 1);

  next_order.nodes = {
    create_node("node_2", 4, true), create_node("node_4", 6, true)};

  next_order.edges = {
    create_edge("edge_3", 5, "node_2", "node_4", true),
  };

  auto res = vda5050_core::validation::is_valid_update(base_order, next_order);

  EXPECT_TRUE(res);
}
