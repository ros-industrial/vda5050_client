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

#include <algorithm>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/order_utils/order_graph_validator.hpp"

namespace vda5050_core {

namespace order_utils {

//=============================================================================
errors::ValidationResult is_valid_graph(const vda5050_core::types::Order& order)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<vda5050_core::types::ErrorReference> refs) {
    refs.push_back({errors::RefOrderId, order.order_id});
    refs.push_back(
      {errors::RefOrderUpdateId, std::to_string(order.order_update_id)});
    res.errors.push_back(
      errors::create_error(errors::GraphValidationError, description, refs));
  };

  // Check if there are nodes in the order, return if empty
  if (order.nodes.empty())
  {
    add_error("Order contains no nodes.", {});
    return res;
  }

  // Check if the number of nodes is n and number of edges is n-1,
  // return if empty
  if (order.nodes.size() != order.edges.size() + 1)
  {
    add_error(
      "Graph mismatch: Order contains " + std::to_string(order.nodes.size()) +
        " node(s) but " + std::to_string(order.edges.size()) + " edge(s).",
      {});
    return res;
  }

  // Check if order update is 0, (means first update of its order)
  // reject if first node sequence is > 0
  if (order.order_update_id == 0 && order.nodes.front().sequence_id != 0)
  {
    add_error(
      "Initial order (update 0) must start at sequence 0.",
      {{errors::RefNodeId, order.nodes.front().node_id},
       {errors::RefSequenceId,
        std::to_string(order.nodes.front().sequence_id)}});
    return res;
  }

  // Check if the first node is released
  if (!order.nodes.front().released)
  {
    add_error(
      "First node of the order must always be released.",
      {{errors::RefNodeId, order.nodes.front().node_id},
       {errors::RefSequenceId,
        std::to_string(order.nodes.front().sequence_id)}});
    return res;
  }

  bool horizon_reached = false;

  // Iterate through nodes and edges
  for (size_t i = 0; i < order.nodes.size(); ++i)
  {
    const auto& node = order.nodes[i];

    // Check if node sequence is even throughout
    if (node.sequence_id % 2 != 0)
    {
      add_error(
        "Node sequences must be even.",
        {{errors::RefNodeId, node.node_id},
         {errors::RefSequenceId, std::to_string(node.sequence_id)}});
    }

    // Check for proper base and horizon separation
    if (node.released && horizon_reached)
    {
      add_error(
        "Released node found within horizon.",
        {{errors::RefNodeId, node.node_id}});
    }
    else if (!node.released && !horizon_reached)
    {
      add_error(
        "Horizon cannot start at a node. "
        "The preceeding edge must also be released.",
        {{errors::RefNodeId, node.node_id}});
    }

    // Check the connectivity of edges
    if (i < order.edges.size())
    {
      const auto& edge = order.edges[i];
      const auto& next_node = order.nodes[i + 1];

      // Check if edge sequence is odd throughout
      if (edge.sequence_id % 2 == 0)
      {
        add_error(
          "Edge sequences must be odd.",
          {{errors::RefEdgeId, edge.edge_id},
           {errors::RefSequenceId, std::to_string(edge.sequence_id)}});
      }

      // Check contuinity of sequences (node -> edge -> node)
      if (
        edge.sequence_id != node.sequence_id + 1 ||
        next_node.sequence_id != edge.sequence_id + 1)
      {
        add_error(
          "Sequence jump detected.", {{errors::RefEdgeId, edge.edge_id}});
      }

      // Check if the edge is connected to correct start and end nodes
      if (
        edge.start_node_id != node.node_id ||
        edge.end_node_id != next_node.node_id)
      {
        add_error(
          "Edge connectivity mismatch.", {{errors::RefEdgeId, edge.edge_id},
                                          {errors::RefNodeId, node.node_id}});
      }

      // Check for proper base and horizon separation
      if (!edge.released)
      {
        horizon_reached = true;
      }
      else if (horizon_reached)
      {
        add_error(
          "Released edge found within horizon.",
          {{errors::RefEdgeId, edge.edge_id}});
      }
    }
  }

  return res;
}

//=============================================================================
errors::ValidationResult is_valid_update(
  const vda5050_core::types::Order& base_order,
  const vda5050_core::types::Order& next_order)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<vda5050_core::types::ErrorReference> refs) {
    refs.push_back({errors::RefOrderId, next_order.order_id});
    refs.push_back(
      {errors::RefOrderUpdateId, std::to_string(next_order.order_update_id)});
    res.errors.push_back(
      errors::create_error(errors::OrderUpdateError, description, refs));
  };

  // Check if it is an existing order
  if (next_order.order_id == base_order.order_id)
  {
    if (next_order.order_update_id <= base_order.order_update_id)
    {
      add_error("Order update must be strictly increasing.", {});
      return res;
    }
  }

  // Check the validity of the graph before proceeding with stitching
  res = is_valid_graph(next_order);
  if (!res) return res;

  // Look for a last released node in the existing order
  auto last_base_it = std::find_if(
    base_order.nodes.rbegin(), base_order.nodes.rend(),
    [](const auto& n) { return n.released; });

  if (last_base_it == base_order.nodes.rend()) return res;

  // Check if the update starts with the last base
  if (next_order.nodes.front() != *last_base_it)
  {
    add_error(
      "Stitching failure. The update must start exactly at the "
      "last released node of the base",
      {{errors::RefSequenceId,
        std::to_string(next_order.nodes.front().sequence_id)}});
    return res;
  }

  return res;
}

}  // namespace order_utils
}  // namespace vda5050_core
