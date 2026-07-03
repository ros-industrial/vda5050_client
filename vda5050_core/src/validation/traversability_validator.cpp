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

#include "vda5050_core/validation/traversability_validator.hpp"

#include <fmt/format.h>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::validation {

namespace {

using AddErrorFn =
  std::function<void(const std::string&, std::vector<types::ErrorReference>)>;

// Reachability (state-driven; runs without a factsheet).
void validate_reachability(
  const PreSendContext& ctx, const types::Order& order,
  const AddErrorFn& add_error)
{
  if (order.nodes.empty())
  {
    return;  // the graph validator already rejects this
  }

  const auto& first = order.nodes.front();

  if (!ctx.last_state.has_value())
  {
    add_error(
      "Cannot determine reachability: AGV has not yet reported any State.", {});
    return;
  }
  const auto& state = ctx.last_state.value();

  if (state.last_node_id == first.node_id) return;

  if (!first.node_position.has_value() || !state.agv_position.has_value())
  {
    add_error(
      "Cannot determine reachability: AGV is not on first node and "
      "either node_position or agv_position is missing.",
      {{errors::RefNodeId, first.node_id}});
    return;
  }

  const auto& np = first.node_position.value();
  const auto& ap = state.agv_position.value();

  if (!ap.position_initialized)
  {
    add_error(
      "Cannot determine reachability: AGV position is not initialized.",
      {{errors::RefNodeId, first.node_id}});
    return;
  }

  if (ap.map_id != np.map_id)
  {
    add_error(
      fmt::format(
        "AGV is on a different map than the first node (AGV map '{}', node "
        "map '{}').",
        ap.map_id, np.map_id),
      {{errors::RefNodeId, first.node_id}});
    return;
  }

  // No declared deviation => must be exactly on the node.
  const double allowed = np.allowed_deviation_x_y.value_or(0.0);

  const double dx = ap.x - np.x;
  const double dy = ap.y - np.y;
  const double distance = std::hypot(dx, dy);

  if (distance > allowed)
  {
    add_error(
      fmt::format(
        "AGV is not within the first node's allowed_deviation_x_y "
        "(distance={} m, allowed={} m).",
        distance, allowed),
      {{errors::RefNodeId, first.node_id}});
  }
}

// Order nodes/edges must exist in the layout with matching endpoints, edge
// direction, and map_id. Coordinates aren't compared (map_id asserts agreement).
void validate_graph_integrity(
  const PreSendContext& ctx, const types::Order& order,
  const AddErrorFn& add_error)
{
  const auto& graph = *ctx.loaded_graph;

  for (const auto& node : order.nodes)
  {
    const auto* gn = graph.find_node(node.node_id);
    if (gn == nullptr)
    {
      add_error(
        fmt::format(
          "Order node_id '{}' is not present in the master's loaded layout.",
          node.node_id),
        {{errors::RefNodeId, node.node_id}});
      continue;
    }
    if (!node.node_position.has_value()) continue;
    const auto& np = node.node_position.value();
    if (np.map_id != gn->map_id)
    {
      add_error(
        fmt::format(
          "Order node '{}' map_id '{}' does not match the layout's map_id "
          "'{}'.",
          node.node_id, np.map_id, gn->map_id),
        {{errors::RefNodeId, node.node_id}});
    }
  }

  for (const auto& edge : order.edges)
  {
    const auto* ge = graph.find_edge(edge.edge_id);
    if (ge == nullptr)
    {
      add_error(
        fmt::format(
          "Order edge_id '{}' is not present in the master's loaded layout "
          "(edge_id must match a layout edge id).",
          edge.edge_id),
        {{errors::RefEdgeId, edge.edge_id}});
      continue;
    }
    if (
      ge->start_node_id != edge.start_node_id ||
      ge->end_node_id != edge.end_node_id)
    {
      add_error(
        fmt::format(
          "Order edge '{}' endpoints ({}->{}) do not match the layout's edge "
          "({}->{}). Layout edges are directed; reverse traversal needs a "
          "separate edge.",
          edge.edge_id, edge.start_node_id, edge.end_node_id, ge->start_node_id,
          ge->end_node_id),
        {{errors::RefEdgeId, edge.edge_id}});
    }
  }
}

}  // namespace

errors::ValidationResult validate_traversability(
  const PreSendContext& ctx, const types::Order& order)
{
  errors::ValidationResult res;

  auto add_error = [&](
                     const std::string& description,
                     std::vector<types::ErrorReference> refs) {
    refs.push_back({errors::RefOrderId, order.order_id});
    res.add_error(errors::create_error(
      errors::TraversabilityValidationError, description, refs));
  };

  validate_reachability(ctx, order, add_error);

  if (ctx.loaded_graph != nullptr)
  {
    validate_graph_integrity(ctx, order, add_error);
  }
  else
  {
    res.add_error(errors::create_error(
      errors::GraphIntegrityCheckSkipped,
      "No layout loaded; graph-integrity checks skipped.",
      {{errors::RefOrderId, order.order_id}}, types::ErrorLevel::WARNING));
  }

  return res;
}

}  // namespace vda5050_core::validation
