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

#include "vda5050_core/master/order/order_publisher.hpp"

#include <cstdint>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/validation/capability_validator.hpp"
#include "vda5050_core/validation/content_validator.hpp"
#include "vda5050_core/validation/order_graph_validator.hpp"
#include "vda5050_core/validation/pre_send_validator.hpp"
#include "vda5050_core/validation/traversability_validator.hpp"

namespace vda5050_core::master {

vda5050_core::errors::ValidationResult OrderPublisher::publish(
  vda5050_core::execution::ProtocolAdapter& adapter,
  const vda5050_core::validation::PreSendContext& ctx,
  const vda5050_core::types::Order& order,
  const std::optional<vda5050_core::types::Order>& active_order,
  std::optional<vda5050_core::types::Order>* merged_out)
{
  // Validator chain: schema → PreSend → structural → traversability →
  // capability → publish. Each link short-circuits on failure.
  auto schema_result = vda5050_core::validation::validate_order_content(order);
  if (!schema_result)
  {
    return schema_result;
  }

  auto pre_send_result = vda5050_core::validation::validate_pre_send(ctx);
  if (!pre_send_result)
  {
    return pre_send_result;
  }

  // Structural validation branches on update vs new order.
  //
  // For a fresh order (no active, or different order_id): `is_valid_graph`
  // validates the full graph standalone — consecutive sequence_ids,
  // alternating node/edge pattern, base/horizon split.
  //
  // For an UPDATE (active present + same order_id): the candidate has
  // sparse sequence_ids by spec (it only contains the stitch node + new
  // nodes/edges; base seqs are absent because the spec forbids
  // retransmitting them). So `is_valid_graph` cannot be applied to the
  // candidate alone — we use `combine_order` instead, which validates
  // stitch identity, base immutability, monotonic order_update_id,
  // released-node preservation, etc., against the active order. We publish
  // the candidate as-sent, but hand the merged result back via `merged_out`
  // so the caller can adopt it as the internal active order without
  // re-combining against a state that may have advanced since.
  if (active_order.has_value() && active_order->order_id == order.order_id)
  {
    const uint32_t last_seq =
      ctx.last_state.has_value() ? ctx.last_state->last_node_sequence_id : 0;
    auto combine_res = combine_order(*active_order, order, last_seq);
    if (!combine_res)
    {
      vda5050_core::errors::ValidationResult res;
      for (auto& error : combine_res.errors)
      {
        res.add_error(std::move(error));
      }
      return res;
    }
    if (merged_out != nullptr)
    {
      *merged_out = std::move(combine_res.order);
    }
  }
  else
  {
    auto graph_result = vda5050_core::validation::is_valid_graph(order);
    if (!graph_result) return graph_result;
    if (graph_result.has_warnings())
    {
      VDA5050_WARN(
        "[OrderPublisher] order {} has {} graph advisory(ies); publishing",
        order.order_id, graph_result.warnings().size());
    }
  }

  auto traversability_result =
    vda5050_core::validation::validate_traversability(ctx, order);
  if (!traversability_result)
  {
    return traversability_result;
  }

  auto capability_result =
    vda5050_core::validation::validate_capability(ctx, order);
  if (!capability_result)
  {
    return capability_result;
  }

  adapter.publish<vda5050_core::types::Order>(
    order, static_cast<int>(OrderQos));

  return vda5050_core::errors::ValidationResult{};  // success
}

}  // namespace vda5050_core::master
