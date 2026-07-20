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

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/validation/capability_validator.hpp"
#include "vda5050_core/validation/content_validator.hpp"
#include "vda5050_core/validation/order_graph_validator.hpp"
#include "vda5050_core/validation/pre_send_validator.hpp"
#include "vda5050_core/validation/protocol_limits_validator.hpp"
#include "vda5050_core/validation/traversability_validator.hpp"

namespace vda5050_core::master {

vda5050_core::errors::ValidationResult OrderPublisher::publish(
  vda5050_core::execution::ProtocolAdapter& adapter,
  const vda5050_core::validation::PreSendContext& ctx,
  const vda5050_core::types::Order& order,
  const std::optional<vda5050_core::types::Order>& active_order,
  std::optional<vda5050_core::types::Order>* merged_out)
{
  // Chain: schema → PreSend → structural → limits → traversability →
  // capability. Structural and limits take the merged order on the stitch
  // path, the order as-sent otherwise.
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

  // Stitch update is sparse: merge, validate the merged graph, publish as-sent.
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
    // is_valid_graph reports structural problems as warnings. A malformed
    // merge is a master-side bug, not caller input — reject, don't adopt it.
    auto merged_graph =
      vda5050_core::validation::is_valid_graph(combine_res.order);
    if (!merged_graph || merged_graph.has_warnings())
    {
      vda5050_core::errors::ValidationResult res;
      res.add_error(vda5050_core::errors::create_error(
        vda5050_core::errors::ValidationError,
        "Merged stitch order is not a valid graph", {}));
      return res;
    }
    // Array limits are per-order, so the merged order is the subject.
    auto merged_limits = vda5050_core::validation::validate_protocol_limits(
      ctx, combine_res.order);
    if (!merged_limits)
    {
      return merged_limits;
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
    auto limits_result =
      vda5050_core::validation::validate_protocol_limits(ctx, order);
    if (!limits_result)
    {
      return limits_result;
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

  // Fail rather than mark an order active that never went on the wire.
  if (!adapter.connected())
  {
    vda5050_core::errors::ValidationResult res;
    res.add_error(vda5050_core::errors::create_error(
      vda5050_core::errors::ValidationError,
      "Broker not connected; order not published", {}));
    return res;
  }

  adapter.publish<vda5050_core::types::Order>(
    order, static_cast<int>(OrderQos));

  return vda5050_core::errors::ValidationResult{};  // success
}

}  // namespace vda5050_core::master
