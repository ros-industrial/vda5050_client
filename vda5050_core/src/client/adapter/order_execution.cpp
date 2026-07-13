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

#include "vda5050_core/client/adapter/order_execution.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
std::shared_ptr<OrderExecution> OrderExecution::make(
  const std::string& order_id, uint32_t order_update_id,
  std::function<void()> finish_callback,
  std::function<void(std::string)> fail_callback)
{
  auto execution = std::shared_ptr<OrderExecution>(new OrderExecution(
    order_id, order_update_id, std::move(finish_callback),
    std::move(fail_callback)));
  return execution;
}

//=============================================================================
const std::string& OrderExecution::order_id() const
{
  return order_id_;
}

//=============================================================================
uint32_t OrderExecution::order_update_id() const
{
  return order_update_id_;
}

//=============================================================================
void OrderExecution::finished()
{
  if (is_finished()) return;

  if (finish_callback_) finish_callback_();

  Execution::finished();
}

//=============================================================================
void OrderExecution::failed(const std::string& reason)
{
  if (is_finished()) return;

  if (fail_callback_) fail_callback_(reason);

  Execution::failed(reason);
}

//=============================================================================
OrderExecution::OrderExecution(
  const std::string& order_id, uint32_t order_update_id,
  std::function<void()> finish_callback,
  std::function<void(std::string)> fail_callback)
: Execution(),
  finish_callback_(std::move(finish_callback)),
  fail_callback_(std::move(fail_callback)),
  order_id_(order_id),
  order_update_id_(order_update_id)
{
  // Nothing to do here ...
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
