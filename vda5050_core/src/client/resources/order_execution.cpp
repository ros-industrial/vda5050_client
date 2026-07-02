/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "vda5050_core/client/resources/order_execution.hpp"

#include <mutex>
#include <utility>

namespace vda5050_core {

namespace client {

bool OrderExecutionResource::is_executing_order() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return executing_order_;
}

void OrderExecutionResource::set_executing_order(bool executing)
{
  std::lock_guard<std::mutex> lock(mutex_);
  executing_order_ = executing;
}

bool OrderExecutionResource::is_awaiting_order_update() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return awaiting_order_update_;
}

void OrderExecutionResource::set_awaiting_order_update(bool awaiting)
{
  std::lock_guard<std::mutex> lock(mutex_);
  awaiting_order_update_ = awaiting;
}

types::State OrderExecutionResource::get_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void OrderExecutionResource::set_state(types::State state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = std::move(state);
}

}  // namespace client
}  // namespace vda5050_core
