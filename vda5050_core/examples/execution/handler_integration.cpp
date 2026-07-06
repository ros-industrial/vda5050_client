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

#include <iostream>
#include <memory>
#include <mutex>
#include <typeindex>
#include <unordered_map>

#include <vda5050_core/logger/logger.hpp>

#include "vda5050_core/execution/base.hpp"
#include "vda5050_core/execution/context_interface.hpp"
#include "vda5050_core/execution/handler.hpp"
#include "vda5050_core/execution/strategy_interface.hpp"

using EventBase = vda5050_core::execution::EventBase;
using UpdateBase = vda5050_core::execution::UpdateBase;
using ResourceBase = vda5050_core::execution::ResourceBase;
using Handler = vda5050_core::execution::Handler;
using Priority = vda5050_core::execution::Priority;

struct ConfigResource
: public vda5050_core::execution::Initialize<ConfigResource, ResourceBase>
{
  std::string serial_number;

  explicit ConfigResource(const std::string& serial_number)
  : serial_number(std::move(serial_number))
  {
    // Nothing to do here ...
  }
};

struct OrderUpdate
: public vda5050_core::execution::Initialize<OrderUpdate, UpdateBase>
{
  std::string order_id;
  uint32_t sequence_id;

  OrderUpdate(const std::string& order_id, uint32_t sequence_id)
  : order_id(std::move(order_id)), sequence_id(sequence_id)
  {
    // Nothing to do here ...
  }
};

struct OrderEvent
: public vda5050_core::execution::Initialize<OrderEvent, EventBase>
{
  std::string order_id;
  uint32_t sequence_id;
  std::string node_id;

  OrderEvent(
    const std::string& order_id, uint32_t sequence_id,
    const std::string& node_id)
  : order_id(std::move(order_id)),
    sequence_id(sequence_id),
    node_id(std::move(node_id))
  {
    // Nothing to do here ...
  }
};

class SimpleContext : public vda5050_core::execution::ContextInterface
{
public:
  void init() override
  {
    provider()->on<OrderUpdate>([this](auto update) {
      std::lock_guard<std::mutex> lock(update_mutex_);
      updates_[update->get_type()] = update;
    });

    auto config = std::make_shared<ConfigResource>("agv_1");
    std::lock_guard<std::mutex> lock(resource_mutex_);
    resources_[config->get_type()] = config;
  }

protected:
  std::shared_ptr<UpdateBase> get_update_raw(
    std::type_index type) const override
  {
    std::lock_guard<std::mutex> lock(update_mutex_);
    auto it = updates_.find(type);
    if (it != updates_.end()) return it->second;
    return nullptr;
  }

  std::shared_ptr<ResourceBase> get_resource_raw(
    std::type_index type) const override
  {
    std::lock_guard<std::mutex> lock(resource_mutex_);
    auto it = resources_.find(type);
    if (it != resources_.end()) return it->second;
    return nullptr;
  }

private:
  std::unordered_map<std::type_index, std::shared_ptr<UpdateBase>> updates_;
  mutable std::mutex update_mutex_;

  std::unordered_map<std::type_index, std::shared_ptr<ResourceBase>> resources_;
  mutable std::mutex resource_mutex_;
};

class OrderDispatchStrategy : public vda5050_core::execution::StrategyInterface
{
public:
  void init(
    std::shared_ptr<vda5050_core::execution::ContextInterface> context) override
  {
    engine()->on<OrderEvent>([](auto event) {
      VDA5050_INFO_STREAM("[Engine] Dispatched Order " << event->order_id);
    });

    context->provider()->on<OrderUpdate>([this](auto update) {
      this->engine()->notify(update);
      VDA5050_INFO("[Context] Adding OrderUpdate. Notifying Engine");
    });
  }

  void step(
    std::shared_ptr<vda5050_core::execution::ContextInterface> context) override
  {
    if (engine()->waiting())
    {
      VDA5050_INFO("[Strategy] Waiting for Engine");
      return;
    }

    engine()->step();

    auto order_update = context->get_update<OrderUpdate>();

    if (order_update)
    {
      auto config = context->get_resource<ConfigResource>();
      VDA5050_INFO_STREAM(
        "[Strategy] AGV with Serial Number {"
        << config->serial_number << "} Executing Order {"
        << order_update->order_id << "} Reached Sequence {"
        << order_update->sequence_id << "}");

      if (order_update->sequence_id == 1)
      {
        engine()->emit<OrderEvent>(Priority::NORMAL, "order_1", 2, "node_2");

        engine()->step();

        engine()->suspend_for<OrderUpdate>(
          std::chrono::seconds(5), [](auto update) {
            return update->order_id == "order_1" && update->sequence_id == 2;
          });
      }
    }
  }
};

int main()
{
  auto context = std::make_shared<SimpleContext>();
  auto strategy = std::make_shared<OrderDispatchStrategy>();
  std::vector<std::shared_ptr<vda5050_core::execution::StrategyInterface>>
    strategies = {strategy};
  auto handler = Handler::make(context, strategies);

  VDA5050_INFO("Stage 1: Triggering Sequence 1 ...");
  context->provider()->push<OrderUpdate>("order_1", 1);
  handler->spin_once();

  VDA5050_INFO("Stage 2: Satisfying the Wait with Sequence 2 ...");
  context->provider()->push<OrderUpdate>("order_1", 2);
  handler->spin_once();

  VDA5050_INFO("Execution Completed");

  return 0;
}
