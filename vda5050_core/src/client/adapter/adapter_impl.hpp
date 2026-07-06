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

#ifndef CLIENT__ADAPTER__ADAPTER_IMPL_HPP_
#define CLIENT__ADAPTER__ADAPTER_IMPL_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"
#include "vda5050_core/types/order.hpp"

#include "vda5050_core/client/adapter/adapter.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

template <typename T>
class SharedState
{
public:
  class Locked
  {
  public:
    Locked(std::mutex& mutex, T& data) : lock_(mutex), data_(data)
    {
      // Nothing to do here ...
    }

    T* operator->()
    {
      return &data_;
    }

    T& operator*()
    {
      return data_;
    }

  private:
    std::unique_lock<std::mutex> lock_;
    T& data_;
  };

  Locked lock()
  {
    return Locked(mutex_, data_);
  }

private:
  std::mutex mutex_;
  T data_;
};

struct ActiveOrder
{
  types::Order order;

  std::unordered_map<uint32_t, std::size_t> node_lookup;
  std::unordered_map<uint32_t, std::size_t> edge_lookup;

  std::optional<std::uint32_t> last_completed_node_sequence_id;

  bool executing{false};
};

struct ActiveOrderState
{
  std::optional<ActiveOrder> order;
};

struct FactsheetManager
{
  std::optional<types::Factsheet> factsheet;
};

class Adapter::Implementation
{
public:
  std::shared_ptr<execution::ProtocolAdapter> protocol_adapter;

  std::shared_ptr<StateManager> state_manager;

  SharedState<ActiveOrderState> active_order;

  SharedState<std::vector<types::Action>> instant_actions;

  SharedState<FactsheetManager> factsheet_manager;

  std::function<void(
    NodeRequest, std::optional<EdgeRequest>, std::shared_ptr<OrderExecution>)>
    navigation_callback;

  std::function<void(ActionRequest, std::shared_ptr<ActionExecution>)>
    action_callback;

  std::thread dispatch_thread;
  std::thread state_thread;

  std::mutex state_mutex;
  std::condition_variable state_cv;

  std::atomic_bool running{false};

  void subscribe_orders();

  void subscribe_instant_actions();

  void start_dispatch_thread();

  void start_state_thread();

  void set_connection_will();

  void publish_connection_online();

  void publish_connection_offline();

  void process_navigation();

  void process_actions();

  void publish_factsheet();

  void publish_state();

  void request_state_publish();

  void handle_state_request(std::shared_ptr<ActionExecution> execution);

  void handle_factsheet_request(std::shared_ptr<ActionExecution> execution);

  void handle_init_position(
    std::vector<types::ActionParameter> parameters,
    std::shared_ptr<ActionExecution> execution);

  types::Factsheet make_default_factsheet();
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // CLIENT__ADAPTER__ADAPTER_IMPL_HPP_
