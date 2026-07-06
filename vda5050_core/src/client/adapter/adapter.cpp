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

#include <thread>
#include <utility>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/validation/order_graph_validator.hpp"

#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"

#include "adapter_impl.hpp"
#include "vda5050_core/client/adapter/adapter.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
void Adapter::Implementation::subscribe_orders()
{
  protocol_adapter->subscribe<types::Order>(
    [this](types::Order order, auto error) {
      if (error.has_value())
      {
        state_manager->add_error(error.value());
        request_state_publish();
        return;
      }

      auto active = active_order.lock();

      if (!active->order.has_value())
      {
        auto result = validation::is_valid_graph(order);

        if (!result)
        {
          for (const auto& e : result.errors)
          {
            state_manager->add_error(e);
          }

          request_state_publish();
          return;
        }

        ActiveOrder incoming;
        incoming.order = order;

        incoming.last_completed_node_sequence_id.reset();

        incoming.node_lookup.clear();
        incoming.edge_lookup.clear();

        for (std::size_t i = 0; i < order.nodes.size(); ++i)
        {
          incoming.node_lookup.emplace(order.nodes[i].sequence_id, i);
        }

        for (std::size_t i = 0; i < order.edges.size(); ++i)
        {
          incoming.edge_lookup.emplace(order.edges[i].sequence_id, i);
        }

        active->order = std::move(incoming);

        state_manager->set_order(active->order->order);
        request_state_publish();

        VDA5050_INFO("Accepted new order [{}]", active->order->order.order_id);

        return;
      }

      auto result = validation::is_valid_update(active->order->order, order);

      if (!result)
      {
        for (const auto& e : result.errors)
        {
          state_manager->add_error(e);
        }

        request_state_publish();
        return;
      }

      active->order->order = std::move(order);

      state_manager->set_order(active->order->order);
      request_state_publish();

      VDA5050_INFO(
        "Accepted order update for ID [{}]", active->order->order.order_id);
    },
    0);
}

//=============================================================================
void Adapter::Implementation::subscribe_instant_actions()
{
  protocol_adapter->subscribe<types::InstantActions>(
    [this](types::InstantActions actions, auto error) {
      if (error.has_value())
      {
        state_manager->add_error(error.value());
        return;
      }

      auto queue = instant_actions.lock();

      for (auto& action : actions.actions)
      {
        queue->push_back(std::move(action));
      }

      VDA5050_INFO("Received {} instant actions", actions.actions.size());
    },
    0);
}

//=============================================================================
void Adapter::Implementation::start_dispatch_thread()
{
  dispatch_thread = std::thread([this]() {
    while (running)
    {
      process_navigation();

      process_actions();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

//=============================================================================
void Adapter::Implementation::start_state_thread()
{
  state_thread = std::thread([this]() {
    while (running)
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      state_cv.wait_for(lock, std::chrono::seconds(30), [this]() {
        return !running || state_manager->consume_publish_requested();
      });

      if (!running) break;

      lock.unlock();

      publish_state();
    }
  });
}

//=============================================================================
void Adapter::Implementation::set_connection_will()
{
  types::Connection connection_will;
  connection_will.connection_state = types::ConnectionState::CONNECTIONBROKEN;
  protocol_adapter->set_will<types::Connection>(connection_will, 1, true);
}

//=============================================================================
void Adapter::Implementation::publish_connection_online()
{
  types::Connection connection_online;
  connection_online.connection_state = types::ConnectionState::ONLINE;
  protocol_adapter->publish<types::Connection>(connection_online, 1, true);
}

//=============================================================================
void Adapter::Implementation::publish_connection_offline()
{
  types::Connection connection_offline;
  connection_offline.connection_state = types::ConnectionState::OFFLINE;
  protocol_adapter->publish<types::Connection>(connection_offline, 1, true);
}

//=============================================================================
void Adapter::Implementation::process_navigation()
{
  if (!navigation_callback) return;

  ActiveOrder order_state;
  {
    auto active = active_order.lock();

    if (!active->order.has_value()) return;

    if (active->order->executing) return;

    order_state = *active->order;
  }

  const auto& order = order_state.order;

  uint32_t next_node_seq;

  if (!order_state.last_completed_node_sequence_id.has_value())
  {
    next_node_seq = 0;
  }
  else
  {
    next_node_seq = order_state.last_completed_node_sequence_id.value() + 2;
  }

  auto node_it = order_state.node_lookup.find(next_node_seq);
  if (node_it == order_state.node_lookup.end()) return;

  const auto& next_node = order.nodes[node_it->second];

  auto node_request = NodeRequest::from_node(next_node);

  uint32_t next_edge_seq = next_node_seq - 1;

  auto edge_it = order_state.edge_lookup.find(next_edge_seq);

  std::optional<EdgeRequest> edge_request;
  if (edge_it != order_state.edge_lookup.end())
  {
    edge_request = EdgeRequest::from_edge(order.edges[edge_it->second]);
  }

  {
    auto active = active_order.lock();

    if (!active->order.has_value()) return;

    active->order->executing = true;
  }

  VDA5050_INFO(
    "Dispatching node ID [{}] with sequence [{}]", next_node.node_id,
    next_node_seq);

  try
  {
    navigation_callback(
      std::move(node_request), std::move(edge_request),
      OrderExecution::make(
        order.order_id, order.order_update_id,
        [this, next_node_seq, next_edge_seq]() {
          ActiveOrder order_state;
          {
            auto active = active_order.lock();
            if (!active->order.has_value()) return;

            order_state = *active->order;

            active->order->executing = false;
            active->order->last_completed_node_sequence_id = next_node_seq;
          }

          auto edge_it = order_state.edge_lookup.find(next_edge_seq);
          if (edge_it != order_state.edge_lookup.end())
          {
            state_manager->edge_traversed(
              order_state.order.edges[edge_it->second]);
          }

          auto node_it = order_state.node_lookup.find(next_node_seq);
          if (node_it != order_state.node_lookup.end())
          {
            state_manager->node_reached(
              order_state.order.nodes[node_it->second]);
          }

          request_state_publish();
        },
        [this](const std::string& reason) {
          types::Error e;
          e.error_type = errors::NavigationFailedError;
          e.error_description = reason;

          state_manager->add_error(e);

          auto active = active_order.lock();
          if (active->order.has_value())
          {
            active->order->executing = false;
          }

          request_state_publish();
        }));
  }
  catch (const std::exception& e)
  {
    VDA5050_ERROR(
      "Navigation failed towards node ID [{}] with sequence ID [{}]",
      next_node.node_id, next_node.sequence_id);
  }
}

//=============================================================================
void Adapter::Implementation::process_actions()
{
  if (!action_callback) return;

  types::Action action;
  {
    auto actions = instant_actions.lock();

    if (actions->empty()) return;

    action = std::move(actions->front());
    actions->erase(actions->begin());
  }

  types::ActionState action_state;
  action_state.action_id = action.action_id;
  action_state.action_type = action.action_type;
  action_state.action_status = types::ActionStatus::RUNNING;
  action_state.action_description = action.action_description;

  state_manager->add_action_state(action_state);

  auto request = ActionRequest::from_action(action);

  VDA5050_INFO(
    "Dispatching action type [{}] with action ID [{}]", action.action_type,
    action.action_id);

  auto execution = ActionExecution::make(
    [this, action](types::ActionStatus status, auto result_description) {
      types::ActionState action_state;
      action_state.action_id = action.action_id;
      action_state.action_type = action.action_type;
      action_state.action_status = status;
      action_state.action_description = action.action_description;
      action_state.result_description = std::move(result_description);

      state_manager->add_action_state(action_state);
      request_state_publish();
    });

  if (action.action_type == "factsheetRequest")
  {
    handle_factsheet_request(execution);
  }
  else if (action.action_type == "stateRequest")
  {
    handle_state_request(execution);
  }
  else if (action.action_type == "initPosition")
  {
    handle_init_position(action.action_parameters.value(), execution);
  }
  else
  {
    try
    {
      action_callback(std::move(request), execution);
    }
    catch (const std::exception& e)
    {
      VDA5050_ERROR(
        "Failed to run action of type [{}] and action ID [{}]",
        action.action_type, action.action_id);
    }
  }
}

//=============================================================================
void Adapter::Implementation::publish_factsheet()
{
  auto manager = factsheet_manager.lock();
  protocol_adapter->publish<types::Factsheet>(manager->factsheet.value(), 0);
}

//=============================================================================
void Adapter::Implementation::publish_state()
{
  protocol_adapter->publish<types::State>(state_manager->state(), 0);
}

//=============================================================================
void Adapter::Implementation::request_state_publish()
{
  state_manager->mark_publish_requested();
  state_cv.notify_all();
}

//=============================================================================
void Adapter::Implementation::handle_state_request(
  std::shared_ptr<ActionExecution> execution)
{
  request_state_publish();
  execution->finished();
  VDA5050_INFO("Received stateRequest");
}

//=============================================================================
void Adapter::Implementation::handle_factsheet_request(
  std::shared_ptr<ActionExecution> execution)
{
  publish_factsheet();
  execution->finished();
  VDA5050_INFO("Received factsheetRequest");
}

//=============================================================================
void Adapter::Implementation::handle_init_position(
  std::vector<types::ActionParameter> parameters,
  std::shared_ptr<ActionExecution> execution)
{
  VDA5050_INFO("Received initPosition");
}

//=============================================================================
types::Factsheet Adapter::Implementation::make_default_factsheet()
{
  types::Factsheet factsheet;

  factsheet.type_specification.series_name = "Generic AMR";
  factsheet.type_specification.agv_kinematic = types::AGVKinematic::DIFF;
  factsheet.type_specification.agv_class = types::AGVClass::CARRIER;

  factsheet.type_specification.max_load_mass = 100.0;

  factsheet.type_specification.localization_types = {"NATURAL"};

  factsheet.type_specification.navigation_types = {"AUTONOMOUS"};

  factsheet.physical_parameters.speed_min = 0.05;
  factsheet.physical_parameters.speed_max = 1.5;

  factsheet.physical_parameters.acceleration_max = 0.5;
  factsheet.physical_parameters.deceleration_max = 0.5;

  factsheet.physical_parameters.height_min = 0.30;
  factsheet.physical_parameters.height_max = 0.30;

  factsheet.physical_parameters.width = 0.60;
  factsheet.physical_parameters.length = 0.80;

  factsheet.physical_parameters.angular_speed_min = 0.1;
  factsheet.physical_parameters.angular_speed_max = 1.0;

  factsheet.protocol_limits.max_string_lens.msg_len = 1024 * 1024;

  factsheet.protocol_limits.max_string_lens.topic_serial_len = 64;
  factsheet.protocol_limits.max_string_lens.topic_elem_len = 64;

  factsheet.protocol_limits.max_string_lens.id_len = 128;
  factsheet.protocol_limits.max_string_lens.enum_len = 64;
  factsheet.protocol_limits.max_string_lens.load_id_len = 128;

  factsheet.protocol_limits.max_string_lens.id_numerical_only = false;

  factsheet.protocol_limits.max_array_lens.order_nodes = 1000;
  factsheet.protocol_limits.max_array_lens.order_edges = 1000;

  factsheet.protocol_limits.max_array_lens.node_actions = 50;
  factsheet.protocol_limits.max_array_lens.edge_actions = 50;

  factsheet.protocol_limits.max_array_lens.actions_actions_parameters = 50;

  factsheet.protocol_limits.max_array_lens.instant_actions = 50;

  factsheet.protocol_limits.max_array_lens.trajectory_knot_vector = 1000;
  factsheet.protocol_limits.max_array_lens.trajectory_control_points = 1000;

  factsheet.protocol_limits.max_array_lens.state_node_states = 1000;
  factsheet.protocol_limits.max_array_lens.state_edge_states = 1000;

  factsheet.protocol_limits.max_array_lens.state_loads = 20;
  factsheet.protocol_limits.max_array_lens.state_action_states = 100;
  factsheet.protocol_limits.max_array_lens.state_errors = 100;
  factsheet.protocol_limits.max_array_lens.state_information = 100;

  factsheet.protocol_limits.max_array_lens.error_error_references = 20;
  factsheet.protocol_limits.max_array_lens.information_info_references = 20;

  factsheet.protocol_features.optional_parameters = {};

  factsheet.protocol_features.agv_actions = {};

  return factsheet;
}

//=============================================================================
Adapter::~Adapter()
{
  stop();
}

//=============================================================================
std::shared_ptr<Adapter> Adapter::make(
  std::shared_ptr<execution::ProtocolAdapter> protocol_adapter)
{
  auto adapter =
    std::shared_ptr<Adapter>(new Adapter(std::move(protocol_adapter)));
  return adapter;
}

//=============================================================================
void Adapter::on_navigate(
  std::function<void(
    NodeRequest, std::optional<EdgeRequest>, std::shared_ptr<OrderExecution>)>
    callback)
{
  pimpl_->navigation_callback = std::move(callback);
}

//=============================================================================
void Adapter::on_action(
  std::function<void(ActionRequest, std::shared_ptr<ActionExecution>)> callback)
{
  pimpl_->action_callback = std::move(callback);
}

//=============================================================================
std::shared_ptr<StateManager> Adapter::state_manager()
{
  return pimpl_->state_manager;
}

//=============================================================================
void Adapter::set_factsheet(const types::Factsheet& factsheet)
{
  pimpl_->factsheet_manager.lock()->factsheet = factsheet;
}

//=============================================================================
void Adapter::start()
{
  if (pimpl_->running) return;

  auto factsheet_manager = pimpl_->factsheet_manager.lock();
  if (!factsheet_manager->factsheet.has_value())
  {
    factsheet_manager->factsheet = pimpl_->make_default_factsheet();
  }

  pimpl_->set_connection_will();
  pimpl_->protocol_adapter->connect();

  if (!pimpl_->protocol_adapter->connected())
  {
    VDA5050_WARN(
      "Adapter started but MQTT broker is not connected. "
      "Messages will be dropped until a connection is established");
  }

  pimpl_->running = true;

  pimpl_->subscribe_orders();

  pimpl_->subscribe_instant_actions();

  pimpl_->publish_connection_online();

  pimpl_->start_dispatch_thread();

  pimpl_->start_state_thread();
}

//=============================================================================
void Adapter::stop()
{
  if (!pimpl_->running) return;

  pimpl_->running = false;
  pimpl_->state_cv.notify_all();

  if (pimpl_->dispatch_thread.joinable()) pimpl_->dispatch_thread.join();

  if (pimpl_->state_thread.joinable()) pimpl_->state_thread.join();

  if (pimpl_->protocol_adapter)
  {
    pimpl_->protocol_adapter->unsubscribe<types::Order>();
    pimpl_->protocol_adapter->unsubscribe<types::InstantActions>();

    pimpl_->publish_connection_offline();

    pimpl_->protocol_adapter->disconnect();
  }
}

//=============================================================================
Adapter::Adapter(std::shared_ptr<execution::ProtocolAdapter> protocol_adapter)
: pimpl_(std::make_unique<Implementation>())
{
  pimpl_->protocol_adapter = std::move(protocol_adapter);

  pimpl_->state_manager = StateManager::make();
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
