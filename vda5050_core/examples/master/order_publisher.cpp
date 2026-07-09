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

#include <atomic>
#include <csignal>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"
#include "vda5050_core/types/blocking_type.hpp"
#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/node.hpp"
#include "vda5050_core/types/node_position.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"

using vda5050_core::execution::ProtocolAdapter;
using vda5050_core::types::Action;
using vda5050_core::types::ActionParameter;
using vda5050_core::types::BlockingType;
using vda5050_core::types::Connection;
using vda5050_core::types::ConnectionState;
using vda5050_core::types::Edge;
using vda5050_core::types::Factsheet;
using vda5050_core::types::InstantActions;
using vda5050_core::types::Node;
using vda5050_core::types::NodePosition;
using vda5050_core::types::Order;
using vda5050_core::types::State;

std::atomic_bool running{true};

void signal_handler(int signal)
{
  VDA5050_INFO_STREAM(
    "System Signal [" << signal << "] received. Shutting down ...");
  running = false;
}

Node create_node(uint32_t i, bool released)
{
  NodePosition node_position;
  node_position.x = static_cast<double>(i) * 1.0;
  node_position.y = static_cast<double>(i) * 0.5;
  node_position.theta = 0.0;
  node_position.map_id = "map_1";

  Node node;
  node.node_id = "N" + std::to_string(i / 2);
  node.sequence_id = i;
  node.released = released;
  node.node_position = node_position;

  return node;
}

Edge create_edge(uint32_t i, bool released)
{
  Edge edge;
  edge.edge_id = "E" + std::to_string((i - 1) / 2);
  edge.sequence_id = i;
  edge.start_node_id = "N" + std::to_string((i - 1) / 2);
  edge.end_node_id = "N" + std::to_string((i + 1) / 2);
  edge.released = released;

  return edge;
}

Action create_action(
  const std::string& action_id, const std::string& action_type,
  std::optional<std::vector<ActionParameter>> action_parameters = std::nullopt)
{
  Action action;
  action.action_id = action_id;
  action.action_type = action_type;
  action.blocking_type = BlockingType::NONE;
  action.action_parameters = action_parameters;

  return action;
}

Order create_order(uint32_t order_update_id = 0)
{
  Order order;
  order.order_id = "test_order";
  order.order_update_id = order_update_id;

  uint32_t start_seq = (order_update_id == 0) ? 0 : (order_update_id - 1) * 6;
  uint32_t end_seq = order_update_id * 6;

  for (uint32_t i = start_seq; i <= end_seq; i++)
  {
    if (i % 2 == 0)
      order.nodes.push_back(create_node(i, true));
    else
      order.edges.push_back(create_edge(i, true));
  }

  order.edges.push_back(create_edge(end_seq + 1, false));
  order.nodes.push_back(create_node(end_seq + 2, false));
  order.edges.push_back(create_edge(end_seq + 3, false));
  order.nodes.push_back(create_node(end_seq + 4, false));

  return order;
}

InstantActions make_factsheet_request()
{
  InstantActions ia;
  ia.actions.push_back(create_action("factsheet_req", "factsheetRequest"));
  return ia;
}

InstantActions make_state_request()
{
  InstantActions ia;
  ia.actions.push_back(create_action("state_req", "stateRequest"));
  return ia;
}

InstantActions make_init_position()
{
  std::vector<ActionParameter> params = {
    {"x", "0.0"},
    {"y", "0.0"},
    {"theta", "0.0"},
    {"mapId", "map_1"},
    {"lastNodeId", "N0"}};

  InstantActions ia;
  ia.actions.push_back(create_action("init_pos", "initPosition", params));
  return ia;
}

enum class MasterState
{
  WAIT_CONNECTION,

  SEND_FACTSHEET_REQUEST,
  WAIT_FACTSHEET,

  SEND_STATE_REQUEST,
  WAIT_UNINITIALIZED_STATE,

  SEND_INIT_POSITION,
  WAIT_INITIALIZED_STATE,

  SEND_ORDER,
  EXECUTING,
};

struct MasterContext
{
  std::mutex mutex;
  MasterState state{MasterState::WAIT_CONNECTION};
  uint32_t order_update_id{0};
  uint32_t last_node_sequence_id{0};
  bool order_active{false};
};

int main()
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  MasterContext context;

  auto mqtt_client = vda5050_core::transport::create_default_client_unique(
    "tcp://localhost:1883", "order_publisher");
  auto protocol_adapter = ProtocolAdapter::make(
    std::move(mqtt_client), "uagv", "2.0.0", "Manufacturer", "S001");

  protocol_adapter->connect();

  protocol_adapter->subscribe<Connection>(
    [&context](auto message, auto error) {
      if (error.has_value()) return;

      VDA5050_INFO(
        "Received connection request from {}", message.header.serial_number);

      if (message.connection_state == ConnectionState::ONLINE)
      {
        std::lock_guard<std::mutex> lock(context.mutex);
        context.state = MasterState::SEND_FACTSHEET_REQUEST;
      }
    },
    0);

  protocol_adapter->subscribe<Factsheet>(
    [&context](auto message, auto error) {
      if (error.has_value()) return;

      VDA5050_INFO("Received factsheet from {}", message.header.serial_number);

      std::lock_guard<std::mutex> lock(context.mutex);
      context.state = MasterState::SEND_STATE_REQUEST;
    },
    0);

  protocol_adapter->subscribe<State>(
    [&context](auto message, auto error) {
      if (error.has_value()) return;

      std::lock_guard<std::mutex> lock(context.mutex);

      bool uninitialized = !message.agv_position.has_value() ||
                           !message.agv_position->position_initialized;

      if (context.state == MasterState::WAIT_UNINITIALIZED_STATE)
      {
        if (uninitialized)
          context.state = MasterState::SEND_INIT_POSITION;
        else
          context.state = MasterState::SEND_ORDER;
      }

      bool initialized = message.agv_position.has_value() &&
                         message.agv_position->position_initialized;
      if (context.state == MasterState::WAIT_INITIALIZED_STATE && initialized)
      {
        context.state = MasterState::SEND_ORDER;
        return;
      }

      if (context.state == MasterState::EXECUTING)
      {
        if (message.last_node_sequence_id == context.last_node_sequence_id)
        {
          context.state = MasterState::SEND_ORDER;
        }
      }
    },
    0);

  while (running)
  {
    {
      std::lock_guard<std::mutex> lock(context.mutex);
      switch (context.state)
      {
        case MasterState::WAIT_CONNECTION:
          break;

        case MasterState::SEND_FACTSHEET_REQUEST:
        {
          protocol_adapter->publish<InstantActions>(
            make_factsheet_request(), 0);
          context.state = MasterState::WAIT_FACTSHEET;
          break;
        }

        case MasterState::SEND_STATE_REQUEST:
        {
          protocol_adapter->publish<InstantActions>(make_state_request(), 0);
          context.state = MasterState::WAIT_UNINITIALIZED_STATE;
          break;
        }

        case MasterState::SEND_INIT_POSITION:
        {
          protocol_adapter->publish<InstantActions>(make_init_position(), 0);
          context.state = MasterState::WAIT_INITIALIZED_STATE;
          break;
        }

        case MasterState::SEND_ORDER:
        {
          protocol_adapter->publish<Order>(
            create_order(context.order_update_id), 0);
          context.state = MasterState::EXECUTING;
          context.last_node_sequence_id = context.order_update_id * 6;
          ++context.order_update_id;
          break;
        }

        case MasterState::WAIT_FACTSHEET:
        case MasterState::WAIT_UNINITIALIZED_STATE:
        case MasterState::WAIT_INITIALIZED_STATE:
        case MasterState::EXECUTING:
          break;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  protocol_adapter->unsubscribe<Connection>();
  protocol_adapter->unsubscribe<Factsheet>();
  protocol_adapter->unsubscribe<State>();
  protocol_adapter->disconnect();

  return 0;
}
