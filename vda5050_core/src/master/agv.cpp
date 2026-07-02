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

#include <utility>

#include "vda5050_core/logger/logger.hpp"

#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/master/standard_names.hpp"

namespace vda5050_core {

namespace master {

//=============================================================================
AGV::AGV(
  std::shared_ptr<ProtocolAdapter> protocol_adapter,
  const std::string& interface_name, const std::string& manufacturer,
  const std::string& serial_number, size_t max_queue_size, bool drop_oldest,
  int state_heartbeat_interval, std::weak_ptr<VDA5050Master> parent)
: interface_name_(interface_name),
  manufacturer_(manufacturer),
  serial_number_(serial_number),
  agv_id_(manufacturer + "/" + serial_number),
  protocol_adapter_(protocol_adapter),
  parent_(std::move(parent)),
  state_heartbeat_interval_(state_heartbeat_interval),
  created_time_(Clock::now()),
  max_queue_size_(max_queue_size),
  drop_oldest_(drop_oldest)
{
  VDA5050_INFO("[AGV] Created AGV instance: {}", agv_id_);
  // setup_subscriptions() must be called by the constructor's caller
  // after make_shared returns — weak_from_this() is only valid once
  // the shared_ptr ownership has been associated.
}

//=============================================================================
AGV::~AGV()
{
  VDA5050_INFO("[AGV] Destroying AGV instance: {}", agv_id_);

  // Teardown order matters (per CLAUDE.md):
  //   1. Stop spinning   — halt the queue processor and heartbeat threads
  //   2. Release resources — unsubscribe per-topic; drop protocol_adapter_
  //   3. Join threads     — handled inside stop_queue_processor() and
  //                         cleanup_heartbeat() above

  // 1. Stop spinning
  stop_queue_processor();
  cleanup_heartbeat();

  // 2. Release resources — unsubscribe via ProtocolAdapter so the broker
  // stops routing to lambdas captured at subscribe time, then drop the
  // shared_ptr. The underlying MqttClient stays alive (master owns it);
  // only the per-AGV typed wrapper goes away.
  if (protocol_adapter_)
  {
    protocol_adapter_->unsubscribe<Connection>();
    protocol_adapter_->unsubscribe<State>();
    protocol_adapter_->unsubscribe<Factsheet>();
    protocol_adapter_->unsubscribe<Visualization>();
  }
  protocol_adapter_.reset();

  VDA5050_INFO("[AGV] AGV instance destroyed: {}", agv_id_);
}

//=============================================================================
void AGV::setup_subscriptions()
{
  if (!protocol_adapter_)
  {
    return;
  }

  create_subscription<Connection>(
    [this](const auto& msg) { handle_connection(msg); }, ConnectionQos);
  create_subscription<State>(
    [this](const auto& msg) { handle_state(msg); }, StateQos);
  create_subscription<Factsheet>(
    [this](const auto& msg) { handle_factsheet(msg); }, FactsheetQos);
  create_subscription<Visualization>(
    [this](const auto& msg) { handle_visualization(msg); }, VisualizationQos);
}

//=============================================================================
void AGV::stop()
{
  VDA5050_INFO("[AGV] Stopping AGV: {}", agv_id_);

  // Stop queue processor and heartbeat
  stop_queue_processor();
  cleanup_heartbeat();

  // Reset states
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connection_status_ = ConnectionState::OFFLINE;
    operational_state_ = AGVState::STATE_UNKNOWN;
  }

  // Clear message queues
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    order_queue_ = {};
    instant_actions_queue_ = {};
  }

  VDA5050_INFO("[AGV] AGV stopped: {}", agv_id_);
}

//=============================================================================
void AGV::restart()
{
  VDA5050_INFO("[AGV] Restarting AGV: {}", agv_id_);

  stop();

  // Clear cached messages and timestamps
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_connection_.reset();
    last_connection_time_.reset();
    last_state_.reset();
    last_state_time_.reset();
    last_factsheet_.reset();
    last_factsheet_time_.reset();
    last_visualization_.reset();
    last_visualization_time_.reset();
  }

  VDA5050_INFO("[AGV] AGV restarted, ready for connections: {}", agv_id_);
}

//=============================================================================
void AGV::pause()
{
  VDA5050_INFO("[AGV] Pausing AGV: {}", agv_id_);

  stop_queue_processor();
  cleanup_heartbeat();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connection_status_ = ConnectionState::OFFLINE;
    operational_state_ = AGVState::UNAVAILABLE;
  }

  VDA5050_INFO("[AGV] AGV paused: {}", agv_id_);
}

//=============================================================================
void AGV::resume()
{
  VDA5050_INFO("[AGV] Resuming AGV: {}", agv_id_);

  setup_heartbeat();
  start_queue_processor();

  VDA5050_INFO("[AGV] AGV resumed: {}", agv_id_);
}

//=============================================================================
bool AGV::is_connected() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_status_ == ConnectionState::ONLINE;
}

//=============================================================================
ConnectionState AGV::get_connection_status() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_status_;
}

//=============================================================================
AGVState AGV::get_operational_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return operational_state_;
}

//=============================================================================
void AGV::set_connection_status(ConnectionState status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto old_status = connection_status_;
  connection_status_ = status;

  // When connection is lost, AGV becomes unavailable
  if (
    status == ConnectionState::OFFLINE ||
    status == ConnectionState::CONNECTIONBROKEN)
  {
    if (operational_state_ != AGVState::UNAVAILABLE)
    {
      operational_state_ = AGVState::UNAVAILABLE;
      VDA5050_INFO(
        "[AGV] Operational state changed to UNAVAILABLE for {} (connection {})",
        agv_id_,
        status == ConnectionState::OFFLINE ? "OFFLINE" : "CONNECTIONBROKEN");
    }
  }

  // Log connection status change
  if (old_status != status)
  {
    const char* status_str = "UNKNOWN";
    switch (status)
    {
      case ConnectionState::ONLINE:
        status_str = "ONLINE";
        break;
      case ConnectionState::OFFLINE:
        status_str = "OFFLINE";
        break;
      case ConnectionState::CONNECTIONBROKEN:
        status_str = "CONNECTIONBROKEN";
        break;
    }
    VDA5050_INFO(
      "[AGV] Connection status changed to {} for {}", status_str, agv_id_);
  }
}

//=============================================================================
void AGV::set_operational_state(AGVState state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  operational_state_ = state;

  const char* state_str = "UNKNOWN";
  switch (state)
  {
    case AGVState::STATE_UNKNOWN:
      state_str = "STATE_UNKNOWN";
      break;
    case AGVState::AVAILABLE:
      state_str = "AVAILABLE";
      break;
    case AGVState::UNAVAILABLE:
      state_str = "UNAVAILABLE";
      break;
    case AGVState::ERROR:
      state_str = "ERROR";
      break;
  }
  VDA5050_INFO(
    "[AGV] Operational state changed to {} for {}", state_str, agv_id_);
}

//=============================================================================
void AGV::on_state_heartbeat_timeout()
{
  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    if (!state_heartbeat_)
    {
      return;
    }
  }
  set_operational_state(AGVState::STATE_UNKNOWN);
  VDA5050_WARN("[AGV] State heartbeat timeout for {}", agv_id_);
}

//=============================================================================
void AGV::setup_heartbeat()
{
  std::lock_guard<std::mutex> lock(heartbeat_mutex_);

  if (state_heartbeat_)
  {
    return;  // Already set up
  }

  VDA5050_INFO("[AGV] Setting up heartbeat for {}", agv_id_);

  state_heartbeat_ = std::make_unique<HeartbeatListener>(
    agv_id_ + "_state_heartbeat", state_heartbeat_interval_,
    [this]() { on_state_heartbeat_timeout(); });
  state_heartbeat_->start_connection_heartbeat();
}

//=============================================================================
void AGV::cleanup_heartbeat()
{
  std::unique_ptr<HeartbeatListener> heartbeat_to_stop;

  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    if (!state_heartbeat_)
    {
      return;  // Nothing to clean up
    }

    VDA5050_INFO("[AGV] Cleaning up heartbeat for {}", agv_id_);

    heartbeat_to_stop = std::move(state_heartbeat_);
  }

  heartbeat_to_stop->stop_connection_heartbeat();
}

//=============================================================================
void AGV::handle_connection(const Connection& msg)
{
  // Update cached message
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_connection_ = msg;
    last_connection_time_ = Clock::now();
  }

  // Update connection status
  set_connection_status(msg.connection_state);

  // Manage heartbeat based on connection state
  if (msg.connection_state == ConnectionState::ONLINE)
  {
    // Start heartbeat and queue processor when ONLINE
    setup_heartbeat();
    start_queue_processor();
  }
  else
  {
    // Stop heartbeat and queue processor when OFFLINE/CONNECTIONBROKEN
    cleanup_heartbeat();
    stop_queue_processor();
  }

  // Dispatch to user override
  if (auto p = parent_.lock())
  {
    p->on_connection(agv_id_, msg);
  }
}

//=============================================================================
void AGV::handle_state(const State& msg)
{
  // Update cached message
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_state_ = msg;
    last_state_time_ = Clock::now();
  }

  // Notify heartbeat listener
  {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    if (state_heartbeat_)
    {
      state_heartbeat_->received_connection();
    }
  }

  // Update operational state to AVAILABLE
  set_operational_state(AGVState::AVAILABLE);

  // Dispatch to user override
  if (auto p = parent_.lock())
  {
    p->on_state(agv_id_, msg);
  }
}

//=============================================================================
void AGV::handle_factsheet(const Factsheet& msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_factsheet_ = msg;
    last_factsheet_time_ = Clock::now();
  }

  // Dispatch to user override
  if (auto p = parent_.lock())
  {
    p->on_factsheet(agv_id_, msg);
  }
}

//=============================================================================
void AGV::handle_visualization(const Visualization& msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_visualization_ = msg;
    last_visualization_time_ = Clock::now();
  }

  // Dispatch to user override
  if (auto p = parent_.lock())
  {
    p->on_visualization(agv_id_, msg);
  }
}

//=============================================================================
std::optional<Connection> AGV::get_last_connection() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_connection_;
}

//=============================================================================
std::optional<State> AGV::get_last_state() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_;
}

//=============================================================================
std::optional<Factsheet> AGV::get_last_factsheet() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_factsheet_;
}

//=============================================================================
std::optional<Visualization> AGV::get_last_visualization() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_visualization_;
}

//=============================================================================
std::optional<AGV::TimePoint> AGV::get_last_connection_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_connection_time_;
}

//=============================================================================
std::optional<AGV::TimePoint> AGV::get_last_state_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_time_;
}

//=============================================================================
std::optional<AGV::TimePoint> AGV::get_last_factsheet_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_factsheet_time_;
}

//=============================================================================
std::optional<AGV::TimePoint> AGV::get_last_visualization_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_visualization_time_;
}

//=============================================================================
bool AGV::send_order(const Order& order)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (order_queue_.size() >= max_queue_size_)
  {
    if (!drop_oldest_)
    {
      VDA5050_WARN(
        "[AGV] Dropping new order: queue full ({}/{}) for {}",
        order_queue_.size(), max_queue_size_, agv_id_);
      return false;
    }
    // Drop oldest order to make room
    VDA5050_WARN(
      "[AGV] Dropping oldest order: queue full ({}/{}) for {}",
      order_queue_.size(), max_queue_size_, agv_id_);
    order_queue_.pop();
  }

  order_queue_.push(order);
  queue_cv_.notify_one();

  VDA5050_INFO("[AGV] Queued order for AGV: {}", agv_id_);
  return true;
}

//=============================================================================
bool AGV::send_instant_actions(const InstantActions& actions)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (instant_actions_queue_.size() >= max_queue_size_)
  {
    if (!drop_oldest_)
    {
      VDA5050_WARN(
        "[AGV] Dropping new instant actions: queue full ({}/{}) for {}",
        instant_actions_queue_.size(), max_queue_size_, agv_id_);
      return false;
    }
    // Drop oldest instant actions to make room
    VDA5050_WARN(
      "[AGV] Dropping oldest instant actions: queue full ({}/{}) for {}",
      instant_actions_queue_.size(), max_queue_size_, agv_id_);
    instant_actions_queue_.pop();
  }

  instant_actions_queue_.push(actions);
  queue_cv_.notify_one();

  VDA5050_INFO("[AGV] Queued instant actions for AGV: {}", agv_id_);
  return true;
}

//=============================================================================
size_t AGV::get_pending_order_count() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return order_queue_.size();
}

//=============================================================================
size_t AGV::get_pending_instant_actions_count() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return instant_actions_queue_.size();
}

//=============================================================================
void AGV::start_queue_processor()
{
  std::lock_guard<std::mutex> lock(thread_mutex_);

  if (queue_processor_running_)
  {
    return;  // Already running
  }

  VDA5050_INFO("[AGV] Starting queue processor for {}", agv_id_);

  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    stop_processing_ = false;
  }
  queue_processor_running_ = true;
  queue_thread_ = std::thread(&AGV::process_queues, this);
}

//=============================================================================
void AGV::stop_queue_processor()
{
  std::thread thread_to_join;

  {
    std::lock_guard<std::mutex> lock(thread_mutex_);

    if (!queue_processor_running_)
    {
      return;  // Not running
    }

    VDA5050_INFO("[AGV] Stopping queue processor for {}", agv_id_);

    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      stop_processing_ = true;
    }
    queue_cv_.notify_one();

    thread_to_join = std::move(queue_thread_);
    queue_processor_running_ = false;
  }

  if (thread_to_join.joinable())
  {
    thread_to_join.join();
  }

  VDA5050_INFO("[AGV] Queue processor stopped for {}", agv_id_);
}

//=============================================================================
void AGV::process_queues()
{
  VDA5050_INFO("[AGV] Queue processing thread started for {}", agv_id_);

  while (true)
  {
    std::optional<Order> order;
    std::optional<InstantActions> actions;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      // Wait for a message or stop signal
      queue_cv_.wait(lock, [this] {
        return stop_processing_ || !order_queue_.empty() ||
               !instant_actions_queue_.empty();
      });

      // Check if we should stop
      if (
        stop_processing_ ||
        (order_queue_.empty() && instant_actions_queue_.empty()))
      {
        break;
      }

      // Process instant actions first (higher priority)
      if (!instant_actions_queue_.empty())
      {
        actions = std::move(instant_actions_queue_.front());
        instant_actions_queue_.pop();
      }
      else if (!order_queue_.empty())
      {
        order = std::move(order_queue_.front());
        order_queue_.pop();
      }
    }

    // Publish the message (outside the lock)
    if (actions)
    {
      publish_instant_actions(*actions);
    }
    else if (order)
    {
      publish_order(*order);
    }
  }

  VDA5050_INFO("[AGV] Queue processing thread stopped for {}", agv_id_);
}

//=============================================================================
void AGV::publish_order(const Order& order)
{
  if (!protocol_adapter_)
  {
    VDA5050_WARN(
      "[AGV] Cannot publish order: no protocol adapter for {}", agv_id_);
    return;
  }
  protocol_adapter_->publish<Order>(order, static_cast<int>(OrderQos));
}

//=============================================================================
void AGV::publish_instant_actions(const InstantActions& actions)
{
  if (!protocol_adapter_)
  {
    VDA5050_WARN(
      "[AGV] Cannot publish instant actions: no protocol adapter for {}",
      agv_id_);
    return;
  }
  protocol_adapter_->publish<InstantActions>(
    actions, static_cast<int>(InstantActionsQos));
}

//=============================================================================
std::string AGV::build_topic(const std::string& topic_name) const
{
  return interface_name_ + "/" + Version + "/" + manufacturer_ + "/" +
         serial_number_ + "/" + topic_name;
}

}  // namespace master
}  // namespace vda5050_core
