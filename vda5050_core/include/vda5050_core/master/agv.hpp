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

#ifndef VDA5050_CORE__MASTER__AGV_HPP_
#define VDA5050_CORE__MASTER__AGV_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>

#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/connection_state.hpp"
#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/logger/logger.hpp"

#include "vda5050_core/master/heartbeat.hpp"
#include "vda5050_core/master/standard_names.hpp"

using vda5050_core::types::Connection;
using vda5050_core::types::ConnectionState;
using vda5050_core::types::Error;
using vda5050_core::types::Factsheet;
using vda5050_core::types::InstantActions;
using vda5050_core::types::Order;
using vda5050_core::types::State;
using vda5050_core::types::Visualization;

using vda5050_core::execution::ProtocolAdapter;

namespace vda5050_core {

namespace master {

// Forward declaration
class VDA5050Master;

/**
 * @brief AGV operational state based on state heartbeat
 */
enum class AGVState
{
  STATE_UNKNOWN,  // Initial state or state heartbeat timed out
  AVAILABLE,      // State heartbeat is being received, AGV operational
  UNAVAILABLE,    // AGV reported unavailable or connection lost
  ERROR           // AGV reported error state
};

/**
 * @brief Represents an individual AGV managed by VDA5050Master
 *
 * This class is primarily a data container that holds:
 * - Identity information (manufacturer, serial number)
 * - Cached VDA5050 messages (connection, state, factsheet, visualization)
 * - Connection and operational state
 * - Outgoing message queue for orders and instant actions
 *
 * The VDA5050Master routes incoming messages to AGV instances and the AGV
 * handles incoming/outgoing messages via the VDA5050Execution ProtocolAdapter.
 *
 * Thread safety: Methods are thread-safe. Cached data access is protected
 * by mutexes.
 */
class AGV : public std::enable_shared_from_this<AGV>
{
public:
  // Type aliases
  using Clock = std::chrono::system_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  /**
   * @brief Construct an AGV instance
   *
   * Caller must invoke setup_subscriptions() after make_shared
   * returns; weak_from_this() is only valid once the shared_ptr
   * has been associated.
   *
   * @param protocol_adapter Protocol adapter for pub/sub
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @param max_queue_size Maximum number of outgoing messages to queue (default: 10)
   * @param drop_oldest If true, drop oldest message when queue full; if false, reject new (default: true)
   * @param state_heartbeat_interval State heartbeat timeout in seconds
   * @param parent Optional non-owning back-pointer to the owning
   *        VDA5050Master. When set, the AGV dispatches incoming
   *        messages to the master's virtual callbacks (on_state,
   *        on_connection, on_factsheet, on_visualization) after
   *        caching them. Held as a `weak_ptr` so the AGV can detect
   *        if the master is gone (returns null cleanly via `lock()`)
   *        rather than silently dangling. The master MUST be
   *        constructed via `std::make_shared<MyMaster>(...)` for the
   *        weak_ptr to be valid — see VDA5050Master class
   *        doc-comment.
   */
  AGV(
    std::shared_ptr<execution::ProtocolAdapter> protocol_adapter,
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true,
    int state_heartbeat_interval = StateHeartbeatInterval,
    std::weak_ptr<VDA5050Master> parent = {});

  /**
   * @brief Destructor - stops the queue processing thread
   */
  ~AGV();

  // Non-copyable, non-movable (due to thread member)
  AGV(const AGV&) = delete;
  AGV& operator=(const AGV&) = delete;
  AGV(AGV&&) = delete;
  AGV& operator=(AGV&&) = delete;

  // ============================================================================
  // Identity
  // ============================================================================

  /**
   * @brief Get the interface name
   */
  const std::string& get_interface_name() const
  {
    return interface_name_;
  }
  /**
   * @brief Get the manufacturer name
   */
  const std::string& get_manufacturer() const
  {
    return manufacturer_;
  }

  /**
   * @brief Get the serial number
   */
  const std::string& get_serial_number() const
  {
    return serial_number_;
  }

  /**
   * @brief Get the AGV ID (manufacturer/serial_number)
   */
  const std::string& get_agv_id() const
  {
    return agv_id_;
  }

  // ============================================================================
  // Connection and Operational State
  // ============================================================================

  /**
   * @brief Check if the AGV is connected (based on VDA5050 connection message)
   * @return true if connection_status is ONLINE, false otherwise
   */
  bool is_connected() const;

  /**
   * @brief Get the AGV connection state (based on VDA5050 connection message)
   * @return ONLINE, OFFLINE, or CONNECTIONBROKEN
   */
  ConnectionState get_connection_status() const;

  /**
   * @brief Get the AGV operational state (based on state heartbeat)
   * @return STATE_UNKNOWN, AVAILABLE, UNAVAILABLE, or ERROR
   */
  AGVState get_operational_state() const;

  /**
   * @brief Stop the AGV, releasing all runtime resources (heartbeat, queue processor)
   *
   * Stops the queue processor and heartbeat, resets connection and operational state,
   * and clears all message queues. Cached messages are preserved.
   * The AGV can be restarted later with start().
   */
  void stop();

  /**
   * @brief Restart the AGV, fully resetting state to accept new connections
   *
   * Calls stop(), then clears cached messages and timestamps.
   * The heartbeat and queue processor will be started automatically when an
   * ONLINE connection message is received.
   */
  void restart();

  /**
   * @brief Pause the AGV, suspending runtime resources without clearing queues
   *
   * Stops the queue processor and heartbeat, sets connection status to OFFLINE
   * and operational state to UNAVAILABLE. Queued messages and cached data are
   * preserved and will be processed when resumed.
   */
  void pause();

  /**
   * @brief Resume a paused AGV, restarting the queue processor and heartbeat
   *
   * Restarts the queue processor and heartbeat so the AGV can resume
   * publishing queued messages and monitoring state heartbeats.
   */
  void resume();

  // ============================================================================
  // Cached Messages (read-only access)
  // ============================================================================

  /**
   * @brief Get the last received connection message
   * @return Optional containing the message if received, nullopt otherwise
   */
  std::optional<Connection> get_last_connection() const;

  /**
   * @brief Get the last received state message
   * @return Optional containing the message if received, nullopt otherwise
   */
  std::optional<State> get_last_state() const;

  /**
   * @brief Get the last received factsheet message
   * @return Optional containing the message if received, nullopt otherwise
   */
  std::optional<Factsheet> get_last_factsheet() const;

  /**
   * @brief Get the last received visualization message
   * @return Optional containing the message if received, nullopt otherwise
   */
  std::optional<Visualization> get_last_visualization() const;

  // ============================================================================
  // Timestamps
  // ============================================================================

  /**
   * @brief Get the time when the AGV was created
   */
  TimePoint get_created_time() const
  {
    return created_time_;
  }

  /**
   * @brief Get the time of the last received connection message
   * @return Optional containing the timestamp if received, nullopt otherwise
   */
  std::optional<TimePoint> get_last_connection_time() const;

  /**
   * @brief Get the time of the last received state message
   * @return Optional containing the timestamp if received, nullopt otherwise
   */
  std::optional<TimePoint> get_last_state_time() const;

  /**
   * @brief Get the time of the last received factsheet message
   * @return Optional containing the timestamp if received, nullopt otherwise
   */
  std::optional<TimePoint> get_last_factsheet_time() const;

  /**
   * @brief Get the time of the last received visualization message
   * @return Optional containing the timestamp if received, nullopt otherwise
   */
  std::optional<TimePoint> get_last_visualization_time() const;

  // ============================================================================
  // Outgoing Messages
  // ============================================================================

  /**
   * @brief Queue an order to be sent to this AGV
   * @param order The order message
   * @return true if queued successfully, false if queue is full (drop_oldest=false)
   */
  bool send_order(const Order& order);

  /**
   * @brief Queue instant actions to be sent to this AGV
   * @param actions The instant actions message
   * @return true if queued successfully, false if queue is full (drop_oldest=false)
   */
  bool send_instant_actions(const InstantActions& actions);

  /**
   * @brief Get the number of pending orders in the queue
   * @return Number of orders waiting to be sent
   */
  size_t get_pending_order_count() const;

  /**
   * @brief Get the number of pending instant actions in the queue
   * @return Number of instant actions waiting to be sent
   */
  size_t get_pending_instant_actions_count() const;

  // ============================================================================
  // Message Handlers (called by VDA5050Master to route incoming messages)
  // ============================================================================

  /**
   * @brief Handle an incoming connection message
   * @param msg The parsed connection message
   */
  void handle_connection(const Connection& msg);

  /**
   * @brief Handle an incoming state message
   * @param msg The parsed state message
   */
  void handle_state(const State& msg);

  /**
   * @brief Handle an incoming factsheet message
   * @param msg The parsed factsheet message
   */
  void handle_factsheet(const Factsheet& msg);

  /**
   * @brief Handle an incoming visualization message
   * @param msg The parsed visualization message
   */
  void handle_visualization(const Visualization& msg);

  // ============================================================================
  // Subscription Management
  // ============================================================================

  /**
   * @brief Wire per-topic subscriptions on the protocol adapter.
   *
   * Must be called by the caller of the constructor after
   * `make_shared<AGV>(...)` returns — the wrapper lambda captures
   * `weak_from_this()`, which is only valid once the shared_ptr
   * ownership has been associated. Calling from inside the
   * constructor would silently install wrappers with empty
   * weak_ptrs, and user callbacks would never fire.
   */
  void setup_subscriptions();

private:
  // Wire a typed subscription on protocol_adapter_. The wrapper
  // captures weak_from_this() so the lambda no-ops cleanly if AGV
  // is destroyed before the wrapper fires (instead of dereferencing
  // a dangling pointer). Lock at the top keeps AGV alive for the
  // entire dispatch — both `self->agv_id_` and `handler(msg)`
  // (which captures [this]) are safe inside the locked scope.
  // Logs parse errors at ERROR level and exceptions thrown by
  // `handler` at WARN level without re-throwing — both are
  // non-fatal for the AGV.
  template <typename MsgType>
  void create_subscription(
    std::function<void(const MsgType&)> handler, QosLevel qos)
  {
    protocol_adapter_->template subscribe<MsgType>(
      [self_weak = weak_from_this(), handler = std::move(handler)](
        MsgType msg, std::optional<Error> error) {
        auto self = self_weak.lock();
        if (!self) return;  // AGV gone — drop the message silently

        if (error.has_value())
        {
          VDA5050_ERROR(
            "[AGV] Failed to parse message for {}: {}", self->agv_id_,
            error->error_description.value_or("unknown error"));
          return;
        }
        try
        {
          handler(msg);
        }
        catch (const std::exception& e)
        {
          VDA5050_WARN(
            "[AGV] Failed to handle message for {}: {}", self->agv_id_,
            e.what());
        }
      },
      static_cast<int>(qos));
  }

  // ============================================================================
  // Internal State Management
  // ============================================================================

  void set_connection_status(ConnectionState status);
  void set_operational_state(AGVState state);
  void on_state_heartbeat_timeout();

  // Setup/cleanup heartbeat when connection state changes
  void setup_heartbeat();
  void cleanup_heartbeat();

  // ============================================================================
  // Queue Processing
  // ============================================================================

  void start_queue_processor();
  void stop_queue_processor();
  void process_queues();

  // Publishing
  void publish_order(const Order& order);
  void publish_instant_actions(const InstantActions& actions);

  // Helper to build topic paths
  std::string build_topic(const std::string& topic_name) const;

  // ============================================================================
  // Member Variables
  // ============================================================================

  // Identity
  std::string interface_name_;
  std::string manufacturer_;
  std::string serial_number_;
  std::string agv_id_;

  // Protocol Adapter for publishing/subscribing
  std::shared_ptr<ProtocolAdapter> protocol_adapter_;

  // Non-owning back-pointer to the owning VDA5050Master.
  // Set at construction and never reassigned — safe to read concurrently
  // from MQTT-callback thread inside handle_*() dispatch.
  // Stored as weak_ptr so dispatch sites can detect master destruction
  // cleanly via lock() rather than silently dangling.
  std::weak_ptr<VDA5050Master> parent_;

  // Heartbeat listener for state timeout detection (protected by heartbeat_mutex_)
  mutable std::mutex heartbeat_mutex_;
  std::unique_ptr<HeartbeatListener> state_heartbeat_;
  int state_heartbeat_interval_;

  // AGV states (protected by state_mutex_)
  mutable std::mutex state_mutex_;
  ConnectionState connection_status_{ConnectionState::OFFLINE};
  AGVState operational_state_{AGVState::STATE_UNKNOWN};

  // Timestamps
  TimePoint created_time_;

  // Cached messages and timestamps (protected by data_mutex_)
  mutable std::mutex data_mutex_;

  std::optional<Connection> last_connection_;
  std::optional<TimePoint> last_connection_time_;

  std::optional<State> last_state_;
  std::optional<TimePoint> last_state_time_;

  std::optional<Factsheet> last_factsheet_;
  std::optional<TimePoint> last_factsheet_time_;

  std::optional<Visualization> last_visualization_;
  std::optional<TimePoint> last_visualization_time_;

  // Outgoing message queues (protected by queue_mutex_)
  size_t max_queue_size_;
  bool drop_oldest_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<Order> order_queue_;
  std::queue<InstantActions> instant_actions_queue_;

  // Queue processing thread
  std::mutex thread_mutex_;
  bool stop_processing_{false};
  bool queue_processor_running_{false};
  std::thread queue_thread_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__AGV_HPP_
