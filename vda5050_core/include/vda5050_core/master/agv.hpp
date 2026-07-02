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
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/actions/instant_actions_publisher.hpp"
#include "vda5050_core/master/heartbeat.hpp"
#include "vda5050_core/master/master_types.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/master/order/order_publisher.hpp"
#include "vda5050_core/master/order/order_stitcher.hpp"
#include "vda5050_core/master/pose_view.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/types/error.hpp"

namespace vda5050_core::master {

// Forward declaration
class VDA5050Master;

/// \brief AGV operational state based on state heartbeat
enum class AGVState
{
  STATE_UNKNOWN,  // Initial state or state heartbeat timed out
  AVAILABLE,      // State heartbeat is being received, AGV operational
  UNAVAILABLE,    // AGV reported unavailable or connection lost
  ERROR           // AGV reported error state
};

/// \brief Represents an individual AGV managed by VDA5050Master
///
/// This class is primarily a data container that holds:
/// - Identity information (manufacturer, serial number)
/// - Cached VDA5050 messages (connection, state, factsheet, visualization)
/// - Connection and operational state
/// - Outgoing message queue for orders and instant actions
///
/// The VDA5050Master routes incoming messages to AGV instances and the AGV
/// handles incoming/outgoing messages via the VDA5050Execution ProtocolAdapter.
///
/// Thread safety: Methods are thread-safe. Cached data access is protected
/// by mutexes.
class AGV : public std::enable_shared_from_this<AGV>
{
public:
  // Type aliases
  using Clock = std::chrono::system_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  /// \brief Construct an AGV instance
  ///
  /// Caller must invoke setup_subscriptions() after make_shared
  /// returns; weak_from_this() is only valid once the shared_ptr
  /// has been associated.
  ///
  /// \param protocol_adapter Protocol adapter for pub/sub
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param max_queue_size Maximum number of outgoing messages to queue (default: 10)
  /// \param drop_oldest If true, drop oldest message when queue full; if
  ///   false, reject new (default: true)
  /// \param state_heartbeat_interval State heartbeat timeout in seconds
  /// \param parent Optional non-owning back-pointer to the owning
  ///        VDA5050Master. When set, the AGV dispatches incoming
  ///        messages to the master's virtual callbacks (on_state,
  ///        on_connection, on_factsheet, on_visualization) after
  ///        caching them. Held as a `weak_ptr` so the AGV can detect
  ///        if the master is gone (returns null cleanly via `lock()`)
  ///        rather than silently dangling. The master MUST be
  ///        constructed via `std::make_shared<MyMaster>(...)` for the
  ///        weak_ptr to be valid — see VDA5050Master class
  ///        doc-comment.
  AGV(
    std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_adapter,
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true,
    int state_heartbeat_interval = StateHeartbeatInterval,
    std::weak_ptr<VDA5050Master> parent = {});

  /// \brief Destructor - stops the queue processing thread
  ~AGV();

  // Non-copyable, non-movable (due to thread member)
  AGV(const AGV&) = delete;
  AGV& operator=(const AGV&) = delete;
  AGV(AGV&&) = delete;
  AGV& operator=(AGV&&) = delete;

  // ============================================================================
  // Identity
  // ============================================================================

  /// \brief Get the interface name
  const std::string& get_interface_name() const
  {
    return interface_name_;
  }
  /// \brief Get the manufacturer name
  const std::string& get_manufacturer() const
  {
    return manufacturer_;
  }

  /// \brief Get the serial number
  const std::string& get_serial_number() const
  {
    return serial_number_;
  }

  /// \brief Get the AGV ID (manufacturer/serial_number)
  const std::string& get_agv_id() const
  {
    return agv_id_;
  }

  // ============================================================================
  // Connection and Operational State
  // ============================================================================

  /// \brief Check if the AGV is connected (based on VDA5050 connection message)
  /// \return true if connection_status is ONLINE, false otherwise
  bool is_connected() const;

  /// \brief Get the AGV connection state (based on VDA5050 connection message)
  /// \return ONLINE, OFFLINE, or CONNECTIONBROKEN
  vda5050_core::types::ConnectionState get_connection_status() const;

  /// \brief Get the AGV operational state (based on state heartbeat)
  /// \return STATE_UNKNOWN, AVAILABLE, UNAVAILABLE, or ERROR
  AGVState get_operational_state() const;

  /// \brief Stop the AGV, releasing all runtime resources (heartbeat, queue processor)
  ///
  /// Stops the queue processor and heartbeat, resets connection and operational state,
  /// and clears all message queues. Cached messages are preserved.
  /// The AGV can be restarted later with start().
  void stop();

  /// \brief Restart the AGV, fully resetting state to accept new connections
  ///
  /// Calls stop(), then clears cached messages and timestamps.
  /// The heartbeat and queue processor will be started automatically when an
  /// ONLINE connection message is received.
  void restart();

  /// \brief Pause the AGV, suspending runtime resources without clearing queues
  ///
  /// Stops the queue processor and heartbeat, sets connection status to OFFLINE
  /// and operational state to UNAVAILABLE. Queued messages and cached data are
  /// preserved and will be processed when resumed.
  void pause();

  /// \brief Resume a paused AGV, restarting the queue processor and heartbeat
  ///
  /// Restarts the queue processor and heartbeat so the AGV can resume
  /// publishing queued messages and monitoring state heartbeats.
  void resume();

  // ============================================================================
  // Cached Messages (read-only access)
  // ============================================================================

  /// \brief Get the last received connection message
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_core::types::Connection> get_last_connection() const;

  /// \brief Get the last received state message
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_core::types::State> get_last_state() const;

  /// \brief Get the last received factsheet message
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_core::types::Factsheet> get_last_factsheet() const;

  /// \brief Get the last received visualization message
  /// \return Optional containing the message if received, nullopt otherwise
  std::optional<vda5050_core::types::Visualization> get_last_visualization()
    const;

  /// \brief Atomic snapshot of all cached State / Connection / Factsheet
  ///        values + their receive timestamps.
  ///
  /// Sequential calls to `get_last_state()` / `get_last_connection()` /
  /// `get_last_factsheet()` each take and release `data_mutex_` separately,
  /// so the cache can advance between calls — producing inconsistent
  /// snapshots (e.g. State from t and Connection from t+1). For
  /// diagnostic queries that must see a coherent view of the AGV at a
  /// single instant, use this method instead.
  struct StatusSnapshot
  {
    std::optional<vda5050_core::types::State> state;
    std::optional<vda5050_core::types::Connection> connection;
    std::optional<vda5050_core::types::Factsheet> factsheet;
    std::optional<TimePoint> state_received_at;
    std::optional<TimePoint> connection_received_at;
    std::optional<TimePoint> factsheet_received_at;
  };

  /// \brief Get a coherent snapshot of all cached messages + timestamps.
  /// \return StatusSnapshot taken under a single data_mutex_ acquisition.
  StatusSnapshot get_status_snapshot() const;

  /// \brief Atomic bundle of cached State + master order-lifecycle view.
  ///
  /// Used by OrderStatusPublisher (called from on_state with the freshest
  /// State already cached) and OrderStatusService (synchronous query).
  /// Sequential calls to `get_last_state()` followed by
  /// `active_order_snapshot()` would allow the cache to advance between
  /// calls — for the service path, an executor-tick can elapse between
  /// the two reads. This bundle takes `data_mutex_` first then queries
  /// the OrderLifecycleManager (which takes `lifecycle_mutex_`
  /// internally), giving a microsecond-class drift window.
  struct OrderStatusBundle
  {
    std::optional<vda5050_core::types::State> state;
    std::optional<TimePoint> state_received_at;
    ActiveOrderSnapshot active_order_snapshot;
    std::size_t pending_stitch_count;
  };

  /// \brief Get a coherent snapshot of cached State + master lifecycle view.
  /// \return OrderStatusBundle suitable for OrderStatus message construction.
  OrderStatusBundle get_order_status_bundle() const;

  /// \brief Fused pose snapshot for the pose_view stream.
  ///
  /// Picks the freshest of the cached State / Visualization (by AGV header
  /// timestamp) that carries an agv_position, takes velocity from that same
  /// source, and relays driving from State — all under one data_mutex_
  /// acquisition. data_age is measured against the master receive time.
  /// \return PoseView; source == None when no position has been received.
  PoseView get_pose_view() const;

  // ============================================================================
  // Order Lifecycle (read-only forwarders to per-AGV OrderLifecycleManager)
  // ============================================================================

  /// \brief Whether the master is tracking an active order for this AGV.
  /// \return true once a successful publish has been recorded and not since
  ///         cleared by recovery / new-order / explicit clear.
  bool has_active_order() const;

  /// \brief order_id of the active tracked order, or nullopt.
  std::optional<std::string> active_order_id() const;

  /// \brief order_update_id of the active tracked order, or nullopt.
  std::optional<uint32_t> active_order_update_id() const;

  /// \brief True once the AGV's reported last_node matches the active
  ///        order's final node. Sticky until a new order is recorded.
  bool is_order_complete() const;

  /// \brief True when the AGV has reported newBaseRequest and master has
  ///        not yet recorded a strictly higher order_update_id (real
  ///        extension) for the active order.
  bool active_order_needs_more_base() const;

  /// \brief Number of order updates queued waiting for stitch conditions.
  size_t pending_update_count() const;

  /// \brief Snapshot of the active-order tracking state. Returned by value
  ///        — safe to read concurrently with state updates.
  ActiveOrderSnapshot active_order_snapshot() const;

  // ============================================================================
  // Timestamps
  // ============================================================================

  /// \brief Get the time when the AGV was created
  TimePoint get_created_time() const
  {
    return created_time_;
  }

  /// \brief Get the time of the last received connection message
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_connection_time() const;

  /// \brief Get the time of the last received state message
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_state_time() const;

  /// \brief Get the time of the last received factsheet message
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_factsheet_time() const;

  /// \brief Get the time of the last received visualization message
  /// \return Optional containing the timestamp if received, nullopt otherwise
  std::optional<TimePoint> get_last_visualization_time() const;

  // ============================================================================
  // Outgoing Messages
  // ============================================================================

  /// \brief Queue an order to be sent to this AGV
  /// \param order The order message
  /// \return true if queued successfully, false if queue is full (drop_oldest=false)
  bool send_order(const vda5050_core::types::Order& order);

  /// \brief Queue instant actions to be sent to this AGV
  /// \param actions The instant actions message
  /// \return true if queued successfully, false if queue is full (drop_oldest=false)
  bool send_instant_actions(const vda5050_core::types::InstantActions& actions);

  /// \brief Get the number of pending orders in the queue
  /// \return Number of orders waiting to be sent
  size_t get_pending_order_count() const;

  /// \brief Get the number of pending instant actions in the queue
  /// \return Number of instant actions waiting to be sent
  size_t get_pending_instant_actions_count() const;

  /// \brief Drop all queued outbound Orders and InstantActions.
  ///
  /// Master-side only — does NOT send a cancelOrder instant action to
  /// the AGV. Intended for use when the connection has gone broken and
  /// queued messages are no longer relevant. To abort an in-flight
  /// order that the AGV has already received, send a cancelOrder
  /// instant action via send_instant_actions().
  ///
  /// Thread-safe: takes queue_mutex_.
  void cancel_pending_orders();

  // ============================================================================
  // Mode-cancelled queue (capture-and-resume on mode change)
  // ============================================================================
  //
  // When the AGV transitions out of AUTOMATIC (per VDA5050 v2.0.0
  // the spec, the AGV stops executing its order and the
  // master shall not send orders / actions in MANUAL / SERVICE /
  // TEACHIN), the library captures whatever's pending in the AGV's
  // outbound order_queue_ + instant_actions_queue_ into a per-AGV
  // ModeCancelledQueue buffer and drains the live queues.
  //
  // The capture happens BEFORE the master's `on_mode_changed`
  // virtual fires, so FMS overrides observe the buffer already
  // populated. On the return-to-AUTOMATIC edge, the FMS chooses one
  // of:
  //   - resume_mode_cancelled_queue() — re-enqueue everything via
  //     the standard send_order / send_instant_actions paths
  //   - discard_mode_cancelled_queue() — drop the buffer
  //   - do nothing — buffer is overwritten on the next leave-AUTOMATIC
  //
  // Orders sent DURING non-AUTOMATIC are rejected at the existing
  // pre-send gate — they do NOT accumulate into the buffer.
  //
  // Each leave-AUTOMATIC OVERWRITES the buffer (does not accumulate).

  /// Snapshot of queue items captured at the most recent
  /// AUTOMATIC→non-AUTOMATIC mode transition.
  struct ModeCancelledQueue
  {
    std::vector<vda5050_core::types::Order> orders;
    std::vector<vda5050_core::types::InstantActions> instant_actions;
    std::optional<TimePoint> cancelled_at;
    std::optional<vda5050_core::types::OperatingMode> from_mode;
    std::optional<vda5050_core::types::OperatingMode> to_mode;
  };

  /// \brief Snapshot the mode-cancelled buffer. Empty when no
  ///        leave-AUTOMATIC has occurred since onboard / since the
  ///        last resume / discard call.
  ///
  /// Thread-safe: takes queue_mutex_; returns a copy.
  ModeCancelledQueue get_mode_cancelled_queue() const;

  /// \brief PREPEND every captured order + instant action to the
  ///        live queue (preserves original FMS-intended ordering)
  ///        and clear the buffer.
  ///
  /// Returns {orders_resumed, actions_resumed} — the count
  /// re-enqueued.
  ///
  /// **Ordering**: buffered items go to the FRONT of the live queue
  /// — they execute BEFORE any new orders FMS may have dispatched
  /// between AUTOMATIC return and this call. Example: buffer = (2,3)
  /// captured at leave-AUTOMATIC; AGV returns to AUTOMATIC; FMS
  /// sends order 4 (live queue = (4)); FMS calls resume → live queue
  /// becomes (2, 3, 4). AGV processes 2, 3, 4 in that order.
  ///
  /// Re-enqueued items run through the full validator chain at
  /// publish time, so any item that's no longer valid (e.g. AGV
  /// moved during MANUAL and the order's first node is no longer
  /// reachable) is rejected by the appropriate validator with a
  /// clear error log; the rest still flow.
  ///
  /// Thread-safe: takes queue_mutex_ once and does the prepend
  /// atomically (no nested-lock window).
  std::pair<std::size_t, std::size_t> resume_mode_cancelled_queue();

  /// \brief Drop the mode-cancelled buffer without re-enqueue.
  ///
  /// Returns {orders_discarded, actions_discarded}.
  ///
  /// Thread-safe: takes queue_mutex_.
  std::pair<std::size_t, std::size_t> discard_mode_cancelled_queue();

  // ============================================================================
  // Message Handlers (called by VDA5050Master to route incoming messages)
  // ============================================================================

  /// \brief Handle an incoming connection message
  /// \param msg The parsed connection message
  void handle_connection(const vda5050_core::types::Connection& msg);

  /// \brief Handle an incoming state message
  /// \param msg The parsed state message
  void handle_state(const vda5050_core::types::State& msg);

  /// \brief Handle an incoming factsheet message
  /// \param msg The parsed factsheet message
  void handle_factsheet(const vda5050_core::types::Factsheet& msg);

  /// \brief Handle an incoming visualization message
  /// \param msg The parsed visualization message
  void handle_visualization(const vda5050_core::types::Visualization& msg);

  // ============================================================================
  // Subscription Management
  // ============================================================================

  /// \brief Wire per-topic subscriptions on the protocol adapter.
  ///
  /// Must be called by the caller of the constructor after
  /// `make_shared<AGV>(...)` returns — the wrapper lambda captures
  /// `weak_from_this()`, which is only valid once the shared_ptr
  /// ownership has been associated. Calling from inside the
  /// constructor would silently install wrappers with empty
  /// weak_ptrs, and user callbacks would never fire.
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
        MsgType msg, std::optional<vda5050_core::types::Error> error) {
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

  void set_connection_status(vda5050_core::types::ConnectionState status);
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
  void publish_order(const vda5050_core::types::Order& order);
  void publish_instant_actions(
    const vda5050_core::types::InstantActions& actions);

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
  std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_adapter_;

  // Publishers — stateless today; will hold the validator chain
  // once it lands.
  OrderPublisher order_publisher_;
  InstantActionsPublisher instant_actions_publisher_;

  // Per-AGV order lifecycle tracking. Owns its own mutex; updated
  // from handle_state (incoming) and publish_order (outgoing).
  OrderLifecycleManager order_lifecycle_;

  // Stateless 4-condition stitch guard. Decides SEND_NOW /
  // QUEUE_PENDING / REJECT at the front of publish_order.
  OrderStitcher order_stitcher_;

  // Non-owning back-pointer to the owning VDA5050Master.
  // Set at construction and never reassigned — safe to read concurrently
  // from MQTT-callback thread inside handle_*() dispatch.
  // Stored as weak_ptr so dispatch sites can detect master destruction
  // cleanly via lock() rather than silently dangling.
  std::weak_ptr<VDA5050Master> parent_;

  // Raw observer pointer to the same master, used ONLY by the queue
  // processor thread inside publish_order / publish_instant_actions.
  // We can't use parent_.lock() there: the temporary shared_ptr would
  // extend master lifetime, and if it becomes the last reference (e.g.
  // the test fixture's master_ goes out of scope while a publish is
  // mid-flight), dropping the temp triggers ~master → ~AGV on the
  // queue thread, which self-joins. VDA5050Master's destructor stops
  // all AGV queue threads BEFORE its members destruct, so this raw
  // pointer is guaranteed valid for the duration of any publish().
  VDA5050Master* parent_raw_{nullptr};

  // Heartbeat listener for state timeout detection (protected by heartbeat_mutex_)
  mutable std::mutex heartbeat_mutex_;
  std::unique_ptr<HeartbeatListener> state_heartbeat_;
  int state_heartbeat_interval_;

  // AGV states (protected by state_mutex_)
  mutable std::mutex state_mutex_;
  vda5050_core::types::ConnectionState connection_status_{
    vda5050_core::types::ConnectionState::OFFLINE};
  AGVState operational_state_{AGVState::STATE_UNKNOWN};

  // Timestamps
  TimePoint created_time_;

  // Cached messages and timestamps (protected by data_mutex_)
  mutable std::mutex data_mutex_;

  std::optional<vda5050_core::types::Connection> last_connection_;
  std::optional<TimePoint> last_connection_time_;

  std::optional<vda5050_core::types::State> last_state_;
  std::optional<TimePoint> last_state_time_;

  std::optional<vda5050_core::types::Factsheet> last_factsheet_;
  std::optional<TimePoint> last_factsheet_time_;

  std::optional<vda5050_core::types::Visualization> last_visualization_;
  std::optional<TimePoint> last_visualization_time_;

  // Previous-message snapshots for stateless event detection. Touched
  // only from the MQTT-callback thread inside handle_state /
  // handle_connection — single writer/reader, so no mutex needed.
  std::optional<vda5050_core::types::State> prev_state_;
  std::optional<vda5050_core::types::Connection> prev_connection_;

  // Outgoing message queues (protected by queue_mutex_)
  size_t max_queue_size_;
  bool drop_oldest_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<vda5050_core::types::Order> order_queue_;
  std::queue<vda5050_core::types::InstantActions> instant_actions_queue_;

  // Mode-cancelled buffer. Protected by queue_mutex_ —
  // populated by capture_and_drain_on_leave_automatic_, drained by
  // resume_mode_cancelled_queue / discard_mode_cancelled_queue.
  ModeCancelledQueue mode_cancelled_queue_;

  // Capture the live queues into mode_cancelled_queue_ and drain
  // them. Called from handle_state on the AUTOMATIC→non-AUTOMATIC
  // edge BEFORE on_mode_changed dispatches to user.
  void capture_and_drain_on_leave_automatic(
    const vda5050_core::types::State& prev,
    const vda5050_core::types::State& curr);

  // Queue processing thread
  std::mutex thread_mutex_;
  bool stop_processing_{false};
  bool queue_processor_running_{false};
  std::thread queue_thread_;
};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__AGV_HPP_
