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
#include "vda5050_core/master/order/active_order_snapshot.hpp"
#include "vda5050_core/master/order/order_lifecycle_manager.hpp"
#include "vda5050_core/master/order/order_publisher.hpp"
#include "vda5050_core/master/order/order_stitcher.hpp"
#include "vda5050_core/master/pose_view.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/types/error.hpp"

namespace vda5050_core {
namespace master {

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

/// \brief An individual AGV managed by VDA5050Master: caches its messages,
///        tracks connection/operational state, and queues outbound messages.
///
/// Thread-safe; cached data is mutex-protected.
class AGV : public std::enable_shared_from_this<AGV>
{
public:
  // Type aliases
  using Clock = std::chrono::system_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  /// \brief Construct an AGV. Caller must invoke setup_subscriptions() after
  ///        make_shared returns (weak_from_this() needs the shared_ptr).
  /// \param max_queue_size Outgoing queue cap (default 10).
  /// \param drop_oldest Drop oldest vs reject-new when the queue is full.
  /// \param state_heartbeat_interval State heartbeat timeout in seconds.
  /// \param parent Non-owning weak back-pointer to the master; used to
  ///        dispatch cached messages to its virtual callbacks. Master must
  ///        be make_shared-constructed for it to be valid.
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

  // ===========================================================================
  // Identity
  // ===========================================================================

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

  // ===========================================================================
  // Connection and Operational State
  // ===========================================================================

  /// \brief True if connection_status is ONLINE.
  bool is_connected() const;

  /// \brief Connection state from the VDA5050 connection message.
  vda5050_core::types::ConnectionState get_connection_status() const;

  /// \brief Operational state derived from the state heartbeat.
  AGVState get_operational_state() const;

  /// \brief Stop the queue processor and heartbeat, reset state, and clear
  ///        queues. Cached messages are preserved; restartable via start().
  void stop();

  /// \brief stop(), then clear cached messages and timestamps. Restarts on
  ///        the next ONLINE connection message.
  void restart();

  /// \brief Suspend queue processor and heartbeat without clearing queues;
  ///        sets OFFLINE / UNAVAILABLE. Queued and cached data are preserved.
  void pause();

  /// \brief Restart the queue processor and heartbeat after pause().
  void resume();

  // ===========================================================================
  // Cached Messages (read-only access)
  // ===========================================================================

  /// \brief Last received connection message, or nullopt.
  std::optional<vda5050_core::types::Connection> get_last_connection() const;

  /// \brief Last received state message, or nullopt.
  std::optional<vda5050_core::types::State> get_last_state() const;

  /// \brief Last received factsheet message, or nullopt.
  std::optional<vda5050_core::types::Factsheet> get_last_factsheet() const;

  /// \brief Last received visualization message, or nullopt.
  std::optional<vda5050_core::types::Visualization> get_last_visualization()
    const;

  /// \brief Coherent snapshot of cached State / Connection / Factsheet plus
  ///        receive timestamps, taken under one data_mutex_ acquisition.
  struct StatusSnapshot
  {
    std::optional<vda5050_core::types::State> state;
    std::optional<vda5050_core::types::Connection> connection;
    std::optional<vda5050_core::types::Factsheet> factsheet;
    std::optional<TimePoint> state_received_at;
    std::optional<TimePoint> connection_received_at;
    std::optional<TimePoint> factsheet_received_at;
  };

  /// \brief Coherent snapshot of all cached messages + timestamps.
  StatusSnapshot get_status_snapshot() const;

  /// \brief Coherent bundle of cached State + master order-lifecycle view,
  ///        taken data_mutex_ first then the OrderLifecycleManager.
  struct OrderStatusBundle
  {
    std::optional<vda5050_core::types::State> state;
    std::optional<TimePoint> state_received_at;
    ActiveOrderSnapshot active_order_snapshot;
    std::size_t pending_stitch_count;
  };

  /// \brief Coherent snapshot of cached State + master lifecycle view.
  OrderStatusBundle get_order_status_bundle() const;

  /// \brief Fused pose snapshot from the freshest of cached State /
  ///        Visualization carrying a position, under one data_mutex_.
  /// \return PoseView; source == None when no position has been received.
  PoseView get_pose_view() const;

  // ===========================================================================
  // Order Lifecycle (read-only forwarders to per-AGV OrderLifecycleManager)
  // ===========================================================================

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

  // ===========================================================================
  // Timestamps
  // ===========================================================================

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

  // ===========================================================================
  // Outgoing Messages
  // ===========================================================================

  /// \brief Queue an order to be sent to this AGV
  /// \param order The order message
  /// \return true if queued, false if queue full (drop_oldest=false)
  bool send_order(const vda5050_core::types::Order& order);

  /// \brief Queue instant actions to be sent to this AGV
  /// \param actions The instant actions message
  /// \return true if queued, false if queue full (drop_oldest=false)
  bool send_instant_actions(const vda5050_core::types::InstantActions& actions);

  /// \brief Get the number of pending orders in the queue
  /// \return Number of orders waiting to be sent
  size_t get_pending_order_count() const;

  /// \brief Get the number of pending instant actions in the queue
  /// \return Number of instant actions waiting to be sent
  size_t get_pending_instant_actions_count() const;

  /// \brief Drop all queued outbound Orders and InstantActions (master-side
  ///        only; does not send a cancelOrder to the AGV). Thread-safe.
  void cancel_pending_orders();

  // ===========================================================================
  // Mode-cancelled queue (capture-and-resume on mode change)
  // ===========================================================================
  //
  // On the AUTOMATIC→non-AUTOMATIC edge the live outbound queues are captured
  // into ModeCancelledQueue and drained (the AGV won't execute while out of
  // AUTOMATIC). Capture runs BEFORE on_mode_changed so an FMS override sees the
  // buffer populated; on return the FMS calls resume/discard (or ignores it —
  // the next leave-AUTOMATIC overwrites it).

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

  /// \brief Prepend the captured buffer to the front of the live queue
  ///        (preserving FMS order) and clear it.
  /// \return {orders_resumed, actions_resumed}.
  ///
  /// Re-enqueued items run the full validator chain at publish; an invalid one
  /// is rejected with a clear error, the rest flow. Thread-safe.
  std::pair<std::size_t, std::size_t> resume_mode_cancelled_queue();

  /// \brief Drop the mode-cancelled buffer without re-enqueue.
  ///
  /// Returns {orders_discarded, actions_discarded}.
  ///
  /// Thread-safe: takes queue_mutex_.
  std::pair<std::size_t, std::size_t> discard_mode_cancelled_queue();

  // ===========================================================================
  // Message Handlers (called by VDA5050Master to route incoming messages)
  // ===========================================================================

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

  // ===========================================================================
  // Subscription Management
  // ===========================================================================

  /// \brief Wire per-topic subscriptions on the protocol adapter. Call after
  ///        make_shared returns — the wrappers capture weak_from_this().
  void setup_subscriptions();

private:
  // Wraps the subscription lambda to lock weak_from_this() first — that keeps
  // the AGV alive for the whole dispatch and no-ops cleanly if it is already
  // gone. Parse errors log at ERROR, handler exceptions at WARN; neither
  // re-throws (non-fatal for the AGV).
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

  // ===========================================================================
  // Internal State Management
  // ===========================================================================

  void set_connection_status(vda5050_core::types::ConnectionState status);
  void set_operational_state(AGVState state);
  void on_state_heartbeat_timeout();

  // Setup/cleanup heartbeat when connection state changes
  void setup_heartbeat();
  void cleanup_heartbeat();

  // ===========================================================================
  // Queue Processing
  // ===========================================================================

  void start_queue_processor();
  void stop_queue_processor();
  void process_queues();

  bool enqueue_order(
    const vda5050_core::types::Order& order, bool pre_stitched);

  // pre_stitched skips the stitch decision for a drained update.
  void publish_order(
    const vda5050_core::types::Order& order, bool pre_stitched = false);
  void publish_instant_actions(
    const vda5050_core::types::InstantActions& actions);

  // Helper to build topic paths
  std::string build_topic(const std::string& topic_name) const;

  // ===========================================================================
  // Member Variables
  // ===========================================================================

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

  // Stateless stitch guard. Decides SEND_NOW / QUEUE_PENDING / IGNORE /
  // REJECT at the front of publish_order.
  OrderStitcher order_stitcher_;

  // Non-owning back-pointer, set at construction and never reassigned (safe to
  // read concurrently). weak_ptr so dispatch can detect master destruction via
  // lock() instead of dangling.
  std::weak_ptr<VDA5050Master> parent_;

  // Raw observer of the same master, used ONLY by the queue-processor thread in
  // publish_*(). parent_.lock() there could make this thread the last owner and
  // trigger ~master → ~AGV self-join; the master destructor stops AGV queue
  // threads before its members destruct, so this pointer stays valid across any
  // publish().
  VDA5050Master* parent_raw_{nullptr};

  // Heartbeat listener for state timeout (guarded by heartbeat_mutex_)
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

  // Outgoing message queues (protected by queue_mutex_)
  size_t max_queue_size_;
  bool drop_oldest_;

  // Drained updates carry pre_stitched=true so publish_order skips the guard.
  struct QueuedOrder
  {
    vda5050_core::types::Order order;
    bool pre_stitched = false;
  };

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<QueuedOrder> order_queue_;
  std::queue<vda5050_core::types::InstantActions> instant_actions_queue_;

  // Mode-cancelled buffer. Protected by queue_mutex_ —
  // populated by capture_and_drain_on_leave_automatic, drained by
  // resume_mode_cancelled_queue / discard_mode_cancelled_queue.
  ModeCancelledQueue mode_cancelled_queue_;

  // Capture the live outbound queues into mode_cancelled_queue_ and drain them.
  // Called from handle_state on the AUTOMATIC→non-AUTOMATIC edge, before the
  // fleet detector fires on_mode_changed.
  void capture_and_drain_on_leave_automatic(
    vda5050_core::types::OperatingMode from,
    vda5050_core::types::OperatingMode to);

  // Queue processing thread
  std::mutex thread_mutex_;
  bool stop_processing_{false};
  bool queue_processor_running_{false};
  std::thread queue_thread_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__AGV_HPP_
