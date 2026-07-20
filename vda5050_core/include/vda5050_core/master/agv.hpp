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
#include "vda5050_core/master/loaded_graph_holder.hpp"
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

/// \brief An AGV managed by VDA5050Master: caches messages, tracks
///        connection/operational state, queues outbound. Thread-safe.
class AGV : public std::enable_shared_from_this<AGV>
{
public:
  using Clock = std::chrono::system_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  /// \brief Construct an AGV. Call setup_subscriptions() after make_shared
  ///        (weak_from_this needs it).
  AGV(
    std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_adapter,
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true,
    int state_heartbeat_interval = StateHeartbeatInterval,
    std::weak_ptr<VDA5050Master> parent = {},
    std::shared_ptr<LoadedGraphHolder> graph_holder = {});

  /// \brief Stop the queue processor and heartbeat; join the worker thread.
  ~AGV();

  AGV(const AGV&) = delete;
  AGV& operator=(const AGV&) = delete;
  AGV(AGV&&) = delete;
  AGV& operator=(AGV&&) = delete;

  // --- Identity ---

  const std::string& get_interface_name() const
  {
    return interface_name_;
  }
  const std::string& get_manufacturer() const
  {
    return manufacturer_;
  }

  const std::string& get_serial_number() const
  {
    return serial_number_;
  }

  const std::string& get_agv_id() const
  {
    return agv_id_;
  }

  // --- Connection and Operational State ---

  bool is_connected() const;

  vda5050_core::types::ConnectionState get_connection_status() const;

  AGVState get_operational_state() const;

  /// \brief Stop the processor + heartbeat and clear queues; cached kept.
  void stop();

  /// \brief stop() plus clear cached messages; re-arms on the next ONLINE.
  void restart();

  /// \brief Suspend processor + heartbeat (OFFLINE/UNAVAILABLE); queues+cache
  ///        kept.
  void pause();

  /// \brief Resume the processor + heartbeat after pause().
  void resume();

  // --- Cached Messages (read-only access) ---

  std::optional<vda5050_core::types::Connection> get_last_connection() const;

  std::optional<vda5050_core::types::State> get_last_state() const;

  std::optional<vda5050_core::types::Factsheet> get_last_factsheet() const;

  std::optional<vda5050_core::types::Visualization> get_last_visualization()
    const;

  /// \brief Coherent snapshot (one data_mutex_ acquisition) of cached
  ///        State/Connection/Factsheet + receive times.
  struct StatusSnapshot
  {
    std::optional<vda5050_core::types::State> state;
    std::optional<vda5050_core::types::Connection> connection;
    std::optional<vda5050_core::types::Factsheet> factsheet;
    std::optional<TimePoint> state_received_at;
    std::optional<TimePoint> connection_received_at;
    std::optional<TimePoint> factsheet_received_at;
  };

  StatusSnapshot get_status_snapshot() const;

  /// \brief Coherent bundle of cached State + order-lifecycle view (data_mutex_
  ///        then OrderLifecycleManager).
  struct OrderStatusBundle
  {
    std::optional<vda5050_core::types::State> state;
    std::optional<TimePoint> state_received_at;
    ActiveOrderSnapshot active_order_snapshot;
    std::size_t pending_stitch_count;
  };

  OrderStatusBundle get_order_status_bundle() const;

  /// \brief Fused pose snapshot from the freshest of cached State /
  ///        Visualization carrying an initialized position, under data_mutex_.
  /// \return PoseView; source == None when no initialized position is cached.
  PoseView get_pose_view() const;

  // --- Order Lifecycle (forwarders to OrderLifecycleManager) ---

  /// \brief Whether the master tracks an active order for this AGV (set on a
  ///        recorded publish; cleared by recovery/new-order/explicit clear).
  bool has_active_order() const;

  std::optional<std::string> active_order_id() const;

  std::optional<uint32_t> active_order_update_id() const;

  /// \brief True once the AGV's reported last_node matches the active
  ///        order's final node. Sticky until a new order is recorded.
  bool is_order_complete() const;

  /// \brief True when the AGV requested newBaseRequest and no higher
  ///        order_update_id has been recorded yet.
  bool active_order_needs_more_base() const;

  size_t pending_update_count() const;

  /// \brief Active-order tracking snapshot; safe to read concurrently.
  ActiveOrderSnapshot active_order_snapshot() const;

  // --- Timestamps ---

  TimePoint get_created_time() const
  {
    return created_time_;
  }

  std::optional<TimePoint> get_last_connection_time() const;

  std::optional<TimePoint> get_last_state_time() const;

  std::optional<TimePoint> get_last_factsheet_time() const;

  std::optional<TimePoint> get_last_visualization_time() const;

  // --- Outgoing Messages ---

  /// \brief Queue an order to this AGV.
  /// \return true if queued, false if the queue is full (drop_oldest=false).
  bool send_order(const vda5050_core::types::Order& order);

  /// \brief Queue instant actions to this AGV.
  /// \return true if queued, false if the queue is full (drop_oldest=false).
  bool send_instant_actions(const vda5050_core::types::InstantActions& actions);

  size_t get_pending_order_count() const;

  size_t get_pending_instant_actions_count() const;

  /// \brief action_ids of instant actions queued but not yet published.
  ///        Used to keep action_id uniqueness checks queue-aware.
  std::vector<std::string> get_queued_instant_action_ids() const;

  /// \brief Drop all queued outbound Orders and InstantActions (master-side
  ///        only; does not send a cancelOrder to the AGV). Thread-safe.
  void cancel_pending_orders();

  // --- Mode-cancelled queue (capture-and-resume on mode change) ---
  // Leaving master control drains the outbound queues here before
  // on_mode_changed; the override calls resume/discard.

  /// Queue items captured at the most recent transition out of master control.
  struct ModeCancelledQueue
  {
    std::vector<vda5050_core::types::Order> orders;
    std::vector<vda5050_core::types::InstantActions> instant_actions;
    std::optional<TimePoint> cancelled_at;
    std::optional<vda5050_core::types::OperatingMode> from_mode;
    std::optional<vda5050_core::types::OperatingMode> to_mode;
  };

  /// \brief Snapshot the mode-cancelled buffer (empty if the AGV hasn't left
  ///        master control since onboard/resume/discard). Thread-safe.
  ModeCancelledQueue get_mode_cancelled_queue() const;

  /// \brief Prepend the captured buffer to the live queue (submission order)
  ///        and clear it; re-enqueued items re-run the validator chain.
  /// \return {orders_resumed, actions_resumed}.
  std::pair<std::size_t, std::size_t> resume_mode_cancelled_queue();

  /// \brief Drop the mode-cancelled buffer without re-enqueue.
  /// \return {orders_discarded, actions_discarded}.
  std::pair<std::size_t, std::size_t> discard_mode_cancelled_queue();

  // --- Message Handlers (VDA5050Master routes incoming messages here) ---

  void handle_connection(const vda5050_core::types::Connection& msg);
  void handle_state(const vda5050_core::types::State& msg);
  void handle_factsheet(const vda5050_core::types::Factsheet& msg);
  void handle_visualization(const vda5050_core::types::Visualization& msg);

  /// \brief Wire per-topic subscriptions. Call after make_shared (the wrappers
  ///        capture weak_from_this()).
  void setup_subscriptions();

private:
  // Subscription wrapper: lock weak_from_this() first (keeps the AGV alive for
  // the dispatch, no-op if gone). Parse/handler errors log, never rethrow.
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

  // --- Internal State Management ---

  void set_connection_status(vda5050_core::types::ConnectionState status);
  // Returns the effective state after precedence (STATE_UNKNOWN is ignored
  // under UNAVAILABLE/ERROR), so callers see if the transition took effect.
  AGVState set_operational_state(AGVState state);
  void on_state_heartbeat_timeout();

  void setup_heartbeat();
  void cleanup_heartbeat();

  // --- Queue Processing ---

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

  // --- Member Variables ---

  std::string interface_name_;
  std::string manufacturer_;
  std::string serial_number_;
  std::string agv_id_;

  std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_adapter_;

  // Stateless publishers that run the outgoing validator chain.
  OrderPublisher order_publisher_;
  InstantActionsPublisher instant_actions_publisher_;

  // Per-AGV order lifecycle; owns its mutex, updated from handle_state and
  // publish_order.
  OrderLifecycleManager order_lifecycle_;

  // Stateless stitch guard at the front of publish_order.
  OrderStitcher order_stitcher_;

  // Set once at construction, never reassigned (safe to read concurrently);
  // weak_ptr so dispatch detects master destruction via lock().
  std::weak_ptr<VDA5050Master> parent_;

  // Loaded graph shared with the master; read by the queue thread without
  // referencing the master. Set once at construction, never reassigned.
  std::shared_ptr<LoadedGraphHolder> graph_holder_;

  // Heartbeat listener for state timeout (guarded by heartbeat_mutex_)
  mutable std::mutex heartbeat_mutex_;
  std::unique_ptr<HeartbeatListener> state_heartbeat_;
  int state_heartbeat_interval_;

  // AGV states (protected by state_mutex_)
  mutable std::mutex state_mutex_;
  vda5050_core::types::ConnectionState connection_status_{
    vda5050_core::types::ConnectionState::OFFLINE};
  AGVState operational_state_{AGVState::STATE_UNKNOWN};

  TimePoint created_time_;

  // Cached messages and timestamps (protected by data_mutex_)
  mutable std::mutex data_mutex_;

  std::optional<vda5050_core::types::Connection> last_connection_;
  std::optional<TimePoint> last_connection_time_;

  std::optional<vda5050_core::types::State> last_state_;
  std::optional<TimePoint> last_state_time_;
  // Monotonic receive stamp for data_age (wall-clock steps must not skew it).
  std::optional<std::chrono::steady_clock::time_point> last_state_steady_;
  // Stale-State gate (QoS 0): drop a State not strictly newer than the last
  // cached header_id; reset on reconnect so a restarted AGV isn't locked out.
  uint32_t last_state_header_id_ = 0;
  bool have_state_baseline_ = false;

  std::optional<vda5050_core::types::Factsheet> last_factsheet_;
  std::optional<TimePoint> last_factsheet_time_;

  std::optional<vda5050_core::types::Visualization> last_visualization_;
  std::optional<TimePoint> last_visualization_time_;
  // Monotonic receive stamp for data_age (see last_state_steady_).
  std::optional<std::chrono::steady_clock::time_point>
    last_visualization_steady_;
  // Stale-Visualization gate (QoS 0), same policy as State.
  uint32_t last_visualization_header_id_ = 0;
  bool have_visualization_baseline_ = false;

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

  // Mode-cancelled buffer; protected by queue_mutex_.
  ModeCancelledQueue mode_cancelled_queue_;

  // Capture + drain the outbound queues; runs before on_mode_changed fires.
  void capture_and_drain_on_leave_master_control(
    vda5050_core::types::OperatingMode from,
    vda5050_core::types::OperatingMode to);

  std::mutex thread_mutex_;
  bool stop_processing_{false};
  bool queue_processor_running_{false};
  std::thread queue_thread_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__AGV_HPP_
