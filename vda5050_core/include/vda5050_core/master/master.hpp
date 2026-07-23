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

#ifndef VDA5050_CORE__MASTER__MASTER_HPP_
#define VDA5050_CORE__MASTER__MASTER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "vda5050_core/errors/validation_result.hpp"
#include "vda5050_core/layout/graph.hpp"
#include "vda5050_core/layout/layout_loader.hpp"
#include "vda5050_core/master/actions/instant_action_assignment_result.hpp"
#include "vda5050_core/master/agv.hpp"
#include "vda5050_core/master/contexts/master_context.hpp"
#include "vda5050_core/master/loaded_graph_holder.hpp"
#include "vda5050_core/master/master_types.hpp"
#include "vda5050_core/master/order/order_assignment_result.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/operating_mode.hpp"

namespace vda5050_core {
namespace master {

/// \brief VDA5050 multi-AGV fleet control: onboards AGVs, routes their
///        messages, queues outbound. Thread-safe.
class VDA5050Master : public std::enable_shared_from_this<VDA5050Master>
{
public:
  /// \brief Create a master. Shared ownership is required: the broker
  ///        callbacks and the owned AGVs hold weak references back to it.
  static std::shared_ptr<VDA5050Master> make(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client);

  ~VDA5050Master();

  VDA5050Master(const VDA5050Master&) = delete;
  VDA5050Master& operator=(const VDA5050Master&) = delete;
  VDA5050Master(VDA5050Master&&) = delete;
  VDA5050Master& operator=(VDA5050Master&&) = delete;

  // --- Connection Management ---

  void connect();
  void disconnect();
  bool is_connected() const;

  /// \brief Snapshot of the master's broker-connection state.
  struct BrokerStatusSnapshot
  {
    bool connected = false;
    /// When the broker last reported a disconnect; nullopt if never.
    std::optional<std::chrono::system_clock::time_point> last_disconnect_at;
    /// Times the broker connection was (re)established; initial connect = 1.
    std::uint64_t reconnect_count = 0;
  };

  BrokerStatusSnapshot get_broker_status() const;

  // --- AGV Onboarding/Offboarding ---

  /// \brief Onboard an AGV (interface "uagv") so its messages are routed.
  /// \param max_queue_size Outgoing queue cap (default 10).
  /// \param drop_oldest    Drop oldest vs reject-new when the queue is full.
  void onboard_agv(
    const std::string& manufacturer, const std::string& serial_number,
    size_t max_queue_size = 10, bool drop_oldest = true);

  /// \brief Onboard an AGV with a custom interface_name (else as above).
  void onboard_agv(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true);

  void offboard_agv(
    const std::string& manufacturer, const std::string& serial_number);

  bool is_agv_onboarded(
    const std::string& manufacturer, const std::string& serial_number) const;

  // --- AGV Access ---

  /// \brief Read-only view of the onboarded AGV, or nullptr if not onboarded.
  std::shared_ptr<const AGV> get_agv(
    const std::string& manufacturer, const std::string& serial_number) const;

  /// \brief Drop the AGV's queued outbound orders and instant actions; does
  ///        not send a cancelOrder to the AGV.
  void cancel_pending_orders(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Re-queue the buffer captured when the AGV left master control.
  /// \return {orders_resumed, actions_resumed}; {0, 0} if not onboarded.
  std::pair<std::size_t, std::size_t> resume_mode_cancelled_queue(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Drop the mode-cancelled buffer without re-queueing.
  /// \return {orders_discarded, actions_discarded}; {0, 0} if not onboarded.
  std::pair<std::size_t, std::size_t> discard_mode_cancelled_queue(
    const std::string& manufacturer, const std::string& serial_number);

  // --- Outgoing Messages ---

  /// \brief Queue an order to an AGV (lower-level; skips assign_order's
  ///        pre-flight and header fill — the caller owns the header).
  /// \return false if the AGV is not onboarded or the queue is full.
  bool publish_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order);

  /// \brief Pre-flight and queue an order. Returns an OrderAssignmentResult:
  ///        the failed check, or ASSIGNED/STITCH_QUEUED. The header's
  ///        version/manufacturer/serial are filled from the args when unset.
  OrderAssignmentResult assign_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order);

  // --- Batch onboarding — Device Manager integration ---

  /// \brief One AGV's slot in a batch onboarding request.
  struct OnboardSpec
  {
    std::string manufacturer;
    std::string serial_number;
    std::size_t max_queue_size = 10;
    bool drop_oldest = true;
  };

  /// \brief Per-entry batch outcome, split into onboarded / skipped / failed.
  struct BatchOnboardResult
  {
    std::vector<OnboardSpec> onboarded;  ///< Newly onboarded by this call.
    /// Already-onboarded keys — idempotent no-op.
    std::vector<OnboardSpec> skipped_already_onboarded;
    std::vector<OnboardSpec> failed;  ///< Failed validation (empty mfg/serial).
  };

  /// \brief Onboard a batch under one `agv_mutex_` acquisition; idempotent per
  ///        AGV, empty mfg/serial counts as failed.
  BatchOnboardResult onboard_agv_batch(const std::vector<OnboardSpec>& specs);

  /// \brief Offboard each present `{mfg, serial}` key; returns the count.
  std::size_t offboard_agv_batch(
    const std::vector<std::pair<std::string, std::string>>& keys);

  std::vector<std::pair<std::string, std::string>> get_onboarded_agvs() const;

  /// \brief Queue instant actions to an AGV (lower-level; skips the
  ///        assign_instant_actions pre-flight and header fill).
  /// \return false if the AGV is not onboarded or the queue is full.
  bool publish_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  /// \brief Pre-flight and queue instant actions. Lighter than assign_order:
  ///        not mode/position/availability-gated, so they run when degraded.
  ///        The header's version/manufacturer/serial are filled from the args
  ///        when unset.
  InstantActionAssignmentResult assign_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  // --- Topology layout ---

  /// \brief Load a LIF topology from JSON; swaps the graph, re-runs alignment.
  /// \return Load result; `lif` holds the parsed layout, else errors.
  vda5050_core::layout::LayoutLoadResult load_layout_from_config(
    const std::string& path);

  /// \brief Install an already-built graph (tests / external loaders).
  void set_graph(vda5050_core::layout::Graph::ConstPtr graph);

  /// \brief The currently-loaded graph, or nullptr; safe to hold across swaps.
  vda5050_core::layout::Graph::ConstPtr get_loaded_graph() const;

  /// \brief Snapshot of the entire alignment cache, keyed by agv_id.
  std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
  get_alignment_cache_snapshot() const;

  /// \brief Refresh one AGV's alignment against the loaded graph after its
  ///        factsheet arrives; prefer overriding on_factsheet over this.
  void refresh_alignment_for_agv(
    const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet);

  // --- Reaction callbacks (register handlers to react to AGV messages) ---
  // Registered handlers fire on the MQTT thread after the AGV caches. Register
  // before connect(); keep them prompt and thread-safe.

  /// \brief Register the handler invoked on every State message.
  void on_state(
    std::function<
      void(const std::string& agv_id, const vda5050_core::types::State& state)>
      callback);

  /// \brief Register the handler invoked on every Connection message.
  void on_connection(std::function<void(
                       const std::string& agv_id,
                       const vda5050_core::types::Connection& connection)>
                       callback);

  /// \brief Register the handler invoked on every Factsheet message.
  void on_factsheet(std::function<void(
                      const std::string& agv_id,
                      const vda5050_core::types::Factsheet& factsheet)>
                      callback);

  /// \brief Register the handler invoked on every Visualization message.
  void on_visualization(
    std::function<void(
      const std::string& agv_id,
      const vda5050_core::types::Visualization& visualization)>
      callback);

  // --- Event triggers ---
  // Edge-detected callbacks layered on on_state / on_connection.

  /// \brief Register the handler invoked when the AGV reports a
  ///        previously-unreached node as released.
  void on_node_reached(
    std::function<void(const std::string& agv_id, const std::string& node_id)>
      callback);

  /// \brief Register the handler invoked when the AGV completes its active
  ///        order (parked at the last released node, all actions terminal).
  void on_order_complete(
    std::function<void(const std::string& agv_id, const std::string& order_id)>
      callback);

  /// \brief Register the handler invoked when a queued order is discarded
  ///        before publish, after assign_order already returned ASSIGNED.
  ///
  /// \param callback receives the errors that caused the rejection. Pairs with
  ///        on_order_complete: every accepted order ends in one or the other.
  void on_order_rejected(
    std::function<void(
      const std::string& agv_id, const std::string& order_id,
      const std::vector<vda5050_core::types::Error>& errors)>
      callback);

  /// \brief Register the handler invoked when the error list gains entries.
  /// \param callback receives only the newly-appeared errors.
  void on_errors_appeared(
    std::function<void(
      const std::string& agv_id,
      const std::vector<vda5050_core::types::Error>& new_errors)>
      callback);

  /// \brief Register the handler invoked when the error list loses entries.
  /// \param callback receives only the entries that disappeared.
  void on_errors_resolved(
    std::function<void(
      const std::string& agv_id,
      const std::vector<vda5050_core::types::Error>& resolved_errors)>
      callback);

  /// \brief Register the handler invoked when new_base_request rises
  ///        false→true.
  void on_new_base_requested(
    std::function<void(const std::string& agv_id)> callback);

  /// \brief Register the handler invoked on operating_mode change. Leaving
  ///        master control drains un-sent queues to a resumable buffer.
  void on_mode_changed(
    std::function<void(
      const std::string& agv_id, vda5050_core::types::OperatingMode new_mode,
      vda5050_core::types::OperatingMode prev_mode)>
      callback);

  /// \brief Register the handler invoked when the AGV's `paused` field flips.
  void on_paused(
    std::function<void(const std::string& agv_id, bool paused)> callback);

  /// \brief Register the handler invoked when the AGV's `driving` field flips.
  void on_driving(
    std::function<void(const std::string& agv_id, bool driving)> callback);

  /// \brief Register the handler invoked when the AGV's loads vector changes.
  /// \param callback receives the full new vector (empty if none).
  void on_loads_changed(std::function<void(
                          const std::string& agv_id,
                          const std::vector<vda5050_core::types::Load>& loads)>
                          callback);

  // --- Connection event triggers ---
  // One named callback per connectionState transition; adds to on_connection.

  /// \brief Register the handler invoked when connection_state becomes ONLINE.
  void on_connect(std::function<void(const std::string& agv_id)> callback);

  /// \brief Register the handler invoked when the AGV publishes OFFLINE.
  void on_offline(std::function<void(const std::string& agv_id)> callback);

  /// \brief Register the last-will (CONNECTIONBROKEN) handler: the AGV dropped
  ///        unexpectedly; the triggering Connection has a stale timestamp.
  void on_connection_broken(
    std::function<void(const std::string& agv_id)> callback);

  // --- State-heartbeat event triggers ---
  // State-heartbeat timeout + recovery edge; additive to on_state.

  /// \brief Register the handler invoked when the state heartbeat exceeds 30s;
  ///        operational_state becomes STATE_UNKNOWN (pre-send rejects).
  void on_state_timeout(
    std::function<void(const std::string& agv_id)> callback);

  /// \brief Register the handler invoked on the first State after a timeout,
  ///        and on the AGV's first-ever State (STATE_UNKNOWN → AVAILABLE).
  void on_state_resumed(
    std::function<void(const std::string& agv_id)> callback);

  // --- Master-broker connection event triggers ---
  // Invoked on the transport thread — handlers must be thread-safe and prompt.

  /// \brief Register the handler invoked when the master's broker connection
  ///        drops; queued orders stay queued until Paho auto-reconnects.
  void on_broker_disconnected(std::function<void()> callback);

  /// \brief Register the handler invoked on initial connect and each reconnect.
  void on_broker_reconnected(std::function<void()> callback);

private:
  VDA5050Master(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client);

  // The owned AGV calls these on the MQTT thread to feed the event detector
  // and run the registered raw-message handlers; not user-facing.
  friend class AGV;
  void ingest_state(
    const std::string& agv_id, const vda5050_core::types::State& state);
  void ingest_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection);
  void dispatch_state(
    const std::string& agv_id, const vda5050_core::types::State& state);
  void dispatch_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection);
  void dispatch_factsheet(
    const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet);
  void dispatch_visualization(
    const std::string& agv_id,
    const vda5050_core::types::Visualization& visualization);
  void dispatch_state_timeout(const std::string& agv_id);
  void dispatch_state_resumed(const std::string& agv_id);
  void dispatch_order_complete(
    const std::string& agv_id, const std::string& order_id);
  void dispatch_order_rejected(
    const std::string& agv_id, const std::string& order_id,
    const std::vector<vda5050_core::types::Error>& errors);

  // --- Internal AGV lookup ---

  std::shared_ptr<AGV> get_agv_by_id(const std::string& agv_id) const;

  // First action_id that is empty, duplicated, or collides with an in-flight,
  // active-order, or queued id (else nullopt).
  std::optional<std::string> first_instant_action_id_conflict(
    const std::shared_ptr<AGV>& agv,
    const std::optional<vda5050_core::types::State>& last_state,
    const vda5050_core::types::InstantActions& actions) const;

  // Wire the fleet fan-out to master_context_'s Provider once, in the ctor:
  // each agv_id-tagged update routes to its observer hook via fire_hook.
  void register_event_dispatch();

  // Run one observer hook, swallowing any exception so one bad callback can't
  // stall the shared inbound thread.
  void fire_hook(
    const std::string& agv_id, const char* hook_name,
    const std::function<void()>& fn);

  // Build an AGV. Caller holds `agv_mutex_`; call setup_subscriptions() only
  // AFTER releasing it (subscribing under the lock can deadlock).
  std::shared_ptr<AGV> create_agv_locked(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, std::size_t max_queue_size,
    bool drop_oldest);

  // --- Member Variables ---

  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client_;

  // Fleet-wide event detector (AGVs feed it via ingest_*). Declared before
  // agvs_ so it outlives them.
  MasterContext master_context_;

  // Onboarded AGVs, keyed by agv_id.
  mutable std::mutex agv_mutex_;
  std::unordered_map<std::string, std::shared_ptr<AGV>> agvs_;

  // Loaded graph, shared with each AGV's queue thread (self-guarded holder,
  // so the queue thread reads it without referencing the master).
  std::shared_ptr<LoadedGraphHolder> graph_holder_ =
    std::make_shared<LoadedGraphHolder>();

  // Per-AGV factsheet-alignment cache.
  // Lock order: agv_mutex_ → map_mutex_. Never the reverse.
  mutable std::mutex map_mutex_;
  std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
    alignment_cache_;

  // Broker connection state: written by the transport callback thread, read by
  // get_broker_status() from any thread.
  mutable std::mutex broker_status_mutex_;
  bool broker_connected_ = false;
  std::optional<std::chrono::system_clock::time_point>
    broker_last_disconnect_at_;
  std::uint64_t broker_reconnect_count_ = 0;

  // Registered reaction callbacks. Set before connect() (the single inbound
  // thread reads them), so no mutex; an unset slot is a no-op.
  std::function<void(const std::string&, const vda5050_core::types::State&)>
    on_state_cb_;
  std::function<void(
    const std::string&, const vda5050_core::types::Connection&)>
    on_connection_cb_;
  std::function<void(const std::string&, const vda5050_core::types::Factsheet&)>
    on_factsheet_cb_;
  std::function<void(
    const std::string&, const vda5050_core::types::Visualization&)>
    on_visualization_cb_;
  std::function<void(const std::string&, const std::string&)>
    on_node_reached_cb_;
  std::function<void(const std::string&, const std::string&)>
    on_order_complete_cb_;
  std::function<void(
    const std::string&, const std::string&,
    const std::vector<vda5050_core::types::Error>&)>
    on_order_rejected_cb_;
  std::function<void(
    const std::string&, const std::vector<vda5050_core::types::Error>&)>
    on_errors_appeared_cb_;
  std::function<void(
    const std::string&, const std::vector<vda5050_core::types::Error>&)>
    on_errors_resolved_cb_;
  std::function<void(const std::string&)> on_new_base_requested_cb_;
  std::function<void(
    const std::string&, vda5050_core::types::OperatingMode,
    vda5050_core::types::OperatingMode)>
    on_mode_changed_cb_;
  std::function<void(const std::string&, bool)> on_paused_cb_;
  std::function<void(const std::string&, bool)> on_driving_cb_;
  std::function<void(
    const std::string&, const std::vector<vda5050_core::types::Load>&)>
    on_loads_changed_cb_;
  std::function<void(const std::string&)> on_connect_cb_;
  std::function<void(const std::string&)> on_offline_cb_;
  std::function<void(const std::string&)> on_connection_broken_cb_;
  std::function<void(const std::string&)> on_state_timeout_cb_;
  std::function<void(const std::string&)> on_state_resumed_cb_;
  std::function<void()> on_broker_disconnected_cb_;
  std::function<void()> on_broker_reconnected_cb_;

  // Transport connection-state handlers: update broker_* under the mutex, then
  // invoke the on_broker_* callbacks outside the lock.
  void handle_broker_connection_lost(const std::string& cause);
  void handle_broker_connected(const std::string& cause);
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__MASTER_HPP_
