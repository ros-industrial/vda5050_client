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
#include "vda5050_core/master/assignment_result.hpp"
#include "vda5050_core/master/contexts/master_context.hpp"
#include "vda5050_core/master/master_types.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/operating_mode.hpp"

namespace vda5050_core {
namespace master {

/// \brief Abstract base for VDA5050 multi-AGV fleet control (override the on_*
///        virtuals). Must be make_shared-constructed; overrides thread-safe.
class VDA5050Master : public std::enable_shared_from_this<VDA5050Master>
{
public:
  VDA5050Master(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client);

  virtual ~VDA5050Master();

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

  /// \brief The onboarded AGV, or nullptr if not onboarded.
  std::shared_ptr<AGV> get_agv(
    const std::string& manufacturer, const std::string& serial_number) const;

  // --- Outgoing Messages ---

  /// \brief Queue an order to an AGV (lower-level; skips assign_order's
  ///        pre-flight).
  /// \return false if the AGV is not onboarded or the queue is full.
  bool publish_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order);

  /// \brief Pre-flight and queue an order. Returns an AssignmentResult: the
  ///        failed check, or ASSIGNED/STITCH_QUEUED.
  /// \param assignment_id  Correlation token; empty skips it. Read via
  ///                       get_active_assignment_id.
  AssignmentResult assign_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order,
    const std::string& assignment_id = "");

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

  // --- Assignment correlation ---
  // One active assignment per AGV; `assignments_mutex_` is never held with
  // `agv_mutex_`.

  /// Record an assignment_id for an AGV. Empty assignment_id clears.
  void record_assignment(
    const std::string& manufacturer, const std::string& serial_number,
    const std::string& assignment_id, const std::string& order_id,
    std::uint32_t order_update_id);

  std::string get_active_assignment_id(
    const std::string& manufacturer, const std::string& serial_number) const;

  void clear_assignment(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Queue instant actions to an AGV (lower-level; skips the
  ///        assign_instant_actions pre-flight).
  /// \return false if the AGV is not onboarded or the queue is full.
  bool publish_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  /// \brief Pre-flight and queue instant actions. Lighter than assign_order:
  ///        not mode/position/availability-gated, so they run when degraded.
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

  // --- User-Extension Callbacks (override in subclass) ---
  // Fire on the MQTT thread after the AGV caches; defaults empty, keep them
  // thread-safe.

  virtual void on_state(
    const std::string& agv_id, const vda5050_core::types::State& state);

  virtual void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection);

  /// \brief Feed a State into the fleet event detector (AGV-called, not an
  ///        override point).
  void ingest_state(
    const std::string& agv_id, const vda5050_core::types::State& state);

  /// \brief Feed a Connection into the fleet event detector (AGV-called).
  void ingest_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection);

  virtual void on_factsheet(
    const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet);

  virtual void on_visualization(
    const std::string& agv_id,
    const vda5050_core::types::Visualization& visualization);

  // --- Event triggers ---
  // Edge-detected hooks layered on on_state / on_connection; defaults empty.

  /// \brief Fired when the AGV reports a previously-unreached node as released.
  virtual void on_node_reached(
    const std::string& agv_id, const std::string& node_id);

  /// \brief Fired when curr.errors contains entries not present in prev.errors.
  /// \param new_errors only the newly-appeared errors (not the full curr list).
  virtual void on_errors_appeared(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& new_errors);

  /// \brief Fired when prev.errors contains entries no longer in curr.errors.
  /// \param resolved_errors only the entries that disappeared.
  virtual void on_errors_resolved(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& resolved_errors);

  /// \brief Fired when new_base_request goes false to true (rising edge).
  virtual void on_new_base_requested(const std::string& agv_id);

  /// \brief Fired on operating_mode change. Leaving master control drains the
  ///        un-sent queues to a resumable buffer; call resume_/discard.
  virtual void on_mode_changed(
    const std::string& agv_id, vda5050_core::types::OperatingMode new_mode,
    vda5050_core::types::OperatingMode prev_mode);

  /// \brief Fired when the AGV's `paused` field flips.
  virtual void on_paused(const std::string& agv_id, bool paused);

  /// \brief Fired when the AGV's `driving` field flips.
  virtual void on_driving(const std::string& agv_id, bool driving);

  /// \brief Fired when the AGV's loads vector changes.
  /// \param loads the full new vector (empty if none).
  virtual void on_loads_changed(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Load>& loads);

  // --- Connection event triggers ---
  // One named hook per connectionState transition; additive to on_connection.

  /// \brief Fired when the AGV's connection_state transitions to ONLINE.
  virtual void on_connect(const std::string& agv_id);

  /// \brief Fired when the AGV publishes OFFLINE (graceful shutdown).
  virtual void on_offline(const std::string& agv_id);

  /// \brief Last-will handler (CONNECTIONBROKEN): the AGV dropped unexpectedly;
  ///        the triggering Connection has a stale timestamp/headerId.
  virtual void on_connection_broken(const std::string& agv_id);

  // --- State-heartbeat event triggers ---
  // State-heartbeat timeout + recovery edge; additive to on_state.

  /// \brief Fired when the state heartbeat exceeds 30s; operational_state
  ///        becomes STATE_UNKNOWN (pre-send rejects); no auto-cancel.
  virtual void on_state_timeout(const std::string& agv_id);

  /// \brief Fired on the first State after a timeout, and on the AGV's
  ///        first-ever State (STATE_UNKNOWN to AVAILABLE).
  virtual void on_state_resumed(const std::string& agv_id);

  // --- Master-broker connection event triggers ---
  // Invoked on the transport thread — overrides must be thread-safe and prompt.

  /// \brief Fired when the master's broker connection drops; queued orders stay
  ///        queued until Paho auto-reconnects.
  virtual void on_broker_disconnected();

  /// \brief Fired on initial connect and every reconnect. No resubscribe; if
  ///        the broker lost its session, resubscribe off the transport thread.
  virtual void on_broker_reconnected();

private:
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

  // Run one observer hook, swallowing any exception so one bad override can't
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

  // Loaded topology map + per-AGV factsheet-alignment cache.
  // Lock order: agv_mutex_ → map_mutex_. Never the reverse.
  mutable std::mutex map_mutex_;
  vda5050_core::layout::Graph::ConstPtr active_graph_;
  std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
    alignment_cache_;

  // Broker connection state: written by the transport callback thread, read by
  // get_broker_status() from any thread.
  mutable std::mutex broker_status_mutex_;
  bool broker_connected_ = false;
  std::optional<std::chrono::system_clock::time_point>
    broker_last_disconnect_at_;
  std::uint64_t broker_reconnect_count_ = 0;

  // Async dispatch correlation, one entry per AGV.
  struct ActiveAssignment
  {
    std::string assignment_id;
    std::string order_id;
    std::uint32_t order_update_id = 0;
  };
  mutable std::mutex assignments_mutex_;
  std::unordered_map<std::string, ActiveAssignment> active_assignments_;

  // Transport connection-state handlers: update broker_* under the mutex, then
  // dispatch the on_broker_* virtuals outside the lock.
  void handle_broker_connection_lost(const std::string& cause);
  void handle_broker_connected(const std::string& cause);
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__MASTER_HPP_
