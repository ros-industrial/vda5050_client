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

/// \brief Abstract base for VDA5050 multi-AGV fleet control over one shared
///        MQTT client; subclass and override the on_* virtuals.
///
/// Must be make_shared-constructed: each AGV holds a weak_ptr back to it (via
/// weak_from_this) to dispatch callbacks on the MQTT thread — a stack or
/// unique_ptr master silently no-ops them. Overrides must be thread-safe.
class VDA5050Master : public std::enable_shared_from_this<VDA5050Master>
{
public:
  /// \brief Construct with a shared MQTT client (creates per-AGV adapters).
  ///        Must be make_shared-constructed — see the class doc.
  VDA5050Master(
    std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client);

  /// \brief Virtual destructor - disconnects MQTT client
  virtual ~VDA5050Master();

  // Non-copyable, non-movable
  VDA5050Master(const VDA5050Master&) = delete;
  VDA5050Master& operator=(const VDA5050Master&) = delete;
  VDA5050Master(VDA5050Master&&) = delete;
  VDA5050Master& operator=(VDA5050Master&&) = delete;

  // ===========================================================================
  // Connection Management
  // ===========================================================================

  /// \brief Connect the MQTT client
  void connect();

  /// \brief Disconnect the MQTT client
  void disconnect();

  /// \brief Check if MQTT client is connected
  bool is_connected() const;

  // Master's own broker-connection state (distinct from per-AGV connection):
  // exposed via the on_broker_* virtuals and get_broker_status().

  /// \brief Snapshot of the master's broker-connection state.
  struct BrokerStatusSnapshot
  {
    /// True iff the master is currently connected to its broker.
    bool connected = false;
    /// Time at which the broker last reported a disconnect. nullopt if
    /// no disconnect has occurred since process start.
    std::optional<std::chrono::system_clock::time_point> last_disconnect_at;
    /// Number of times the master's broker connection has been
    /// (re)established. Initial successful connect counts as 1; every
    /// Paho-driven auto-reconnect increments this.
    std::uint64_t reconnect_count = 0;
  };

  /// Snapshot of the master's broker-connection state.
  BrokerStatusSnapshot get_broker_status() const;

  // ===========================================================================
  // AGV Onboarding/Offboarding
  // ===========================================================================

  /// \brief Onboard an AGV (interface "uagv") so its messages are routed.
  /// \param manufacturer   AGV manufacturer.
  /// \param serial_number  AGV serial number.
  /// \param max_queue_size Outgoing queue cap (default 10).
  /// \param drop_oldest Drop oldest vs reject-new when the queue is full.
  void onboard_agv(
    const std::string& manufacturer, const std::string& serial_number,
    size_t max_queue_size = 10, bool drop_oldest = true);

  /// \brief Onboard an AGV with a custom interface_name so its messages route.
  /// \param interface_name Interface name.
  /// \param manufacturer   AGV manufacturer.
  /// \param serial_number  AGV serial number.
  /// \param max_queue_size Outgoing queue cap (default 10).
  /// \param drop_oldest Drop oldest vs reject-new when the queue is full.
  void onboard_agv(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true);

  /// \brief Offboard an AGV; further messages from it are ignored.
  /// \param manufacturer   AGV manufacturer.
  /// \param serial_number  AGV serial number.
  void offboard_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief True if the AGV is onboarded.
  /// \param manufacturer   AGV manufacturer.
  /// \param serial_number  AGV serial number.
  bool is_agv_onboarded(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ===========================================================================
  // AGV Access
  // ===========================================================================

  /// \brief The onboarded AGV, or nullptr if not onboarded.
  /// \param manufacturer   AGV manufacturer.
  /// \param serial_number  AGV serial number.
  std::shared_ptr<AGV> get_agv(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ===========================================================================
  // Outgoing Messages
  // ===========================================================================

  /// \brief Queue an order to an AGV (lower-level; skips assign_order's
  ///        pre-flight).
  /// \param manufacturer   AGV manufacturer.
  /// \param serial_number  AGV serial number.
  /// \param order          The order to queue.
  /// \return false if the AGV is not onboarded or the queue is full.
  bool publish_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order);

  /// \brief Pre-flight an order (onboarded, ONLINE, AVAILABLE, AUTOMATIC,
  ///        position-initialized, stitch-acceptable) then queue it.
  ///
  /// The recommended FMS entry point. Returns an AssignmentResult naming the
  /// failed check with diagnostics (nothing queued) or ASSIGNED/STITCH_QUEUED;
  /// the async validator chain re-checks on the queue thread as defense.
  /// \param assignment_id  Correlation token recorded on success; empty skips
  ///                       it. Read back via get_active_assignment_id.
  AssignmentResult assign_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order,
    const std::string& assignment_id = "");

  // ===========================================================================
  // Batch onboarding — Device Manager integration
  // ===========================================================================

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
    /// Entries newly onboarded by this call.
    std::vector<OnboardSpec> onboarded;
    /// Entries whose {mfg, serial} key was already in the master's
    /// onboarded set. Idempotent no-op.
    std::vector<OnboardSpec> skipped_already_onboarded;
    /// Entries that failed validation (empty mfg or serial today).
    std::vector<OnboardSpec> failed;
  };

  /// Onboard a batch of AGVs under a single `agv_mutex_` acquisition.
  /// Idempotent per AGV (already-onboarded entries are skipped);
  /// empty mfg or serial counted as failed.
  BatchOnboardResult onboard_agv_batch(const std::vector<OnboardSpec>& specs);

  /// Offboard each `{mfg, serial}` key if present. Missing keys are
  /// silently ignored. Returns the number actually offboarded.
  std::size_t offboard_agv_batch(
    const std::vector<std::pair<std::string, std::string>>& keys);

  /// Snapshot of currently-onboarded AGVs as `{mfg, serial}` pairs.
  std::vector<std::pair<std::string, std::string>> get_onboarded_agvs() const;

  // ===========================================================================
  // Assignment correlation — caller assignment_id per AGV for async dispatch
  // ===========================================================================
  // One active assignment per AGV (overwrites). `assignments_mutex_`, never
  // held with `agv_mutex_`.

  /// Record an assignment_id for an AGV. Empty assignment_id clears.
  void record_assignment(
    const std::string& manufacturer, const std::string& serial_number,
    const std::string& assignment_id, const std::string& order_id,
    std::uint32_t order_update_id);

  /// Return the active assignment_id for an AGV, or empty if none.
  std::string get_active_assignment_id(
    const std::string& manufacturer, const std::string& serial_number) const;

  /// Drop the active assignment_id mapping for an AGV.
  void clear_assignment(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Queue instant actions to an AGV (lower-level; skips the
  ///        assign_instant_actions pre-flight).
  /// \return false if the AGV is not onboarded or the queue is full.
  bool publish_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  /// \brief Pre-flight instant actions (onboarded, ONLINE, action_id unique)
  ///        then queue them.
  ///
  /// Lighter than assign_order on purpose — instant actions must work in
  /// degraded states (cancelOrder in ERROR, initPosition before localization),
  /// so mode/position/availability are not gated. Returns an
  /// InstantActionAssignmentResult naming any failed check; nothing queued on
  /// rejection. The async validator chain re-checks on the queue thread.
  InstantActionAssignmentResult assign_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  // ==========================================================================
  // Topology layout
  // ==========================================================================
  //
  // The master loads a LIF topology from JSON at startup and cross-checks it
  // against onboarded factsheets; the traversability validator consults it.
  // `active_graph_`/`alignment_cache_` are guarded by `map_mutex_` (lock order
  // agv_mutex_ then map_mutex_); readers copy the immutable snapshot under the
  // lock, then work lock-free.

  /// \brief Load a LIF topology from a JSON config; on success swaps the active
  ///        graph and re-runs alignment (the prior graph is kept on failure).
  /// \param path  Path to the LIF JSON config.
  /// \return Load result; `lif` holds the parsed layout on success.
  vda5050_core::layout::LayoutLoadResult load_layout_from_config(
    const std::string& path);

  /// \brief Install an already-built graph (tests / external loaders); runs the
  ///        same alignment refresh as load_layout_from_config.
  void set_graph(vda5050_core::layout::Graph::ConstPtr graph);

  /// \brief Snapshot of the currently-loaded graph, or nullptr if none is
  /// loaded. Safe to hold across graph swaps.
  vda5050_core::layout::Graph::ConstPtr get_loaded_graph() const;

  /// \brief Snapshot of the entire alignment cache, keyed by agv_id.
  std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
  get_alignment_cache_snapshot() const;

  /// \brief Refresh the alignment cache for one AGV against the loaded
  /// graph after its factsheet arrives. Pairs with load/set to keep
  /// alignment fresh whichever of {layout-load, factsheet-arrival} happens
  /// last. Override `on_factsheet` rather than calling this directly.
  void refresh_alignment_for_agv(
    const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet);

  // ===========================================================================
  // User-Extension Callbacks (override in subclass)
  // ===========================================================================
  //
  // These virtuals fire on the Paho MQTT callback thread after the AGV caches
  // the deserialized message, dispatched via the AGV's back-pointer to its
  // master. Defaults are empty — override for fleet-level reactions, and keep
  // overrides thread-safe for any state they touch.

  /// \brief Called after a State message arrives and is cached on the AGV.
  /// \param agv_id  manufacturer/serial composite ID
  /// \param state   the parsed State message
  virtual void on_state(
    const std::string& agv_id, const vda5050_core::types::State& state);

  /// \brief Called after a Connection message arrives and is cached.
  virtual void on_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection);

  /// \brief Feed a State into the fleet event detector. Called by the AGV after
  ///        on_state; drives the named edge-detected hooks below. Not an
  ///        override point.
  void ingest_state(
    const std::string& agv_id, const vda5050_core::types::State& state);

  /// \brief Feed a Connection into the fleet event detector. Called by the AGV
  ///        after on_connection; drives the connection event hooks.
  void ingest_connection(
    const std::string& agv_id,
    const vda5050_core::types::Connection& connection);

  /// \brief Called after a Factsheet message arrives and is cached.
  virtual void on_factsheet(
    const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet);

  /// \brief Called after a Visualization message arrives and is cached.
  virtual void on_visualization(
    const std::string& agv_id,
    const vda5050_core::types::Visualization& visualization);

  // ===========================================================================
  // Event triggers
  // ===========================================================================
  //
  // Named edge-detected hooks layered on top of on_state / on_connection (the
  // raw virtuals still fire). Defaults empty — override the ones you need.

  /// \brief Fired when the AGV reports a previously-unreached node as released.
  virtual void on_node_reached(
    const std::string& agv_id, const std::string& node_id);

  /// \brief Fired when curr.errors contains entries not present in prev.errors.
  /// \param new_errors only the newly-appeared errors (not the full curr list).
  virtual void on_errors_appeared(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& new_errors);

  /// \brief Fired when prev.errors contains entries no longer in curr.errors.
  ///        Captures the spec's "self-resolving WARNING" recovery.
  /// \param resolved_errors only the entries that disappeared.
  virtual void on_errors_resolved(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Error>& resolved_errors);

  /// \brief Fired when new_base_request goes false to true (rising edge).
  virtual void on_new_base_requested(const std::string& agv_id);

  /// \brief Fired when operating_mode changes.
  /// \param new_mode  the mode now in effect.
  /// \param prev_mode the mode before the change.
  ///
  /// When the AGV leaves master control the outbound queues are already
  /// captured into a resumable buffer (see agv->get_mode_cancelled_queue())
  /// and drained. On the return edge call resume_mode_cancelled_queue() or
  /// discard_mode_cancelled_queue(), else the buffer is overwritten on the
  /// next such transition.
  ///
  /// The buffer holds only un-sent orders; an already-dispatched active order
  /// is not captured. In MANUAL the AGV clears its own orders, so reconciling
  /// or re-issuing the previously-active order on return is the FMS's
  /// responsibility.
  virtual void on_mode_changed(
    const std::string& agv_id, vda5050_core::types::OperatingMode new_mode,
    vda5050_core::types::OperatingMode prev_mode);

  /// \brief Fired when the AGV's `paused` field flips (either direction).
  /// \param paused the new value (true = now paused, false = now unpaused).
  virtual void on_paused(const std::string& agv_id, bool paused);

  /// \brief Fired when the AGV's `driving` field flips.
  /// \param driving the new value (true = started driving, false = stopped).
  virtual void on_driving(const std::string& agv_id, bool driving);

  /// \brief Fired when the AGV's loads vector changes (count, contents,
  ///        or both).
  /// \param loads the new full loads vector (empty vector if AGV reports
  ///        no loads).
  virtual void on_loads_changed(
    const std::string& agv_id,
    const std::vector<vda5050_core::types::Load>& loads);

  // ===========================================================================
  // Connection event triggers
  // ===========================================================================
  //
  // Fired after connection_update_detector classifies the transition, one named
  // hook per connectionState value. The raw on_connection still fires for every
  // Connection message; these are additive.

  /// \brief Fired when AGV's connection_state transitions to ONLINE.
  ///        Initial connect or reconnect after recovery.
  virtual void on_connect(const std::string& agv_id);

  /// \brief Fired when AGV's connection_state transitions to OFFLINE
  ///        (graceful shutdown — AGV explicitly published OFFLINE
  ///        before disconnecting).
  virtual void on_offline(const std::string& agv_id);

  /// \brief Last-will handler (CONNECTIONBROKEN): the AGV's TCP connection
  ///        dropped unexpectedly. The triggering Connection has stale
  ///        timestamp/headerId. React to AGV death here (cancel pending).
  virtual void on_connection_broken(const std::string& agv_id);

  // ===========================================================================
  // State-heartbeat event triggers
  // ===========================================================================
  //
  // Fired by the AGV's state-topic HeartbeatListener when the spec's 30s window
  // is violated, and again on the recovery edge. Additive to on_state, which
  // still fires for every State message.

  /// \brief Fired when the state-topic heartbeat exceeds the spec's 30s window.
  ///
  /// operational_state is already STATE_UNKNOWN (pre-send now rejects this
  /// AGV's orders). Pending orders are NOT auto-cancelled (silence may be
  /// transient); override to call AGV::cancel_pending_orders() if desired. Runs
  /// on the HeartbeatListener monitor thread.
  virtual void on_state_timeout(const std::string& agv_id);

  /// \brief Fired on the first State after an on_state_timeout, and once on the
  ///        AGV's first-ever State (STATE_UNKNOWN to AVAILABLE). Runs on the
  ///        state-subscriber thread, before on_state.
  virtual void on_state_resumed(const std::string& agv_id);

  // ===========================================================================
  // Master-broker connection event triggers
  // ===========================================================================
  //
  // Fired when the master's OWN broker connection drops or (re)establishes —
  // distinct from the per-AGV on_connect/on_offline/on_connection_broken hooks.
  // Invoked on the MQTT transport I/O thread: overrides must be thread-safe and
  // return promptly so they don't stall Paho's reconnect loop.

  /// \brief Fired when the master's broker connection drops
  ///        (get_broker_status() already reflects it). Orders can't flow to
  ///        AGVs until Paho auto-reconnects; queued orders stay queued.
  virtual void on_broker_disconnected();

  /// \brief Fired on initial connect and on every Paho auto-reconnect
  ///        (get_broker_status().reconnect_count distinguishes them).
  ///
  /// The master does NOT re-issue SUBSCRIBEs on reconnect: with a persistent
  /// session (clean_session=false) the broker resumes routing to the existing
  /// subscriptions. If the broker instead lost its session (a restart without
  /// persistence), no messages will arrive though the connection reports up;
  /// an override that must survive that case should re-subscribe here (off the
  /// transport thread — a synchronous subscribe on this thread deadlocks).
  virtual void on_broker_reconnected();

private:
  // ===========================================================================
  // Internal AGV lookup
  // ===========================================================================

  std::shared_ptr<AGV> get_agv_by_id(const std::string& agv_id) const;

  // First action_id in `actions` that is empty, duplicated in the batch, or
  // collides with an in-flight, active-order, or already-queued id (else
  // nullopt). `last_state` is passed in to reuse one AGV State snapshot.
  std::optional<std::string> first_instant_action_id_conflict(
    const std::shared_ptr<AGV>& agv,
    const std::optional<vda5050_core::types::State>& last_state,
    const vda5050_core::types::InstantActions& actions) const;

  // Subscribe the fleet fan-out to master_context_'s Provider once, in the
  // constructor: each typed update is routed to the matching observer hook by
  // its agv_id tag. Callbacks fire the hook directly (no AGV lookup — the
  // AGV-local side-effects live on the AGV) and are wrapped so a throwing
  // override can't kill the shared inbound thread.
  void register_event_dispatch();

  // Run one observer hook, swallowing any exception so one bad override can't
  // stall the shared inbound thread.
  void fire_hook(
    const std::string& agv_id, const char* hook_name,
    const std::function<void()>& fn);

  // Build an AGV. Caller holds `agv_mutex_`; insert into `agvs_` and call
  // setup_subscriptions() AFTER releasing it: subscribing under the mutex can
  // deadlock against an inbound on_state -> get_agv() on Paho's network thread.
  std::shared_ptr<AGV> create_agv_locked(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, std::size_t max_queue_size,
    bool drop_oldest);

  // ===========================================================================
  // Member Variables
  // ===========================================================================

  // Shared MQTT client for protocol adapters
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client_;

  // One fleet-wide event detector: every AGV feeds its State / Connection here
  // (via ingest_*), it diffs per-AGV baselines and publishes agv_id-tagged
  // updates that register_event_dispatch() fans out. Declared before agvs_ so
  // it outlives the AGVs that feed it; the fan-out never fires during teardown
  // because AGVs are stopped first and ingest goes through parent_.lock().
  MasterContext master_context_;

  // Onboarded AGVs (shared_ptr allows safe access)
  mutable std::mutex agv_mutex_;
  std::unordered_map<std::string, std::shared_ptr<AGV>> agvs_;

  // Loaded topology map + per-AGV factsheet-alignment cache.
  // Lock order: agv_mutex_ → map_mutex_. Never the reverse.
  mutable std::mutex map_mutex_;
  vda5050_core::layout::Graph::ConstPtr active_graph_;
  std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
    alignment_cache_;

  // Master-broker connection state. Mutated by the MQTT
  // transport thread via the connection-state callbacks registered in
  // connect(); read by get_broker_status() from arbitrary threads.
  mutable std::mutex broker_status_mutex_;
  bool broker_connected_ = false;
  std::optional<std::chrono::system_clock::time_point>
    broker_last_disconnect_at_;
  std::uint64_t broker_reconnect_count_ = 0;

  // Async dispatch correlation, one entry per AGV (keyed like agvs_). See
  // record_assignment / get_active_assignment_id / clear_assignment.
  struct ActiveAssignment
  {
    std::string assignment_id;
    std::string order_id;
    std::uint32_t order_update_id = 0;
  };
  mutable std::mutex assignments_mutex_;
  std::unordered_map<std::string, ActiveAssignment> active_assignments_;

  // Internal handlers for the MQTT transport's connection-state
  // callbacks. Update broker_* state under the mutex, then
  // dispatch to the on_broker_* virtuals OUTSIDE the lock so user
  // code may safely call back into get_broker_status() / publish.
  void handle_broker_connection_lost(const std::string& cause);
  void handle_broker_connected(const std::string& cause);
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__MASTER_HPP_
