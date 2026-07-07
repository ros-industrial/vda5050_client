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
#include "vda5050_core/master/master_types.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/operating_mode.hpp"

namespace vda5050_core {
namespace master {

/// \brief VDA5050 Master for multi-AGV fleet management
///
/// This abstract base class manages VDA5050 communication for multiple AGVs
/// using a single shared MQTT client that creates adapters per AGV.
///
/// Features:
/// - Single shared MQTT client that creates protocol adapters for each AGV
/// - AGV onboarding/offboarding with allow list
/// - Message routing based on topic parsing
/// - Protocol adapters for subscribing and publishing to AGVs
///
/// Thread safety note: Callbacks are invoked on the MQTT client thread.
/// If thread safety is required, the callback implementation should handle
/// synchronization (e.g., using a mutex).
///
/// Construction requirement: VDA5050Master MUST be constructed via
/// `std::make_shared<MyMaster>(...)`. Each onboarded AGV stores a
/// `std::weak_ptr<VDA5050Master>` back-pointer (populated via
/// `weak_from_this()`) and uses it to dispatch incoming messages to the
/// master's virtual callbacks. `weak_from_this()` only returns a valid
/// weak_ptr if the master is currently inside a `shared_ptr` — stack-
/// allocated or `unique_ptr`-managed masters silently no-op on user
/// callbacks.
class VDA5050Master : public std::enable_shared_from_this<VDA5050Master>
{
public:
  /// \brief Construct a master with a shared MQTT client and broker address
  /// \param mqtt_client Shared MQTT client for subscriptions
  ///
  /// The mqtt_client creates protocol adapters for each onboarded AGV.
  ///
  /// IMPORTANT: must be constructed via `std::make_shared<MyMaster>(...)` —
  /// see class doc-comment.
  ///
  /// Example usage:
  /// \code
  /// auto c = transport::create_default_client_shared(broker, "master");
  /// auto master = std::make_shared<MyMaster>(client);
  /// master->connect();
  /// \endcode
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

  // Master-broker connection observability. Surfaces the master's own
  // MQTT-broker connection state to the FMS via the on_broker_* virtuals and
  // the get_broker_status() snapshot. Distinct from per-AGV connection state:
  // an AGV can be ONLINE while the master has lost its broker, and vice versa.

  /// \brief Read-only snapshot of the master's broker-connection state.
  ///
  /// Returned by `get_broker_status()`. All fields are stable across the
  /// lifetime of the snapshot — repeat the call to observe changes.
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

  /// \brief Onboard an AGV to allow message routing
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param max_queue_size Max outgoing messages to queue (default: 10)
  /// \param drop_oldest If true, drop oldest message when queue full; if
  ///   false, reject new message (default: true)
  ///
  /// Creates an AGV instance in the allowed list. Messages from this AGV
  /// will be routed to the appropriate handlers.
  /// The interface name is set to "uagv" by default.
  void onboard_agv(
    const std::string& manufacturer, const std::string& serial_number,
    size_t max_queue_size = 10, bool drop_oldest = true);

  /// \brief Onboard an AGV with a custom interface_name for routing
  /// \param interface_name Interface name
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param max_queue_size Max outgoing messages to queue (default: 10)
  /// \param drop_oldest If true, drop oldest message when queue full; if
  ///   false, reject new message (default: true)
  ///
  /// Creates an AGV instance in the allowed list. Messages from this AGV
  /// will be routed to the appropriate handlers.
  void onboard_agv(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true);

  /// \brief Offboard an AGV to stop message routing
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  ///
  /// Removes the AGV from the allowed list. Messages from this AGV
  /// will be ignored with a warning.
  void offboard_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /// \brief Check if an AGV is onboarded
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \return true if AGV is onboarded
  bool is_agv_onboarded(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ===========================================================================
  // AGV Access
  // ===========================================================================

  /// \brief Get a shared pointer to an onboarded AGV
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \return Shared pointer to AGV, or nullptr if not onboarded
  std::shared_ptr<AGV> get_agv(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ===========================================================================
  // Outgoing Messages
  // ===========================================================================

  /// \brief Publish an order to a specific AGV
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param order The order message
  /// \return true if queued successfully, false if queue is full
  /// \throws std::runtime_error if AGV is not onboarded
  bool publish_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order);

  /// \brief Synchronously check AGV readiness and assign the order.
  ///
  /// Performs caller-visible pre-flight validation BEFORE handing the
  /// order to the async publish queue. Implements the master-side
  /// "vehicle is in a state to receive an order" check.
  ///
  /// Pre-flight (synchronous, on caller thread):
  ///   1. AGV onboarded with this manufacturer/serial?
  ///   2. Connection ONLINE?
  ///   3. Operational state AVAILABLE (not ERROR/UNAVAILABLE/UNKNOWN)?
  ///   4. Last State has operating_mode == AUTOMATIC?
  ///   5. Last State has position_initialized == true?
  ///   6. If `order` is an update for the active order: would the
  ///      stitcher accept it (SEND_NOW), queue it (QUEUE_PENDING),
  ///      or reject it (REJECT)?
  ///
  /// On ASSIGNED: order is queued via `AGV::send_order`; the async
  /// validator chain (schema/PreSend/graph/traversability) runs later
  /// on the queue-processor thread as defense-in-depth.
  ///
  /// On STITCH_QUEUED: order is enqueued in the lifecycle's pending
  /// updates; will be drained when AGV state reaches the stitch point.
  ///
  /// On any rejection: returns an AssignmentDecision identifying which
  /// check failed plus diagnostic errors; nothing is queued.
  ///
  /// Recommended FMS-facing entry point. The bool-returning
  /// `publish_order(mfg, serial, order)` is kept as a lower-level API
  /// that bypasses the synchronous pre-flight.
  AssignmentResult assign_order(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::Order& order);

  // ===========================================================================
  // Batch onboarding — Device Manager integration. The body is a
  // sequential loop under a single lock; signature is stable for a
  // future batched MQTT SUBSCRIBE swap.
  // ===========================================================================

  /// One AGV's slot in a batch onboarding request. Same parameters as
  /// `onboard_agv()`.
  struct OnboardSpec
  {
    std::string manufacturer;
    std::string serial_number;
    std::size_t max_queue_size = 10;
    bool drop_oldest = true;
  };

  /// Summary of a batch call. Never throws; per-entry outcomes are
  /// classified into three lists so callers can report which AGV
  /// landed in which bucket. Counts are `.size()` of each list.
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
  // Assignment correlation — caller UUID stash for async dispatch.
  // At most one active assignment per (mfg, serial); a second record
  // overwrites. Protected by `assignments_mutex_`; never held with
  // `agv_mutex_`.
  // ===========================================================================

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

  /// \brief Publish instant actions to a specific AGV
  /// \param manufacturer Manufacturer name
  /// \param serial_number Serial number
  /// \param actions The instant actions message
  /// \return true if queued successfully, false if queue is full
  /// \throws std::runtime_error if AGV is not onboarded
  bool publish_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  /// \brief Synchronously check AGV reachability and dispatch
  ///        instantActions.
  ///
  /// Mirrors the assign_order pattern for instant actions, but with
  /// a deliberately lighter pre-flight: instantActions are designed to
  /// function in degraded states (cancelOrder during ERROR, factsheetRequest
  /// before any state report, initPosition before position is initialized).
  ///
  /// Pre-flight (synchronous, on caller thread):
  ///   1. AGV onboarded with this manufacturer/serial?
  ///   2. Connection ONLINE? (QoS 0 instantActions delivered only to live
  ///      connections; sending to OFFLINE AGV is a silent drop.)
  ///   3. action_id uniqueness — within the candidate batch, vs
  ///      `state.action_states[].action_id`, vs the active order's node /
  ///      edge action_ids. The spec requires global
  ///      uniqueness; AGV behaviour on collision is undefined.
  ///
  /// Skipped (with rationale): operational_state, last_state existence,
  /// operating_mode, position_initialized — all four are designed to be
  /// bypassable for the recovery / commissioning use cases instantActions
  /// exist for.
  ///
  /// On ASSIGNED: actions are queued via `AGV::send_instant_actions`; the
  /// async validator chain (schema, pre-send, traversability, capability)
  /// runs on the queue-processor thread as defense-in-depth.
  ///
  /// On any rejection: returns an InstantActionDecision identifying which
  /// check failed plus diagnostic errors; nothing is queued.
  ///
  /// Recommended FMS-facing entry point. The bool-returning
  /// `publish_instant_actions(mfg, serial, actions)` is kept as a
  /// lower-level API that bypasses the synchronous pre-flight.
  InstantActionAssignmentResult assign_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const vda5050_core::types::InstantActions& actions);

  // ==========================================================================
  // Topology layout
  // ==========================================================================
  //
  // The master loads a topology layout (LIF) from a JSON config file at
  // startup and cross-checks it against onboarded AGVs' factsheets. The
  // loaded graph is consulted by the traversability validator.
  //
  // Concurrency: `active_graph_` and `alignment_cache_` are protected by
  // `map_mutex_`. Readers copy the `Graph::ConstPtr` snapshot under the
  // mutex, then release it before doing work — the graph is immutable so
  // concurrent reads are safe. Lock order when both `agv_mutex_` and
  // `map_mutex_` are needed: `agv_mutex_` → `map_mutex_`.

  /// \brief Load a topology layout (LIF) from a JSON config file.
  ///
  /// On success, swaps the master's active graph and re-runs factsheet
  /// alignment against every onboarded AGV. On failure the previously
  /// loaded graph (if any) is left unchanged.
  ///
  /// \param path  path to the LIF JSON config
  /// \return load result; on success `lif` holds the parsed layout
  vda5050_core::layout::LayoutLoadResult load_layout_from_config(
    const std::string& path);

  /// \brief Install an already-built graph directly. Used by tests and by
  /// integrators who load the layout by other means; triggers the same
  /// alignment refresh as load_layout_from_config.
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
  // These virtuals fire after the per-AGV ProtocolAdapter receives and
  // deserializes a message, and after the owning AGV instance has cached
  // it. They are dispatched via a back-pointer the AGV holds to its
  // owning master. Default implementations are empty — subclass and
  // override to plug in fleet-level reaction logic.
  //
  // Threading: invoked on the Paho MQTT callback thread (not the user's
  // main thread). User overrides must be thread-safe with respect to
  // any state they touch.

  /// \brief Called after a State message arrives and is cached on the AGV.
  /// \param agv_id  manufacturer/serial composite ID
  /// \param state   the parsed State message
  virtual void on_state(
    const std::string& agv_id, const vda5050_core::types::State& state);

  /// \brief Called after a Connection message arrives and is cached.
  virtual void on_connection(
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
  // Fired by AGV's orchestration callbacks after the per-message
  // detector classifies a transition. Default implementations are
  // empty — override only the events your master cares about.
  //
  // These are layered ON TOP of on_state / on_connection above. The
  // raw-message virtuals still fire; these are the named, edge-detected
  // convenience hooks.

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

  /// \brief Fired when curr.operating_mode != prev.operating_mode.
  ///
  /// **Leave / return AUTOMATIC edges**:
  ///
  /// When `prev_mode == AUTOMATIC && new_mode != AUTOMATIC` (leave
  /// edge), the library has ALREADY captured the AGV's outbound
  /// order / instant-action queues into a resumable buffer and
  /// drained the live queues, before this callback fires. FMS may
  /// inspect the buffer via `agv->get_mode_cancelled_queue()`.
  ///
  /// When `new_mode == AUTOMATIC && prev_mode != AUTOMATIC` (return
  /// edge), FMS **should** call exactly ONE of:
  ///   - `agv->resume_mode_cancelled_queue()` — prepend buffered
  ///     items to the front of the live queue (preserves original
  ///     dispatch order; buffered items execute before any new
  ///     orders FMS dispatched in the meantime)
  ///   - `agv->discard_mode_cancelled_queue()` — drop the buffer
  ///
  /// If neither is called, the buffer will be **overwritten** on
  /// the next leave-AUTOMATIC and the items silently lost. Don't
  /// rely on long-term retention.
  ///
  /// Recommended pattern — handle resume/discard inside this
  /// override on the return edge:
  ///
  /// \code
  /// void on_mode_changed(id, new_mode, prev_mode) override {
  ///   if (new_mode == AUTOMATIC && prev_mode != AUTOMATIC) {
  ///     get_agv(id)->resume_mode_cancelled_queue();
  ///     // OR: get_agv(id)->discard_mode_cancelled_queue();
  ///   }
  /// }
  /// \endcode
  ///
  /// Pure-ROS-2 FMS (no C++ subclass) can call the resume / discard
  /// via the `/<namespace>/resume_mode_cancelled_queue` and
  /// `/<namespace>/discard_mode_cancelled_queue` services exposed
  /// by `VDA5050MasterROS2`.
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
  // Fired by AGV's connection_subscriber callback after the
  // connection_update_detector classifies the transition. Three distinct
  // event types correspond to the three `connectionState` values. The
  // existing `on_connection` virtual still fires for raw access to
  // every Connection message; these named virtuals are additive
  // convenience hooks.

  /// \brief Fired when AGV's connection_state transitions to ONLINE.
  ///        Initial connect or reconnect after recovery.
  virtual void on_connect(const std::string& agv_id);

  /// \brief Fired when AGV's connection_state transitions to OFFLINE
  ///        (graceful shutdown — AGV explicitly published OFFLINE
  ///        before disconnecting).
  virtual void on_offline(const std::string& agv_id);

  /// \brief Fired when AGV's connection_state transitions to
  ///        CONNECTIONBROKEN — the broker auto-published the AGV's
  ///        pre-registered last-will message because the AGV's TCP
  ///        connection unexpectedly dropped.
  ///
  /// **This is the last-will handler.** Use this to react to
  /// unexpected AGV death (cancel pending orders via
  /// `AGV::cancel_pending_orders()`, alert FMS, page on-call, etc.).
  ///
  /// Note: the Connection message that triggered this event has stale
  /// `timestamp` / `headerId` fields (registered at AGV connect time,
  /// not delivery time).
  virtual void on_connection_broken(const std::string& agv_id);

  // ===========================================================================
  // State-heartbeat event triggers
  // ===========================================================================
  //
  // Fired by the AGV's state-topic HeartbeatListener when the spec's
  // 30s state-publish window is violated, and
  // again on the recovery edge when state messages resume. Layered
  // ON TOP of the existing `on_state` raw-message virtual:
  // `on_state_timeout` / `on_state_resumed` are the named edges,
  // `on_state` still fires for every State message including the
  // recovery message.

  /// \brief Fired when the AGV's state-topic heartbeat exceeds the
  ///        spec's 30s window with no new State message.
  ///
  /// The library has already flipped `operational_state` to
  /// `STATE_UNKNOWN` and the pre-send validator chain will now
  /// hard-reject orders queued for this AGV. This virtual is the
  /// FMS reaction surface — alert operators, page on-call, kick
  /// off recovery probes, etc. Default impl is empty.
  ///
  /// The library does NOT auto-cancel pending orders on this event
  /// (state silence is potentially transient — AGV power-cycle,
  /// broker reconnect, network blip). FMS overrides that want
  /// aggressive cancellation can call
  /// `AGV::cancel_pending_orders()` here. Compare with
  /// `on_connection_broken`, which IS auto-cancelled because TCP
  /// last-will is a hard signal.
  ///
  /// Threading: invoked on the HeartbeatListener's monitor thread.
  /// Override must be thread-safe with respect to any state it
  /// touches.
  virtual void on_state_timeout(const std::string& agv_id);

  /// \brief Fired when the first State message arrives after a
  ///        previously-fired `on_state_timeout` — the silence-to-active
  ///        edge.
  ///
  /// Pairs with `on_state_timeout`: every timeout fires exactly one
  /// recovery (or zero, if the AGV stays silent until offboard). Use
  /// to log recovery duration, re-issue stale orders, page resolved
  /// alerts. Default impl is empty.
  ///
  /// Also fires once on the AGV's first-ever State message (initial
  /// `STATE_UNKNOWN` → `AVAILABLE`). FMS that needs to distinguish
  /// fresh-start from post-silence recovery uses the connection
  /// events (`on_connect`).
  ///
  /// Threading: invoked on the AGV's state-subscriber thread,
  /// BEFORE the user's `on_state(...)` virtual.
  virtual void on_state_resumed(const std::string& agv_id);

  // ===========================================================================
  // Master-broker connection event triggers
  // ===========================================================================
  //
  // Fired when the underlying MQTT client reports the master's own
  // connection to the broker has been lost or (re)established. Distinct
  // from `on_connect` / `on_offline` / `on_connection_broken`, which
  // report per-AGV last-will / heartbeat events. These virtuals are
  // additive to the existing per-AGV surface — the AGV-side virtuals
  // still fire when AGVs publish on the connection topic.
  //
  // Threading: invoked on the MQTT transport's I/O thread. Override
  // implementations must be thread-safe with respect to any state
  // they touch and should return promptly to avoid stalling the
  // transport's reconnect loop.

  /// \brief Fired when the master's MQTT-broker connection drops.
  ///
  /// The library has already updated `get_broker_status()` to
  /// `connected = false` and stamped `last_disconnect_at = now()` before
  /// this virtual is dispatched. FMS use this to alert operators that
  /// orders cannot currently flow through to AGVs (queued orders will
  /// remain queued and will retry once Paho's auto-reconnect succeeds).
  ///
  /// Default impl is empty.
  virtual void on_broker_disconnected();

  /// \brief Fired when the master's MQTT-broker connection is
  ///        (re)established.
  ///
  /// Fires once on initial successful connect AND once on every Paho-
  /// driven auto-reconnect. The library has already updated
  /// `get_broker_status()` to `connected = true` and incremented
  /// `reconnect_count` before this virtual is dispatched. FMS may
  /// inspect `get_broker_status().reconnect_count` to distinguish
  /// the first connect from a recovery.
  ///
  /// Default impl is empty.
  virtual void on_broker_reconnected();

private:
  // ===========================================================================
  // Internal AGV lookup
  // ===========================================================================

  std::shared_ptr<AGV> get_agv_by_id(const std::string& agv_id) const;

  // Build an AGV instance. Caller must hold `agv_mutex_` and is
  // responsible for inserting the returned shared_ptr into `agvs_` and
  // calling `setup_subscriptions()` AFTER releasing `agv_mutex_` —
  // SUBSCRIBE can race with inbound PUBLISH on Paho's network thread,
  // and the resulting on_state -> get_agv() callback would deadlock if
  // we held the mutex while subscribing.
  std::shared_ptr<AGV> create_agv_locked(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, std::size_t max_queue_size,
    bool drop_oldest);

  // ===========================================================================
  // Member Variables
  // ===========================================================================

  // Shared MQTT client for protocol adapters
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client_;

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

  // Active assignment_id per onboarded AGV (async dispatch
  // correlation). One entry per (mfg, serial); a second record
  // overwrites. Map key is the same `{mfg}/{serial}` id format used
  // by agvs_. See record_assignment / get_active_assignment_id /
  // clear_assignment for the contract.
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
