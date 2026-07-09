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

#include "vda5050_core/master/agv.hpp"

#include <algorithm>
#include <utility>

#include "nlohmann/json.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/master.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/validation/content_validator.hpp"
#include "vda5050_core/validation/pre_send_validator.hpp"

namespace vda5050_core::master {

namespace {

// Human-readable label for a stitch GuardFailure, used only in log
// messages. Stable strings — FMS log scrapers may match on these.
const char* guard_failure_to_str(GuardFailure g)
{
  switch (g)
  {
    case GuardFailure::ORDER_ID_MISMATCH:
      return "order_id mismatch";
    case GuardFailure::NO_STATE_YET:
      return "AGV has not reported State yet";
    case GuardFailure::PREV_UPDATE_NOT_CONFIRMED:
      return "previous order_update_id not confirmed";
    case GuardFailure::NONE:
    default:
      return "none";
  }
}

// Age of a cached sample at "now", floored at zero so a backward wall-clock
// step never yields a negative duration.
std::chrono::nanoseconds age_since(const AGV::TimePoint& tp)
{
  return std::max(
    std::chrono::nanoseconds::zero(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      AGV::Clock::now() - tp));
}

}  // namespace

// ============================================================================
// Constructor / Destructor
// ============================================================================

AGV::AGV(
  std::shared_ptr<vda5050_core::execution::ProtocolAdapter> protocol_adapter,
  const std::string& interface_name, const std::string& manufacturer,
  const std::string& serial_number, size_t max_queue_size, bool drop_oldest,
  int state_heartbeat_interval, std::weak_ptr<VDA5050Master> parent)
: interface_name_(interface_name),
  manufacturer_(manufacturer),
  serial_number_(serial_number),
  agv_id_(manufacturer + "/" + serial_number),
  protocol_adapter_(protocol_adapter),
  order_lifecycle_(agv_id_),
  parent_(parent),
  parent_raw_(parent.lock().get()),
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
    protocol_adapter_->unsubscribe<vda5050_core::types::Connection>();
    protocol_adapter_->unsubscribe<vda5050_core::types::State>();
    protocol_adapter_->unsubscribe<vda5050_core::types::Factsheet>();
    protocol_adapter_->unsubscribe<vda5050_core::types::Visualization>();
  }
  protocol_adapter_.reset();

  VDA5050_INFO("[AGV] AGV instance destroyed: {}", agv_id_);
}

void AGV::setup_subscriptions()
{
  if (!protocol_adapter_)
  {
    return;
  }

  create_subscription<vda5050_core::types::Connection>(
    [this](const auto& msg) { handle_connection(msg); }, ConnectionQos);
  create_subscription<vda5050_core::types::State>(
    [this](const auto& msg) { handle_state(msg); }, StateQos);
  create_subscription<vda5050_core::types::Factsheet>(
    [this](const auto& msg) { handle_factsheet(msg); }, FactsheetQos);
  create_subscription<vda5050_core::types::Visualization>(
    [this](const auto& msg) { handle_visualization(msg); }, VisualizationQos);
}

void AGV::stop()
{
  VDA5050_INFO("[AGV] Stopping AGV: {}", agv_id_);

  // Stop queue processor and heartbeat
  stop_queue_processor();
  cleanup_heartbeat();

  // Reset states
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connection_status_ = vda5050_core::types::ConnectionState::OFFLINE;
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

  // Clear order lifecycle tracking (no active order, no pending updates)
  // — the AGV's prior order context is no longer valid after restart.
  order_lifecycle_.clear();

  VDA5050_INFO("[AGV] AGV restarted, ready for connections: {}", agv_id_);
}

void AGV::pause()
{
  VDA5050_INFO("[AGV] Pausing AGV: {}", agv_id_);

  stop_queue_processor();
  cleanup_heartbeat();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    connection_status_ = vda5050_core::types::ConnectionState::OFFLINE;
    operational_state_ = AGVState::UNAVAILABLE;
  }

  VDA5050_INFO("[AGV] AGV paused: {}", agv_id_);
}

void AGV::resume()
{
  VDA5050_INFO("[AGV] Resuming AGV: {}", agv_id_);

  setup_heartbeat();
  start_queue_processor();

  VDA5050_INFO("[AGV] AGV resumed: {}", agv_id_);
}

// ============================================================================
// Connection and Operational State
// ============================================================================

bool AGV::is_connected() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_status_ == vda5050_core::types::ConnectionState::ONLINE;
}

vda5050_core::types::ConnectionState AGV::get_connection_status() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_status_;
}

AGVState AGV::get_operational_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return operational_state_;
}

void AGV::set_connection_status(vda5050_core::types::ConnectionState status)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto old_status = connection_status_;
  connection_status_ = status;

  // When connection is lost, AGV becomes unavailable
  if (
    status == vda5050_core::types::ConnectionState::OFFLINE ||
    status == vda5050_core::types::ConnectionState::CONNECTIONBROKEN)
  {
    if (operational_state_ != AGVState::UNAVAILABLE)
    {
      operational_state_ = AGVState::UNAVAILABLE;
      VDA5050_INFO(
        "[AGV] Operational state changed to UNAVAILABLE for {} (connection {})",
        agv_id_,
        status == vda5050_core::types::ConnectionState::OFFLINE
          ? "OFFLINE"
          : "CONNECTIONBROKEN");
    }
  }

  // Log connection status change
  if (old_status != status)
  {
    const char* status_str = "UNKNOWN";
    switch (status)
    {
      case vda5050_core::types::ConnectionState::ONLINE:
        status_str = "ONLINE";
        break;
      case vda5050_core::types::ConnectionState::OFFLINE:
        status_str = "OFFLINE";
        break;
      case vda5050_core::types::ConnectionState::CONNECTIONBROKEN:
        status_str = "CONNECTIONBROKEN";
        break;
    }
    VDA5050_INFO(
      "[AGV] Connection status changed to {} for {}", status_str, agv_id_);
  }
}

void AGV::set_operational_state(AGVState state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Precedence: connection-loss UNAVAILABLE and fault ERROR outrank the
  // heartbeat-timer STATE_UNKNOWN, so a state-timeout on a disconnected AGV
  // can't mask "connection lost" as "STATE_UNKNOWN". Recovery is unaffected:
  // a valid State sets AVAILABLE, which skips this guard.
  if (
    state == AGVState::STATE_UNKNOWN &&
    (operational_state_ == AGVState::UNAVAILABLE ||
     operational_state_ == AGVState::ERROR))
  {
    return;
  }

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

  // Dispatch the named timeout edge to the user. Library
  // does NOT auto-cancel pending orders here — silence is potentially
  // transient. The pre-send validator chain hard-rejects orders for
  // STATE_UNKNOWN AGVs at publish time as defense-in-depth.
  if (auto p = parent_.lock())
  {
    p->on_state_timeout(agv_id_);
  }
}

// ============================================================================
// Heartbeat Management
// ============================================================================

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

// ============================================================================
// Message Handlers
// ============================================================================

void AGV::handle_connection(const vda5050_core::types::Connection& msg)
{
  // Schema gate. Drop malformed messages before they touch
  // cache, heartbeat, or user callbacks.
  auto schema_result =
    vda5050_core::validation::validate_connection_content(msg);
  if (!schema_result)
  {
    VDA5050_WARN(
      "[AGV] Dropping malformed connection from {}: {} schema error(s)",
      agv_id_, schema_result.fatal_errors().size());
    return;
  }

  // Update cached message
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_connection_ = msg;
    last_connection_time_ = Clock::now();
  }

  const auto prev_status = get_connection_status();
  set_connection_status(msg.connection_state);

  // Manage heartbeat based on connection state
  if (msg.connection_state == vda5050_core::types::ConnectionState::ONLINE)
  {
    // Start heartbeat and queue processor when ONLINE
    setup_heartbeat();
    start_queue_processor();
    // Reset the stale-State gate on the reconnect edge only (ONLINE recurs as
    // a heartbeat) so a restarted AGV isn't locked out.
    if (prev_status != vda5050_core::types::ConnectionState::ONLINE)
    {
      order_lifecycle_.reset_state_baseline();
    }
  }
  else
  {
    // Stop heartbeat and queue processor when OFFLINE/CONNECTIONBROKEN
    cleanup_heartbeat();
    stop_queue_processor();
  }

  // Last-will: clear the outbound queues and stale pending stitch updates
  // before the fleet detector fires on_connection_broken.
  if (
    msg.connection_state ==
    vda5050_core::types::ConnectionState::CONNECTIONBROKEN)
  {
    cancel_pending_orders();
    order_lifecycle_.clear_pending();
  }

  // Dispatch to the user override, then feed the fleet event detector, which
  // diffs the connection and fans the transition out to the named hooks.
  if (auto p = parent_.lock())
  {
    p->on_connection(agv_id_, msg);
    p->ingest_connection(agv_id_, msg);
  }
}

void AGV::cancel_pending_orders()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  order_queue_ = {};
  instant_actions_queue_ = {};
  VDA5050_INFO("[AGV] Cleared pending outbound queues for {}", agv_id_);
}

// ============================================================================
// Mode-cancelled queue
// ============================================================================

void AGV::capture_and_drain_on_leave_automatic(
  vda5050_core::types::OperatingMode from,
  vda5050_core::types::OperatingMode to)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  mode_cancelled_queue_.orders.clear();
  mode_cancelled_queue_.instant_actions.clear();
  while (!order_queue_.empty())
  {
    mode_cancelled_queue_.orders.push_back(
      std::move(order_queue_.front().order));
    order_queue_.pop();
  }
  while (!instant_actions_queue_.empty())
  {
    mode_cancelled_queue_.instant_actions.push_back(
      std::move(instant_actions_queue_.front()));
    instant_actions_queue_.pop();
  }
  mode_cancelled_queue_.cancelled_at = Clock::now();
  mode_cancelled_queue_.from_mode = from;
  mode_cancelled_queue_.to_mode = to;

  if (
    !mode_cancelled_queue_.orders.empty() ||
    !mode_cancelled_queue_.instant_actions.empty())
  {
    VDA5050_WARN(
      "[AGV] {} left AUTOMATIC — captured {} order(s) + {} instant "
      "action(s) into resumable buffer",
      agv_id_, mode_cancelled_queue_.orders.size(),
      mode_cancelled_queue_.instant_actions.size());
  }
  else
  {
    VDA5050_INFO(
      "[AGV] {} left AUTOMATIC — outbound queues already empty", agv_id_);
  }
}

AGV::ModeCancelledQueue AGV::get_mode_cancelled_queue() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return mode_cancelled_queue_;
}

std::pair<std::size_t, std::size_t> AGV::resume_mode_cancelled_queue()
{
  // Atomic prepend: build a reordered queue with buffer items first +
  // existing live queue items second, then swap. Single lock — no
  // nested-lock window. Preserves FMS-intended ordering: buffered
  // orders execute BEFORE any new orders dispatched between AUTOMATIC
  // return and this call.
  std::lock_guard<std::mutex> lock(queue_mutex_);
  const std::size_t orders_resumed = mode_cancelled_queue_.orders.size();
  const std::size_t actions_resumed =
    mode_cancelled_queue_.instant_actions.size();

  if (orders_resumed > 0)
  {
    std::queue<QueuedOrder> reordered;
    for (auto& o : mode_cancelled_queue_.orders)
    {
      // Resumed orders re-run the stitch decision — the world moved on while
      // parked out of AUTOMATIC.
      reordered.push(QueuedOrder{std::move(o), false});
    }
    while (!order_queue_.empty())
    {
      reordered.push(std::move(order_queue_.front()));
      order_queue_.pop();
    }
    std::swap(order_queue_, reordered);
  }
  if (actions_resumed > 0)
  {
    std::queue<vda5050_core::types::InstantActions> reordered;
    for (auto& a : mode_cancelled_queue_.instant_actions)
    {
      reordered.push(std::move(a));
    }
    while (!instant_actions_queue_.empty())
    {
      reordered.push(std::move(instant_actions_queue_.front()));
      instant_actions_queue_.pop();
    }
    std::swap(instant_actions_queue_, reordered);
  }
  mode_cancelled_queue_ = ModeCancelledQueue{};

  if (orders_resumed > 0 || actions_resumed > 0)
  {
    VDA5050_INFO(
      "[AGV] {} resumed {} order(s) + {} instant action(s) at front of "
      "live queue",
      agv_id_, orders_resumed, actions_resumed);
    queue_cv_.notify_all();
  }
  return {orders_resumed, actions_resumed};
}

std::pair<std::size_t, std::size_t> AGV::discard_mode_cancelled_queue()
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  const auto orders_n = mode_cancelled_queue_.orders.size();
  const auto actions_n = mode_cancelled_queue_.instant_actions.size();
  mode_cancelled_queue_ = ModeCancelledQueue{};
  if (orders_n > 0 || actions_n > 0)
  {
    VDA5050_INFO(
      "[AGV] {} discarded {} order(s) + {} instant action(s) from "
      "mode-cancelled buffer",
      agv_id_, orders_n, actions_n);
  }
  return {orders_n, actions_n};
}

void AGV::handle_state(const vda5050_core::types::State& msg)
{
  // Schema gate. Drop malformed messages before they touch
  // cache, heartbeat, event detection, or user callbacks.
  auto schema_result = vda5050_core::validation::validate_state_content(msg);
  if (!schema_result)
  {
    VDA5050_WARN(
      "[AGV] Dropping malformed state from {}: {} schema error(s)", agv_id_,
      schema_result.fatal_errors().size());
    return;
  }

  // Prior operating mode, read before the cache is overwritten — used for the
  // AGV's own leave-AUTOMATIC queue capture below.
  std::optional<vda5050_core::types::OperatingMode> prev_mode;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_state_) prev_mode = last_state_->operating_mode;
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

  // Capture prior operational state BEFORE flipping to AVAILABLE — used
  // to detect the STATE_UNKNOWN→AVAILABLE recovery edge.
  // get_operational_state() takes data_mutex_ once; result is a stack-
  // local snapshot.
  const auto prev_op_state = get_operational_state();

  // Update operational state to AVAILABLE
  set_operational_state(AGVState::AVAILABLE);

  // Runs before the user callback so observers see current lifecycle state.
  // Drained updates are enqueued pre_stitched (they already cleared the guard).
  auto ready_updates = order_lifecycle_.on_state_update(msg);
  for (std::size_t i = 0; i < ready_updates.size(); ++i)
  {
    if (enqueue_order(ready_updates[i], true)) continue;
    // Outbound queue full — re-queue this and the rest to pending, in order,
    // so a later State retries and no update is sent ahead of an earlier one.
    VDA5050_WARN(
      "[AGV] Outbound queue full for {}; re-queuing {} order update(s)",
      agv_id_, ready_updates.size() - i);
    for (std::size_t j = i; j < ready_updates.size(); ++j)
    {
      order_lifecycle_.enqueue_pending_update(ready_updates[j]);
    }
    break;
  }

  // Fires once on the STATE_UNKNOWN edge (first State, or post-silence
  // recovery), before the user's on_state hook.
  if (prev_op_state == AGVState::STATE_UNKNOWN)
  {
    if (auto p = parent_.lock())
    {
      p->on_state_resumed(agv_id_);
    }
  }

  // Capture+drain the outbound queues before on_mode_changed fires, so an
  // override sees the buffer populated.
  if (
    prev_mode == vda5050_core::types::OperatingMode::AUTOMATIC &&
    msg.operating_mode != vda5050_core::types::OperatingMode::AUTOMATIC)
  {
    capture_and_drain_on_leave_automatic(*prev_mode, msg.operating_mode);
  }

  // Dispatch to the user override, then feed the fleet event detector, which
  // diffs the state and fans each transition out to the named hooks (the first
  // State only seeds — no hooks fire).
  if (auto p = parent_.lock())
  {
    p->on_state(agv_id_, msg);
    p->ingest_state(agv_id_, msg);
  }
}

void AGV::handle_factsheet(const vda5050_core::types::Factsheet& msg)
{
  // Schema gate.
  auto schema_result =
    vda5050_core::validation::validate_factsheet_content(msg);
  if (!schema_result)
  {
    VDA5050_WARN(
      "[AGV] Dropping malformed factsheet from {}: {} schema error(s)", agv_id_,
      schema_result.fatal_errors().size());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_factsheet_ = msg;
    last_factsheet_time_ = Clock::now();
  }

  // Dispatch to master: first refresh alignment cache (
  // symmetric trigger), then invoke user override.
  if (auto p = parent_.lock())
  {
    p->refresh_alignment_for_agv(agv_id_, msg);
    p->on_factsheet(agv_id_, msg);
  }
}

void AGV::handle_visualization(const vda5050_core::types::Visualization& msg)
{
  // Schema gate.
  auto schema_result =
    vda5050_core::validation::validate_visualization_content(msg);
  if (!schema_result)
  {
    VDA5050_WARN(
      "[AGV] Dropping malformed visualization from {}: {} schema error(s)",
      agv_id_, schema_result.fatal_errors().size());
    return;
  }

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

// ============================================================================
// Cached Messages - Get
// ============================================================================

std::optional<vda5050_core::types::Connection> AGV::get_last_connection() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_connection_;
}

std::optional<vda5050_core::types::State> AGV::get_last_state() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_;
}

std::optional<vda5050_core::types::Factsheet> AGV::get_last_factsheet() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_factsheet_;
}

std::optional<vda5050_core::types::Visualization> AGV::get_last_visualization()
  const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_visualization_;
}

AGV::StatusSnapshot AGV::get_status_snapshot() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return StatusSnapshot{last_state_,           last_connection_,
                        last_factsheet_,       last_state_time_,
                        last_connection_time_, last_factsheet_time_};
}

AGV::OrderStatusBundle AGV::get_order_status_bundle() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return OrderStatusBundle{
    last_state_, last_state_time_, order_lifecycle_.snapshot(),
    order_lifecycle_.pending_update_count()};
}

PoseView AGV::get_pose_view() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  PoseView view;

  // Visualization carries no driving flag; relay it from State.
  if (last_state_) view.driving = last_state_->driving;

  // Latest-wins between State and Visualization by AGV header timestamp,
  // considering only sources that actually carry a position. Both timestamps
  // come from the same AGV clock, so they are directly comparable.
  const bool state_has_pos = last_state_ && last_state_->agv_position;
  const bool viz_has_pos =
    last_visualization_ && last_visualization_->agv_position;

  bool use_viz = viz_has_pos;
  if (state_has_pos && viz_has_pos)
  {
    use_viz =
      last_visualization_->header.timestamp >= last_state_->header.timestamp;
  }

  // Position and velocity always come from the same source (never mix a
  // position from one with a velocity from the other). data_age uses the
  // master receive time so it stays on a single clock.
  if (use_viz)
  {
    view.source = PoseSource::Visualization;
    view.agv_position = last_visualization_->agv_position;
    view.velocity = last_visualization_->velocity;
    view.data_age = age_since(*last_visualization_time_);
  }
  else if (state_has_pos)
  {
    view.source = PoseSource::State;
    view.agv_position = last_state_->agv_position;
    view.velocity = last_state_->velocity;
    view.data_age = age_since(*last_state_time_);
  }

  return view;
}

// ============================================================================
// Order Lifecycle (forwarders to OrderLifecycleManager)
// ============================================================================

bool AGV::has_active_order() const
{
  return order_lifecycle_.has_active_order();
}

std::optional<std::string> AGV::active_order_id() const
{
  return order_lifecycle_.active_order_id();
}

std::optional<uint32_t> AGV::active_order_update_id() const
{
  return order_lifecycle_.active_order_update_id();
}

bool AGV::is_order_complete() const
{
  return order_lifecycle_.is_order_complete();
}

bool AGV::active_order_needs_more_base() const
{
  return order_lifecycle_.active_order_needs_more_base();
}

size_t AGV::pending_update_count() const
{
  return order_lifecycle_.pending_update_count();
}

ActiveOrderSnapshot AGV::active_order_snapshot() const
{
  return order_lifecycle_.snapshot();
}

// ============================================================================
// Timestamps
// ============================================================================

std::optional<AGV::TimePoint> AGV::get_last_connection_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_connection_time_;
}

std::optional<AGV::TimePoint> AGV::get_last_state_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_state_time_;
}

std::optional<AGV::TimePoint> AGV::get_last_factsheet_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_factsheet_time_;
}

std::optional<AGV::TimePoint> AGV::get_last_visualization_time() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_visualization_time_;
}

// ============================================================================
// Outgoing Messages - Queue
// ============================================================================

bool AGV::send_order(const vda5050_core::types::Order& order)
{
  return enqueue_order(order, false);
}

bool AGV::enqueue_order(
  const vda5050_core::types::Order& order, bool pre_stitched)
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

  order_queue_.push(QueuedOrder{order, pre_stitched});
  queue_cv_.notify_one();

  VDA5050_INFO("[AGV] Queued order for AGV: {}", agv_id_);
  return true;
}

bool AGV::send_instant_actions(
  const vda5050_core::types::InstantActions& actions)
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

size_t AGV::get_pending_order_count() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return order_queue_.size();
}

size_t AGV::get_pending_instant_actions_count() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return instant_actions_queue_.size();
}

// ============================================================================
// Queue Processing
// ============================================================================

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

void AGV::stop_queue_processor()
{
  std::thread thread_to_join;

  {
    std::lock_guard<std::mutex> lock(thread_mutex_);

    // Always signal stop + steal the thread handle, even when
    // queue_processor_running_ is false. Without this, a destructor
    // racing against a freshly-started queue thread (e.g. start fired
    // from handle_connection on the test thread, dtor entered before
    // the queue thread first observed stop_processing_) could leave
    // queue_thread_ joinable when member destructors run -> std::thread
    // dtor calls std::terminate.
    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      stop_processing_ = true;
    }
    queue_cv_.notify_all();

    if (queue_processor_running_ || queue_thread_.joinable())
    {
      VDA5050_INFO("[AGV] Stopping queue processor for {}", agv_id_);
    }

    thread_to_join = std::move(queue_thread_);
    queue_processor_running_ = false;
  }

  if (thread_to_join.joinable())
  {
    thread_to_join.join();
    VDA5050_INFO("[AGV] Queue processor stopped for {}", agv_id_);
  }
}

void AGV::process_queues()
{
  VDA5050_INFO("[AGV] Queue processing thread started for {}", agv_id_);

  while (true)
  {
    std::optional<QueuedOrder> order;
    std::optional<vda5050_core::types::InstantActions> actions;

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
      publish_order(order->order, order->pre_stitched);
    }
  }

  VDA5050_INFO("[AGV] Queue processing thread stopped for {}", agv_id_);
}

// ============================================================================
// Publishing
// ============================================================================

void AGV::publish_order(
  const vda5050_core::types::Order& order, bool pre_stitched)
{
  if (!protocol_adapter_)
  {
    VDA5050_WARN(
      "[AGV] Cannot publish order: no protocol adapter for {}", agv_id_);
    return;
  }

  const auto snap = order_lifecycle_.snapshot();

  // A drained update already cleared the guard; re-deciding could wrongly
  // reject it if the state advanced meanwhile.
  if (!pre_stitched)
  {
    const auto stitch = order_stitcher_.decide(order, snap);
    switch (stitch.decision)
    {
      case StitchDecision::SEND_NOW:
        break;
      case StitchDecision::IGNORE:
        VDA5050_INFO(
          "[AGV] Ignoring duplicate order {} (update {}) for {}: already "
          "applied",
          order.order_id, order.order_update_id, agv_id_);
        return;
      case StitchDecision::QUEUE_PENDING:
        if (!order_lifecycle_.enqueue_pending_update(order))
        {
          VDA5050_WARN(
            "[AGV] Pending queue full for {}; dropping order {} (update {})",
            agv_id_, order.order_id, order.order_update_id);
        }
        else
        {
          VDA5050_INFO(
            "[AGV] Queued order {} (update {}) for {}: {}", order.order_id,
            order.order_update_id, agv_id_,
            guard_failure_to_str(stitch.first_failed_guard));
        }
        return;
      case StitchDecision::REJECT:
        VDA5050_ERROR(
          "[AGV] Stitch validation rejected order {} (update {}) for {}: "
          "{} error(s)",
          order.order_id, order.order_update_id, agv_id_, stitch.errors.size());
        return;
    }
  }

  // Rebuild the active order from the snapshot for the publisher chain.
  std::optional<vda5050_core::types::Order> active_order;
  if (snap.has_active)
  {
    vda5050_core::types::Order ao;
    ao.order_id = snap.order_id;
    ao.order_update_id = snap.order_update_id;
    ao.nodes = snap.nodes;
    ao.edges = snap.edges;
    active_order = std::move(ao);
  }

  // Capture the loaded graph so it outlives a mid-flight layout swap. Use
  // parent_raw_, not parent_.lock() (see parent_raw_ doc-comment).
  vda5050_core::layout::Graph::ConstPtr loaded_graph;
  if (parent_raw_)
  {
    loaded_graph = parent_raw_->get_loaded_graph();
  }

  vda5050_core::validation::PreSendContext ctx{
    get_connection_status(), get_last_state(), get_last_factsheet(),
    get_operational_state(), std::move(loaded_graph)};

  if (!ctx.last_factsheet.has_value())
  {
    VDA5050_WARN(
      "[AGV] No factsheet cached for {}; traversability capability and "
      "limit checks will be skipped (reachability still runs).",
      agv_id_);
  }

  std::optional<vda5050_core::types::Order> merged;
  auto result = order_publisher_.publish(
    *protocol_adapter_, ctx, order, active_order, &merged);
  if (!result)
  {
    VDA5050_ERROR(
      "[AGV] Order validation failed for {}: {} error(s)", agv_id_,
      result.fatal_errors().size());
    for (const auto& err : result.fatal_errors())
    {
      VDA5050_ERROR(
        "[AGV]   - type={} level={} desc={}", err.error_type,
        err.error_level == vda5050_core::types::ErrorLevel::FATAL ? "FATAL"
                                                                  : "WARNING",
        err.error_description.value_or(""));
    }
    return;
  }

  // Pass the merged order so the tracker adopts it without re-combining.
  order_lifecycle_.record_published(order, merged);
}

void AGV::publish_instant_actions(
  const vda5050_core::types::InstantActions& actions)
{
  if (!protocol_adapter_)
  {
    VDA5050_WARN(
      "[AGV] Cannot publish instant actions: no protocol adapter for {}",
      agv_id_);
    return;
  }

  // Pull master's loaded-graph snapshot for the traversability checks.
  // Use parent_raw_ instead of parent_.lock() to avoid extending master
  // lifetime inside the queue thread — see parent_raw_ doc-comment.
  vda5050_core::layout::Graph::ConstPtr loaded_graph;
  if (parent_raw_)
  {
    loaded_graph = parent_raw_->get_loaded_graph();
  }

  vda5050_core::validation::PreSendContext ctx{
    get_connection_status(), get_last_state(), get_last_factsheet(),
    get_operational_state(), std::move(loaded_graph)};

  if (!ctx.last_factsheet.has_value())
  {
    VDA5050_WARN(
      "[AGV] No factsheet cached for {}; traversability capability check "
      "will be skipped for this InstantActions publish.",
      agv_id_);
  }

  auto result =
    instant_actions_publisher_.publish(*protocol_adapter_, ctx, actions);
  if (!result)
  {
    VDA5050_ERROR(
      "[AGV] Instant actions validation failed for {}: {} error(s)", agv_id_,
      result.fatal_errors().size());
  }
}

// ============================================================================
// Helper Methods
// ============================================================================

std::string AGV::build_topic(const std::string& topic_name) const
{
  return interface_name_ + "/" + Version + "/" + manufacturer_ + "/" +
         serial_number_ + "/" + topic_name;
}

}  // namespace vda5050_core::master
