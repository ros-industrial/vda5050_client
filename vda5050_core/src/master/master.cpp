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

#include "vda5050_core/master/master.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/actions/instant_actions_publisher.hpp"
#include "vda5050_core/master/order/order_stitcher.hpp"
#include "vda5050_core/master/standard_names.hpp"
#include "vda5050_core/master/updates/agv_updates.hpp"
#include "vda5050_core/validation/content_validator.hpp"
#include "vda5050_core/validation/factsheet_alignment.hpp"
#include "vda5050_core/validation/operating_mode_control.hpp"
#include "vda5050_core/validation/pre_send_validator.hpp"

namespace vda5050_core::master {

namespace {

// Fill unset header fields from the routing args; the transport re-stamps
// identity at publish, so this only satisfies pre-send validation.
void stamp_outbound_header(
  vda5050_core::types::Header& header, const std::string& manufacturer,
  const std::string& serial_number)
{
  if (header.version.empty()) header.version = SupportedSchemaVersions.front();
  if (header.manufacturer.empty()) header.manufacturer = manufacturer;
  if (header.serial_number.empty()) header.serial_number = serial_number;
}

// Warn once per process, not per master, so repeated construction stays quiet.
void warn_experimental_once()
{
  static std::once_flag flag;
  std::call_once(flag, [] {
    VDA5050_WARN(
      "VDA5050Master API is experimental; public interfaces may change in "
      "future releases.");
  });
}

}  // namespace

// ============================================================================
// Constructor / Destructor
// ============================================================================

std::shared_ptr<VDA5050Master> VDA5050Master::make(
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client)
{
  warn_experimental_once();
  return std::shared_ptr<VDA5050Master>(
    new VDA5050Master(std::move(mqtt_client)));
}

VDA5050Master::VDA5050Master(
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client)
: mqtt_client_(std::move(mqtt_client))
{
  register_event_dispatch();
  VDA5050_DEBUG("Created VDA5050Master instance");
  // Broker connection-state callbacks are wired in connect(), where
  // weak_from_this() is valid (it is not during construction).
}

VDA5050Master::~VDA5050Master()
{
  VDA5050_DEBUG("Destroying VDA5050Master instance");
  // Clear the connection-state callbacks: the weak_ptr already no-ops in-flight
  // ones, and this stops a post-destruction auto-reconnect from firing.
  if (mqtt_client_)
  {
    mqtt_client_->set_connection_lost_callback(nullptr);
    mqtt_client_->set_connected_callback(nullptr);
  }
  disconnect();

  // Stop AGV worker threads before members destruct, so no queue thread runs
  // against shared state (MQTT client, graph holder) mid-teardown.
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    for (auto& kv : agvs_)
    {
      if (kv.second)
      {
        kv.second->stop();
      }
    }
  }
  VDA5050_DEBUG("VDA5050Master instance destroyed");
}

// ============================================================================
// Connection Management
// ============================================================================

void VDA5050Master::connect()
{
  if (!mqtt_client_)
  {
    VDA5050_WARN("Cannot connect: no MQTT client");
    return;
  }

  if (mqtt_client_->connected())
  {
    VDA5050_WARN("Already connected");
    return;
  }

  // Capture weak_ptr, not `this`: lock() no-ops once the master is gone, and
  // keeps it alive while held (callbacks can race destruction).
  std::weak_ptr<VDA5050Master> weak = weak_from_this();
  mqtt_client_->set_connection_lost_callback([weak](const std::string& cause) {
    if (auto self = weak.lock()) self->handle_broker_connection_lost(cause);
  });
  mqtt_client_->set_connected_callback([weak](const std::string& cause) {
    if (auto self = weak.lock()) self->handle_broker_connected(cause);
  });

  VDA5050_DEBUG("Connecting MQTT client");
  mqtt_client_->connect();
}

void VDA5050Master::disconnect()
{
  if (!mqtt_client_)
  {
    return;
  }

  if (!mqtt_client_->connected())
  {
    return;
  }

  VDA5050_DEBUG("Disconnecting MQTT client");
  mqtt_client_->disconnect();
  {
    std::lock_guard<std::mutex> lock(broker_status_mutex_);
    broker_connected_ = false;
  }
  VDA5050_DEBUG("Disconnected");
}

bool VDA5050Master::is_connected() const
{
  return mqtt_client_ && mqtt_client_->connected();
}

// ============================================================================
// AGV Onboarding/Offboarding
// ============================================================================

std::shared_ptr<AGV> VDA5050Master::create_agv_locked(
  const std::string& interface_name, const std::string& manufacturer,
  const std::string& serial_number, std::size_t max_queue_size,
  bool drop_oldest)
{
  // weak_from_this() back-ref lets the AGV dispatch into our callbacks and
  // detect master destruction. Caller wires subscriptions outside agv_mutex_.
  return std::make_shared<AGV>(
    vda5050_core::execution::ProtocolAdapter::make(
      mqtt_client_, interface_name, Version, manufacturer, serial_number),
    interface_name, manufacturer, serial_number, max_queue_size, drop_oldest,
    StateHeartbeatInterval, weak_from_this(), graph_holder_);
}

void VDA5050Master::onboard_agv(
  const std::string& interface_name, const std::string& manufacturer,
  const std::string& serial_number, size_t max_queue_size, bool drop_oldest)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> new_agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);

    if (get_agv_by_id(agv_id))
    {
      VDA5050_WARN("AGV already onboarded [{}]", agv_id);
      return;
    }

    new_agv = create_agv_locked(
      interface_name, manufacturer, serial_number, max_queue_size, drop_oldest);
    agvs_[agv_id] = new_agv;
  }

  // Clear any baseline a prior offboard may have left so this AGV starts clean.
  master_context_.forget_agv(agv_id);

  // Subscribe outside agv_mutex_: SUBSCRIBE blocks on SUBACK, and a racing
  // inbound on_state -> get_agv() needs the same lock (deadlock otherwise).
  new_agv->setup_subscriptions();

  VDA5050_INFO("Onboarded AGV [{}]", agv_id);
}

VDA5050Master::BatchOnboardResult VDA5050Master::onboard_agv_batch(
  const std::vector<OnboardSpec>& specs)
{
  BatchOnboardResult result;
  std::vector<std::shared_ptr<AGV>> new_agvs;

  {
    std::lock_guard<std::mutex> lock(agv_mutex_);

    for (const auto& spec : specs)
    {
      if (spec.manufacturer.empty() || spec.serial_number.empty())
      {
        result.failed.push_back(spec);
        VDA5050_WARN("Rejected AGV with empty manufacturer or serial");
        continue;
      }

      const std::string agv_id = spec.manufacturer + "/" + spec.serial_number;
      if (agvs_.find(agv_id) != agvs_.end())
      {
        result.skipped_already_onboarded.push_back(spec);
        continue;
      }

      auto agv = create_agv_locked(
        DefaultInterfaceName, spec.manufacturer, spec.serial_number,
        spec.max_queue_size, spec.drop_oldest);
      agvs_[agv_id] = agv;
      new_agvs.push_back(std::move(agv));
      result.onboarded.push_back(spec);
    }
  }

  // Clear any prior-offboard baseline, then wire MQTT subscriptions outside
  // agv_mutex_ — see onboard_agv() for the deadlock this avoids.
  for (auto& agv : new_agvs)
  {
    master_context_.forget_agv(agv->get_agv_id());
    agv->setup_subscriptions();
  }

  VDA5050_INFO(
    "Batch onboard: onboarded={} skipped={} failed={}", result.onboarded.size(),
    result.skipped_already_onboarded.size(), result.failed.size());
  return result;
}

std::size_t VDA5050Master::offboard_agv_batch(
  const std::vector<std::pair<std::string, std::string>>& keys)
{
  // AGV::stop() joins the queue thread; gather under the lock and
  // stop outside, same as offboard_agv().
  std::vector<std::shared_ptr<AGV>> to_stop;
  to_stop.reserve(keys.size());
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    for (const auto& key : keys)
    {
      if (key.first.empty() || key.second.empty()) continue;
      const std::string agv_id = key.first + "/" + key.second;
      auto it = agvs_.find(agv_id);
      if (it == agvs_.end()) continue;
      if (it->second) to_stop.push_back(std::move(it->second));
      agvs_.erase(it);
    }
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (const auto& key : keys)
    {
      if (key.first.empty() || key.second.empty()) continue;
      alignment_cache_.erase(key.first + "/" + key.second);
    }
  }

  // Stop and destroy first (the dtor unsubscribes), THEN drop the baselines so
  // an in-flight message can't re-seed them after forget.
  for (auto& agv : to_stop)
  {
    if (agv) agv->stop();
  }
  const std::size_t offboarded = to_stop.size();
  to_stop.clear();

  for (const auto& key : keys)
  {
    if (key.first.empty() || key.second.empty()) continue;
    master_context_.forget_agv(key.first + "/" + key.second);
  }

  VDA5050_INFO("Batch offboard: offboarded={}", offboarded);
  return offboarded;
}

std::vector<std::pair<std::string, std::string>>
VDA5050Master::get_onboarded_agvs() const
{
  std::lock_guard<std::mutex> lock(agv_mutex_);
  std::vector<std::pair<std::string, std::string>> out;
  out.reserve(agvs_.size());
  for (const auto& kv : agvs_)
  {
    const std::string& agv_id = kv.first;
    const auto slash = agv_id.find('/');
    if (slash == std::string::npos)
    {
      out.emplace_back(agv_id, std::string{});
    }
    else
    {
      out.emplace_back(agv_id.substr(0, slash), agv_id.substr(slash + 1));
    }
  }
  return out;
}

void VDA5050Master::onboard_agv(
  const std::string& manufacturer, const std::string& serial_number,
  size_t max_queue_size, bool drop_oldest)
{
  onboard_agv(
    DefaultInterfaceName, manufacturer, serial_number, max_queue_size,
    drop_oldest);
}

//=============================================================================
void VDA5050Master::offboard_agv(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    auto it = agvs_.find(agv_id);
    if (it == agvs_.end())
    {
      VDA5050_WARN("Cannot offboard: AGV not found [{}]", agv_id);
      return;
    }
    agv = std::move(it->second);
    agvs_.erase(it);
  }

  // Stop, then destroy so the dtor unsubscribes before we drop the baselines —
  // otherwise an in-flight State/Connection could re-seed them after forget.
  agv->stop();
  agv.reset();
  master_context_.forget_agv(agv_id);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    alignment_cache_.erase(agv_id);
  }

  VDA5050_INFO("Offboarded AGV [{}]", agv_id);
}

bool VDA5050Master::is_agv_onboarded(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(agv_mutex_);
  std::string agv_id = manufacturer + "/" + serial_number;
  return get_agv_by_id(agv_id) != nullptr;
}

// ============================================================================
// AGV Access
// ============================================================================

std::shared_ptr<const AGV> VDA5050Master::get_agv(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(agv_mutex_);
  std::string agv_id = manufacturer + "/" + serial_number;
  return get_agv_by_id(agv_id);
}

void VDA5050Master::cancel_pending_orders(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(manufacturer + "/" + serial_number);
  }
  if (agv) agv->cancel_pending_orders();
}

std::pair<std::size_t, std::size_t> VDA5050Master::resume_mode_cancelled_queue(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(manufacturer + "/" + serial_number);
  }
  return agv ? agv->resume_mode_cancelled_queue()
             : std::pair<std::size_t, std::size_t>{0, 0};
}

std::pair<std::size_t, std::size_t> VDA5050Master::discard_mode_cancelled_queue(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(manufacturer + "/" + serial_number);
  }
  return agv ? agv->discard_mode_cancelled_queue()
             : std::pair<std::size_t, std::size_t>{0, 0};
}

std::shared_ptr<AGV> VDA5050Master::get_agv_by_id(
  const std::string& agv_id) const
{
  // Note: Caller must hold agv_mutex_
  auto it = agvs_.find(agv_id);
  return (it != agvs_.end()) ? it->second : nullptr;
}

// ============================================================================
// Fleet event dispatch
// ============================================================================

void VDA5050Master::ingest_state(
  const std::string& agv_id, const vda5050_core::types::State& state)
{
  master_context_.on_state(agv_id, state);
}

void VDA5050Master::ingest_connection(
  const std::string& agv_id, const vda5050_core::types::Connection& connection)
{
  master_context_.on_connection(agv_id, connection);
}

void VDA5050Master::fire_hook(
  const std::string& agv_id, const char* hook_name,
  const std::function<void()>& fn)
{
  try
  {
    fn();
  }
  catch (const std::exception& e)
  {
    VDA5050_ERROR("{} hook threw for [{}]: {}", hook_name, agv_id, e.what());
  }
  catch (...)
  {
    VDA5050_ERROR(
      "{} hook threw a non-std exception for [{}]", hook_name, agv_id);
  }
}

void VDA5050Master::register_event_dispatch()
{
  auto provider = master_context_.provider();

  provider->on<NodeReachedUpdate>([this](std::shared_ptr<NodeReachedUpdate> u) {
    fire_hook(u->agv_id, "on_node_reached", [&] {
      if (on_node_reached_cb_) on_node_reached_cb_(u->agv_id, u->node_id);
    });
  });

  provider->on<ErrorsChangedUpdate>(
    [this](std::shared_ptr<ErrorsChangedUpdate> u) {
      fire_hook(u->agv_id, "on_errors", [&] {
        if (on_errors_appeared_cb_ && !u->appeared.empty())
          on_errors_appeared_cb_(u->agv_id, u->appeared);
        if (on_errors_resolved_cb_ && !u->resolved.empty())
          on_errors_resolved_cb_(u->agv_id, u->resolved);
      });
    });

  provider->on<NewBaseRequestUpdate>(
    [this](std::shared_ptr<NewBaseRequestUpdate> u) {
      fire_hook(u->agv_id, "on_new_base_requested", [&] {
        if (on_new_base_requested_cb_) on_new_base_requested_cb_(u->agv_id);
      });
    });

  provider->on<OperatingModeChangedUpdate>(
    [this](std::shared_ptr<OperatingModeChangedUpdate> u) {
      fire_hook(u->agv_id, "on_mode_changed", [&] {
        if (on_mode_changed_cb_)
          on_mode_changed_cb_(u->agv_id, u->mode, u->prev_mode);
      });
    });

  provider->on<PausedChangedUpdate>(
    [this](std::shared_ptr<PausedChangedUpdate> u) {
      fire_hook(u->agv_id, "on_paused", [&] {
        if (on_paused_cb_) on_paused_cb_(u->agv_id, u->paused);
      });
    });

  provider->on<DrivingChangedUpdate>(
    [this](std::shared_ptr<DrivingChangedUpdate> u) {
      fire_hook(u->agv_id, "on_driving", [&] {
        if (on_driving_cb_) on_driving_cb_(u->agv_id, u->driving);
      });
    });

  provider->on<LoadsChangedUpdate>(
    [this](std::shared_ptr<LoadsChangedUpdate> u) {
      fire_hook(u->agv_id, "on_loads_changed", [&] {
        if (on_loads_changed_cb_) on_loads_changed_cb_(u->agv_id, u->loads);
      });
    });

  provider->on<ConnectionChangedUpdate>(
    [this](std::shared_ptr<ConnectionChangedUpdate> u) {
      fire_hook(u->agv_id, "on_connection", [&] {
        switch (u->kind)
        {
          case ConnectionTransition::CONNECTED:
            if (on_connect_cb_) on_connect_cb_(u->agv_id);
            break;
          case ConnectionTransition::OFFLINE:
            if (on_offline_cb_) on_offline_cb_(u->agv_id);
            break;
          case ConnectionTransition::CONNECTIONBROKEN:
            if (on_connection_broken_cb_) on_connection_broken_cb_(u->agv_id);
            break;
          case ConnectionTransition::NONE:
            break;
        }
      });
    });
}

// ============================================================================
// Outgoing Messages
// ============================================================================

bool VDA5050Master::publish_order(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::Order& order)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(agv_id);
  }

  if (!agv)
  {
    VDA5050_WARN("Cannot publish order: AGV not onboarded [{}]", agv_id);
    return false;
  }

  return agv->send_order(order);
}

OrderAssignmentResult VDA5050Master::assign_order(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::Order& order_in)
{
  vda5050_core::types::Order order = order_in;
  stamp_outbound_header(order.header, manufacturer, serial_number);

  OrderAssignmentResult res;
  auto add_error = [&](const std::string& description) {
    res.errors.push_back(vda5050_core::errors::create_error(
      vda5050_core::errors::PreSendValidationError, description, {}));
  };

  const std::string agv_id = manufacturer + "/" + serial_number;
  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(agv_id);
  }
  if (!agv)
  {
    res.decision = OrderAssignmentDecision::AGV_NOT_ONBOARDED;
    add_error("AGV not onboarded: " + agv_id);
    return res;
  }

  if (
    agv->get_connection_status() !=
    vda5050_core::types::ConnectionState::ONLINE)
  {
    res.decision = OrderAssignmentDecision::AGV_OFFLINE;
    add_error("AGV connection_status is not ONLINE");
    return res;
  }

  if (agv->get_operational_state() != AGVState::AVAILABLE)
  {
    res.decision = OrderAssignmentDecision::AGV_NOT_READY;
    add_error("AGV operational_state is not AVAILABLE");
    return res;
  }

  auto last_state = agv->get_last_state();
  if (!last_state.has_value())
  {
    res.decision = OrderAssignmentDecision::AGV_NO_STATE_YET;
    add_error("AGV has not yet reported any State");
    return res;
  }
  if (!vda5050_core::validation::is_master_in_control(
        last_state->operating_mode))
  {
    res.decision = OrderAssignmentDecision::AGV_MODE_NOT_AUTO;
    add_error("AGV operating_mode is not AUTOMATIC or SEMIAUTOMATIC");
    return res;
  }
  if (
    !last_state->agv_position.has_value() ||
    !last_state->agv_position->position_initialized)
  {
    res.decision = OrderAssignmentDecision::AGV_POSITION_NOT_INITIALIZED;
    add_error("AGV position is not initialized");
    return res;
  }

  // Caller-feedback pre-flight only; the queue thread re-runs the stitcher and
  // solely owns the pending enqueue.
  const auto snap = agv->active_order_snapshot();
  bool stitch_will_queue = false;
  if (snap.has_active)
  {
    OrderStitcher stitcher;
    auto stitch = stitcher.decide(order, snap);
    if (stitch.decision == StitchDecision::REJECT)
    {
      res.decision = OrderAssignmentDecision::STITCH_REJECTED;
      res.errors = std::move(stitch.errors);
      return res;
    }
    if (stitch.decision == StitchDecision::IGNORE)
    {
      // Duplicate already applied — don't re-publish.
      res.decision = OrderAssignmentDecision::DUPLICATE_IGNORED;
      return res;
    }
    stitch_will_queue = (stitch.decision == StitchDecision::QUEUE_PENDING);
  }

  // send_order returns false only when the outbound queue is full.
  if (!agv->send_order(order))
  {
    res.decision = OrderAssignmentDecision::AGV_QUEUE_FULL;
    add_error("AGV outbound queue full or unable to accept order");
    return res;
  }

  res.decision = stitch_will_queue ? OrderAssignmentDecision::STITCH_QUEUED
                                   : OrderAssignmentDecision::ASSIGNED;
  return res;
}

bool VDA5050Master::publish_instant_actions(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::InstantActions& actions)
{
  std::string agv_id = manufacturer + "/" + serial_number;

  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(agv_id);
  }

  if (!agv)
  {
    VDA5050_WARN(
      "Cannot publish instant actions: AGV not onboarded [{}]", agv_id);
    return false;
  }

  // Enforce action_id uniqueness even on the thin path (readiness pre-flight
  // is still skipped) — a duplicate id is undefined on the AGV.
  if (
    auto conflict =
      first_instant_action_id_conflict(agv, agv->get_last_state(), actions))
  {
    VDA5050_WARN(
      "Not publishing instant actions for [{}]: {}", agv_id, *conflict);
    return false;
  }

  return agv->send_instant_actions(actions);
}

std::optional<std::string> VDA5050Master::first_instant_action_id_conflict(
  const std::shared_ptr<AGV>& agv,
  const std::optional<vda5050_core::types::State>& last_state,
  const vda5050_core::types::InstantActions& actions) const
{
  std::set<std::string> in_flight;
  if (last_state.has_value())
  {
    for (const auto& as : last_state->action_states)
    {
      in_flight.insert(as.action_id);
    }
  }
  const auto snap = agv->active_order_snapshot();
  if (snap.has_active)
  {
    for (const auto& n : snap.nodes)
      for (const auto& a : n.actions) in_flight.insert(a.action_id);
    for (const auto& e : snap.edges)
      for (const auto& a : e.actions) in_flight.insert(a.action_id);
  }
  for (const auto& id : agv->get_queued_instant_action_ids())
  {
    in_flight.insert(id);
  }

  std::set<std::string> seen_in_batch;
  for (const auto& a : actions.actions)
  {
    if (a.action_id.empty())
    {
      return "action with empty action_id is not allowed";
    }
    if (in_flight.count(a.action_id))
    {
      return "action_id '" + a.action_id +
             "' collides with an in-flight, active-order, or queued action";
    }
    if (!seen_in_batch.insert(a.action_id).second)
    {
      return "action_id '" + a.action_id + "' is duplicated within the batch";
    }
  }
  return std::nullopt;
}

InstantActionAssignmentResult VDA5050Master::assign_instant_actions(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::InstantActions& actions_in)
{
  vda5050_core::types::InstantActions actions = actions_in;
  stamp_outbound_header(actions.header, manufacturer, serial_number);

  InstantActionAssignmentResult res;
  auto add_error = [&](const std::string& description) {
    res.errors.push_back(vda5050_core::errors::create_error(
      vda5050_core::errors::PreSendValidationError, description, {}));
  };

  const std::string agv_id = manufacturer + "/" + serial_number;
  std::shared_ptr<AGV> agv;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agv = get_agv_by_id(agv_id);
  }
  if (!agv)
  {
    res.decision = InstantActionDecision::AGV_NOT_ONBOARDED;
    add_error("AGV not onboarded: " + agv_id);
    return res;
  }

  if (
    agv->get_connection_status() !=
    vda5050_core::types::ConnectionState::ONLINE)
  {
    res.decision = InstantActionDecision::AGV_OFFLINE;
    add_error("AGV connection_status is not ONLINE");
    return res;
  }

  if (actions.actions.empty())
  {
    res.decision = InstantActionDecision::INVALID_CONTENT;
    add_error("InstantActions batch is empty");
    return res;
  }

  // One coherent readiness snapshot feeds every check below.
  vda5050_core::validation::PreSendContext ia_ctx{
    agv->get_connection_status(), agv->get_last_state(),
    agv->get_last_factsheet(), agv->get_operational_state(),
    get_loaded_graph()};

  auto schema_res =
    vda5050_core::validation::validate_instant_actions_content(actions);
  if (!schema_res)
  {
    res.errors = schema_res.fatal_errors();
    res.decision = InstantActionDecision::INVALID_CONTENT;
    return res;
  }

  // Global action_id uniqueness (spec-mandated; collision is vendor-undefined).
  if (
    auto conflict =
      first_instant_action_id_conflict(agv, ia_ctx.last_state, actions))
  {
    res.decision = InstantActionDecision::DUPLICATE_ACTION_ID;
    add_error(*conflict);
    return res;
  }

  auto gate = InstantActionsPublisher::validate_gate(ia_ctx, actions);
  if (gate.failed != ActionGateStep::NONE)
  {
    res.errors = gate.result.fatal_errors();
    switch (gate.failed)
    {
      case ActionGateStep::MODE:
        res.decision = InstantActionDecision::AGV_MODE_NOT_AUTO_FOR_ACTION;
        break;
      case ActionGateStep::CAPABILITY:
        res.decision = InstantActionDecision::AGV_CANNOT_PERFORM_ACTION;
        break;
      case ActionGateStep::LIMITS:
        res.decision = InstantActionDecision::EXCEEDS_PROTOCOL_LIMITS;
        break;
      case ActionGateStep::CONFLICT:
      {
        // A mixed batch can carry both; report HARD if any error is HARD.
        const bool any_hard =
          std::any_of(res.errors.begin(), res.errors.end(), [](const auto& e) {
            return e.error_type == vda5050_core::errors::HardActionBlockedError;
          });
        res.decision = any_hard
                         ? InstantActionDecision::HARD_ACTION_BLOCKED
                         : InstantActionDecision::ACTION_BLOCKED_BY_DRIVING;
        break;
      }
      case ActionGateStep::NONE:
        break;
    }
    return res;
  }

  if (!agv->send_instant_actions(actions))
  {
    res.decision = InstantActionDecision::AGV_QUEUE_FULL;
    add_error("AGV outbound queue full or unable to accept instant actions");
    return res;
  }
  res.decision = InstantActionDecision::ASSIGNED;
  return res;
}

// ============================================================================
// Reaction callbacks — registration setters
// ============================================================================

void VDA5050Master::on_state(
  std::function<void(const std::string&, const vda5050_core::types::State&)>
    callback)
{
  on_state_cb_ = std::move(callback);
}

void VDA5050Master::on_connection(
  std::function<
    void(const std::string&, const vda5050_core::types::Connection&)>
    callback)
{
  on_connection_cb_ = std::move(callback);
}

void VDA5050Master::on_factsheet(
  std::function<void(const std::string&, const vda5050_core::types::Factsheet&)>
    callback)
{
  on_factsheet_cb_ = std::move(callback);
}

// ============================================================================
// Topology layout
// ============================================================================

vda5050_core::layout::LayoutLoadResult VDA5050Master::load_layout_from_config(
  const std::string& path)
{
  auto result = vda5050_core::layout::load_from_file(path);
  if (!result)
  {
    VDA5050_ERROR(
      "Layout config load failed for '{}': {} error(s)", path,
      result.errors.size());
    return result;
  }

  auto graph = vda5050_core::layout::Graph::from_lif(*result.lif);

  VDA5050_INFO(
    "Loaded layout '{}' v{} from '{}' ({} nodes, {} edges)",
    result.lif->meta_information.project_identification,
    result.lif->meta_information.lif_version, path, graph->node_count(),
    graph->edge_count());

  set_graph(std::move(graph));
  return result;
}

void VDA5050Master::set_graph(vda5050_core::layout::Graph::ConstPtr graph)
{
  // Snapshot AGVs under agv_mutex_ (lock order: agv then map) so alignment can
  // be computed off-lock below.
  std::vector<std::pair<std::string, std::shared_ptr<AGV>>> agvs_snapshot;
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    agvs_snapshot.reserve(agvs_.size());
    for (const auto& kv : agvs_)
      agvs_snapshot.emplace_back(kv.first, kv.second);
  }

  // Compute alignment for each AGV against the new graph outside any
  // master lock — uses each AGV's own data_mutex_ via accessors.
  std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
    new_cache;
  for (const auto& kv : agvs_snapshot)
  {
    const auto& fs_opt = kv.second->get_last_factsheet();
    if (!fs_opt.has_value()) continue;
    auto alignment = vda5050_core::validation::check_factsheet_alignment(
      *graph, fs_opt.value());
    new_cache.emplace(kv.first, std::move(alignment));
  }

  // Install the graph (shared holder) + replace the alignment cache atomically.
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    graph_holder_->set(std::move(graph));
    alignment_cache_ = std::move(new_cache);
  }
}

vda5050_core::layout::Graph::ConstPtr VDA5050Master::get_loaded_graph() const
{
  return graph_holder_->get();
}

std::unordered_map<std::string, vda5050_core::errors::ValidationResult>
VDA5050Master::get_alignment_cache_snapshot() const
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  return alignment_cache_;
}

void VDA5050Master::refresh_alignment_for_agv(
  const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet)
{
  vda5050_core::layout::Graph::ConstPtr snap = graph_holder_->get();
  if (snap == nullptr) return;  // no layout loaded yet — nothing to align

  auto alignment =
    vda5050_core::validation::check_factsheet_alignment(*snap, factsheet);
  if (alignment.has_warnings())
  {
    VDA5050_WARN(
      "Factsheet alignment mismatch for AGV [{}] against layout "
      "'{}': {} finding(s)",
      agv_id, snap->lif().meta_information.project_identification,
      alignment.warnings().size());
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  // Drop the write if the graph was swapped while we computed off-lock — that
  // swap already recomputed this AGV against the new graph.
  if (graph_holder_->get() == snap)
  {
    alignment_cache_[agv_id] = std::move(alignment);
  }
}

void VDA5050Master::on_visualization(
  std::function<
    void(const std::string&, const vda5050_core::types::Visualization&)>
    callback)
{
  on_visualization_cb_ = std::move(callback);
}

// ============================================================================
// Event triggers — registration setters
// ============================================================================

void VDA5050Master::on_node_reached(
  std::function<void(const std::string&, const std::string&)> callback)
{
  on_node_reached_cb_ = std::move(callback);
}

void VDA5050Master::on_order_complete(
  std::function<void(const std::string&, const std::string&)> callback)
{
  on_order_complete_cb_ = std::move(callback);
}

void VDA5050Master::on_errors_appeared(
  std::function<
    void(const std::string&, const std::vector<vda5050_core::types::Error>&)>
    callback)
{
  on_errors_appeared_cb_ = std::move(callback);
}

void VDA5050Master::on_errors_resolved(
  std::function<
    void(const std::string&, const std::vector<vda5050_core::types::Error>&)>
    callback)
{
  on_errors_resolved_cb_ = std::move(callback);
}

void VDA5050Master::on_new_base_requested(
  std::function<void(const std::string&)> callback)
{
  on_new_base_requested_cb_ = std::move(callback);
}

void VDA5050Master::on_mode_changed(
  std::function<void(
    const std::string&, vda5050_core::types::OperatingMode,
    vda5050_core::types::OperatingMode)>
    callback)
{
  on_mode_changed_cb_ = std::move(callback);
}

void VDA5050Master::on_paused(
  std::function<void(const std::string&, bool)> callback)
{
  on_paused_cb_ = std::move(callback);
}

void VDA5050Master::on_driving(
  std::function<void(const std::string&, bool)> callback)
{
  on_driving_cb_ = std::move(callback);
}

void VDA5050Master::on_loads_changed(
  std::function<
    void(const std::string&, const std::vector<vda5050_core::types::Load>&)>
    callback)
{
  on_loads_changed_cb_ = std::move(callback);
}

void VDA5050Master::on_connect(std::function<void(const std::string&)> callback)
{
  on_connect_cb_ = std::move(callback);
}

void VDA5050Master::on_offline(std::function<void(const std::string&)> callback)
{
  on_offline_cb_ = std::move(callback);
}

void VDA5050Master::on_connection_broken(
  std::function<void(const std::string&)> callback)
{
  on_connection_broken_cb_ = std::move(callback);
}

void VDA5050Master::on_state_timeout(
  std::function<void(const std::string&)> callback)
{
  on_state_timeout_cb_ = std::move(callback);
}

void VDA5050Master::on_state_resumed(
  std::function<void(const std::string&)> callback)
{
  on_state_resumed_cb_ = std::move(callback);
}

void VDA5050Master::on_broker_disconnected(std::function<void()> callback)
{
  on_broker_disconnected_cb_ = std::move(callback);
}

void VDA5050Master::on_broker_reconnected(std::function<void()> callback)
{
  on_broker_reconnected_cb_ = std::move(callback);
}

// ============================================================================
// AGV-called dispatch — invoke the registered raw-message handler (guarded)
// ============================================================================

void VDA5050Master::dispatch_state(
  const std::string& agv_id, const vda5050_core::types::State& state)
{
  if (on_state_cb_)
    fire_hook(agv_id, "on_state", [&] { on_state_cb_(agv_id, state); });
}

void VDA5050Master::dispatch_connection(
  const std::string& agv_id, const vda5050_core::types::Connection& connection)
{
  if (on_connection_cb_)
    fire_hook(
      agv_id, "on_connection", [&] { on_connection_cb_(agv_id, connection); });
}

void VDA5050Master::dispatch_factsheet(
  const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet)
{
  if (on_factsheet_cb_)
    fire_hook(
      agv_id, "on_factsheet", [&] { on_factsheet_cb_(agv_id, factsheet); });
}

void VDA5050Master::dispatch_visualization(
  const std::string& agv_id,
  const vda5050_core::types::Visualization& visualization)
{
  if (on_visualization_cb_)
  {
    fire_hook(agv_id, "on_visualization", [&] {
      on_visualization_cb_(agv_id, visualization);
    });
  }
}

void VDA5050Master::dispatch_state_timeout(const std::string& agv_id)
{
  if (on_state_timeout_cb_)
    fire_hook(
      agv_id, "on_state_timeout", [&] { on_state_timeout_cb_(agv_id); });
}

void VDA5050Master::dispatch_state_resumed(const std::string& agv_id)
{
  if (on_state_resumed_cb_)
    fire_hook(
      agv_id, "on_state_resumed", [&] { on_state_resumed_cb_(agv_id); });
}

void VDA5050Master::dispatch_order_complete(
  const std::string& agv_id, const std::string& order_id)
{
  if (on_order_complete_cb_)
  {
    fire_hook(agv_id, "on_order_complete", [&] {
      on_order_complete_cb_(agv_id, order_id);
    });
  }
}

// ============================================================================
// Master-broker connection state
// ============================================================================

VDA5050Master::BrokerStatusSnapshot VDA5050Master::get_broker_status() const
{
  std::lock_guard<std::mutex> lock(broker_status_mutex_);
  BrokerStatusSnapshot snap;
  snap.connected = broker_connected_;
  snap.last_disconnect_at = broker_last_disconnect_at_;
  snap.reconnect_count = broker_reconnect_count_;
  return snap;
}

void VDA5050Master::handle_broker_connection_lost(const std::string& cause)
{
  {
    std::lock_guard<std::mutex> lock(broker_status_mutex_);
    broker_connected_ = false;
    broker_last_disconnect_at_ = std::chrono::system_clock::now();
  }
  VDA5050_WARN(
    "Broker connection lost: {}",
    cause.empty() ? "(no cause reported)" : cause.c_str());
  // Guard the callback so a throw can't unwind onto the transport thread.
  try
  {
    if (on_broker_disconnected_cb_) on_broker_disconnected_cb_();
  }
  catch (const std::exception& e)
  {
    VDA5050_ERROR("on_broker_disconnected threw: {}", e.what());
  }
  catch (...)
  {
    VDA5050_ERROR("on_broker_disconnected threw a non-std exception");
  }
}

void VDA5050Master::handle_broker_connected(const std::string& /*cause*/)
{
  std::uint64_t count = 0;
  {
    std::lock_guard<std::mutex> lock(broker_status_mutex_);
    broker_connected_ = true;
    broker_reconnect_count_ += 1;
    count = broker_reconnect_count_;
  }
  // The transport logs the initial connect; only a reconnect adds information.
  if (count > 1)
  {
    VDA5050_INFO("Broker reconnected (count={})", count);
  }
  else
  {
    VDA5050_DEBUG("Broker connection established");
  }
  // Guard the callback so a throw can't unwind onto the transport thread.
  try
  {
    if (on_broker_reconnected_cb_) on_broker_reconnected_cb_();
  }
  catch (const std::exception& e)
  {
    VDA5050_ERROR("on_broker_reconnected threw: {}", e.what());
  }
  catch (...)
  {
    VDA5050_ERROR("on_broker_reconnected threw a non-std exception");
  }
}

}  // namespace vda5050_core::master
