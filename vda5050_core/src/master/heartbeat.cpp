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

#include "vda5050_core/master/heartbeat.hpp"

#include <utility>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_core {
namespace master {

//=============================================================================
HeartbeatListener::HeartbeatListener(
  const std::string& id, const int heartbeat_interval,
  std::function<void()> disconnection_callback)
: id_(id),
  heartbeat_interval_(heartbeat_interval),
  state_(HeartbeatState::STOPPED),
  last_connection_report_(std::chrono::steady_clock::now()),
  disconnection_callback_(std::move(disconnection_callback))
{
}

//=============================================================================
HeartbeatListener::~HeartbeatListener()
{
  stop_connection_heartbeat();
  VDA5050_INFO("[" + id_ + "] Deconstructing HeartbeatListener");
}

//=============================================================================
void HeartbeatListener::start_connection_heartbeat()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (state_ != HeartbeatState::STOPPED)
  {
    VDA5050_DEBUG("Connection heartbeat already running or stopping, ignored");
    return;
  }

  // Reset last-report so a long gap before start doesn't fire an immediate
  // timeout. Use now() directly, not the virtual get_current_time() (tests
  // mock that to inject skew; resetting through it would defeat their setup).
  {
    std::lock_guard<std::mutex> ts_lock(last_connection_report_mutex_);
    last_connection_report_ = std::chrono::steady_clock::now();
  }

  VDA5050_INFO("Starting Connection heartbeat listener");
  state_ = HeartbeatState::RUNNING;
  connection_thread_ = std::thread(&HeartbeatListener::listen, this);
}

//=============================================================================
void HeartbeatListener::stop_connection_heartbeat()
{
  std::thread conn_thread_to_join;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_ == HeartbeatState::STOPPED)
    {
      VDA5050_DEBUG("Connection heartbeat already stopped");
      return;
    }

    VDA5050_INFO("Stopping Connection heartbeat listener");
    state_ = HeartbeatState::STOPPING;

    conn_thread_to_join = std::move(connection_thread_);
  }

  message_received_.notify_all();

  if (conn_thread_to_join.joinable())
  {
    conn_thread_to_join.join();
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = HeartbeatState::STOPPED;
  }

  VDA5050_INFO("Stopped Connection heartbeat listener");
}

//=============================================================================
void HeartbeatListener::received_connection()
{
  if (get_state() != HeartbeatState::RUNNING)
  {
    VDA5050_DEBUG("Connection heartbeat not running, ignored...");
    return;
  }
  std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
  last_connection_report_ = get_current_time();
  VDA5050_INFO("[" + id_ + "] Received connection heartbeat");
  message_received_.notify_all();
}

//=============================================================================
std::chrono::steady_clock::time_point
HeartbeatListener::get_last_connection_report()
{
  std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
  return last_connection_report_;
}

//=============================================================================
HeartbeatState HeartbeatListener::get_state()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

//=============================================================================
std::chrono::steady_clock::time_point HeartbeatListener::get_current_time()
{
  return std::chrono::steady_clock::now();
}

//=============================================================================
int HeartbeatListener::get_check_interval()
{
  return heartbeat_interval_;
}

//=============================================================================
bool HeartbeatListener::is_timeout()
{
  std::chrono::steady_clock::time_point current_time = get_current_time();
  int time_since_last_connection_report;
  {
    std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
    time_since_last_connection_report =
      std::chrono::duration_cast<std::chrono::seconds>(
        current_time - last_connection_report_)
        .count();
  }

  if (time_since_last_connection_report > heartbeat_interval_)
  {
    VDA5050_WARN(
      "[" + id_ + "] Connection heartbeat timeout after " +
      std::to_string(time_since_last_connection_report) + " seconds " +
      "(max: " + std::to_string(heartbeat_interval_) + "s)");
    return true;
  }
  return false;
}

//=============================================================================
void HeartbeatListener::listen()
{
  // One callback per timeout episode, reset on recovery; the thread spins
  // until an explicit stop.
  bool timeout_fired = false;

  while (true)
  {
    {
      // Wait on state_mutex_ (which guards state_) so stop_connection_
      // heartbeat()'s state change + notify can't be lost; the timeout also
      // bounds the poll interval. Any non-RUNNING state means stop.
      std::unique_lock<std::mutex> lock(state_mutex_);
      message_received_.wait_for(
        lock, std::chrono::seconds(get_check_interval()),
        [this] { return state_ != HeartbeatState::RUNNING; });

      if (state_ != HeartbeatState::RUNNING)
      {
        VDA5050_DEBUG("[" + id_ + "] Shutdown requested, exiting listen loop");
        return;
      }
    }

    if (is_timeout())
    {
      if (!timeout_fired)
      {
        VDA5050_INFO("[" + id_ + "] Heartbeat timeout fired");
        disconnection_callback_();
        timeout_fired = true;
      }
    }
    else
    {
      timeout_fired = false;
    }
  }
}

}  // namespace master
}  // namespace vda5050_core
