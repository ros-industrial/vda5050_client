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

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_core {
namespace master {

namespace {

// Poll several times per interval so a timeout is detected near the interval,
// not at ~2x it (a poll period equal to the interval misses the boundary).
constexpr int kPollsPerInterval = 4;
// Grace above the interval (interval / kTimeoutGraceDivisor) so a healthy AGV
// reporting right at the boundary isn't falsely flagged.
constexpr int kTimeoutGraceDivisor = 10;

}  // namespace

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
  if (heartbeat_interval_ <= 0)
  {
    throw std::invalid_argument(
      "HeartbeatListener interval must be a positive number of seconds");
  }
}

//=============================================================================
HeartbeatListener::~HeartbeatListener()
{
  stop_connection_heartbeat();
  VDA5050_DEBUG("[{}] Destroying HeartbeatListener", id_);
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

  // Reset the baseline so a gap before start doesn't immediately time out.
  {
    std::lock_guard<std::mutex> ts_lock(last_connection_report_mutex_);
    last_connection_report_ = std::chrono::steady_clock::now();
  }

  VDA5050_DEBUG("Starting Connection heartbeat listener");
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

    VDA5050_DEBUG("Stopping Connection heartbeat listener");
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

  VDA5050_DEBUG("Stopped Connection heartbeat listener");
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
  VDA5050_DEBUG("[{}] Received connection heartbeat", id_);
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
  return std::max(1, heartbeat_interval_ / kPollsPerInterval);
}

//=============================================================================
bool HeartbeatListener::is_timeout()
{
  std::chrono::steady_clock::time_point current_time = get_current_time();
  std::chrono::steady_clock::duration age;
  {
    std::lock_guard<std::mutex> lock(last_connection_report_mutex_);
    age = current_time - last_connection_report_;
  }

  const auto timeout = std::chrono::seconds(
    heartbeat_interval_ + heartbeat_interval_ / kTimeoutGraceDivisor);
  if (age >= timeout)
  {
    const auto age_s =
      std::chrono::duration_cast<std::chrono::seconds>(age).count();
    VDA5050_WARN(
      "[" + id_ + "] Connection heartbeat timeout after " +
      std::to_string(age_s) + " seconds " +
      "(max: " + std::to_string(heartbeat_interval_) + "s)");
    return true;
  }
  return false;
}

//=============================================================================
void HeartbeatListener::listen()
{
  // One callback per timeout episode, reset on recovery.
  bool timeout_fired = false;

  while (true)
  {
    {
      // Wait under state_mutex_ so a concurrent stop's state change + notify
      // isn't lost.
      std::unique_lock<std::mutex> lock(state_mutex_);
      message_received_.wait_for(
        lock, std::chrono::seconds(get_check_interval()),
        [this] { return state_ != HeartbeatState::RUNNING; });

      if (state_ != HeartbeatState::RUNNING)
      {
        VDA5050_DEBUG("[{}] Shutdown requested, exiting listen loop", id_);
        return;
      }
    }

    if (is_timeout())
    {
      if (!timeout_fired)
      {
        VDA5050_DEBUG("[{}] Heartbeat timeout fired", id_);
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
