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

#ifndef VDA5050_CORE__MASTER__HEARTBEAT_HPP_
#define VDA5050_CORE__MASTER__HEARTBEAT_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace vda5050_core {

namespace master {

/// \brief Heartbeat listener lifecycle states
///
/// State transitions:
///   STOPPED -> RUNNING (via start_connection_heartbeat())
///   RUNNING -> STOPPING (via stop_connection_heartbeat())
///   STOPPING -> STOPPED (when cleanup completes)
enum class HeartbeatState
{
  STOPPED,  // Not running, safe to destroy or restart
  RUNNING,  // Actively monitoring heartbeats
  STOPPING  // Stop in progress, cleanup ongoing
};

class HeartbeatListener
{
public:
  /// \brief Construct a heartbeat listener (call start_connection_heartbeat()
  /// to begin monitoring).
  ///
  /// Subclassing contract: a derived class overriding the virtuals MUST call
  /// stop_connection_heartbeat() in its destructor before the base runs — else
  /// the worker thread can call through a torn-down vtable (vptr race).
  HeartbeatListener(
    const std::string& id, const int heartbeat_interval,
    std::function<void()> disconnection_callback);

  /// \brief Stop the listener (joins the worker thread) and destroy it.
  virtual ~HeartbeatListener();

  // Non-copyable, non-movable (due to thread member)
  HeartbeatListener(const HeartbeatListener&) = delete;
  HeartbeatListener& operator=(const HeartbeatListener&) = delete;
  HeartbeatListener(HeartbeatListener&&) = delete;
  HeartbeatListener& operator=(HeartbeatListener&&) = delete;

  /// \brief Start the heartbeat listener thread
  void start_connection_heartbeat();

  /// \brief Stop the heartbeat listener thread
  void stop_connection_heartbeat();

  /// \brief Notify that a heartbeat was received
  void received_connection();

  /// \brief Get the timestamp of the last received heartbeat
  std::chrono::steady_clock::time_point get_last_connection_report();

  /// \brief Get the current heartbeat listener state
  ///
  /// \return HeartbeatState enum value
  HeartbeatState get_state();

  /// \brief Get the current time (virtual for testing)
  virtual std::chrono::steady_clock::time_point get_current_time();

  /// \brief Get the check interval (virtual for testing)
  virtual int get_check_interval();

protected:
  std::condition_variable message_received_;

private:
  bool is_timeout();
  void listen();

  std::string id_;
  std::thread connection_thread_;
  int heartbeat_interval_;

  // Lifecycle state protected by state_mutex_; also the wait mutex for
  // message_received_ so a stop is never missed.
  HeartbeatState state_;
  mutable std::mutex state_mutex_;

  std::chrono::steady_clock::time_point last_connection_report_;
  mutable std::mutex last_connection_report_mutex_;

  std::function<void()> disconnection_callback_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__HEARTBEAT_HPP_
