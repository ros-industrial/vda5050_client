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

#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_core/master/heartbeat.hpp"
#include "vda5050_core/master/standard_names.hpp"

using vda5050_core::master::ConnectionHeartbeatInterval;
using vda5050_core::master::HeartbeatListener;
using vda5050_core::master::HeartbeatState;

class MockHeartbeatListener : public HeartbeatListener
{
public:
  MockHeartbeatListener(
    const std::string& id, const int heartbeat_interval,
    std::function<void()> disconnection_callback, int time_to_skip = 0)
  : HeartbeatListener(id, heartbeat_interval, disconnection_callback),
    time_to_skip_(time_to_skip)
  {
    start_connection_heartbeat();
  }

  std::chrono::steady_clock::time_point get_current_time() override
  {
    VDA5050_INFO(
      "get_current_time with skip of " + std::to_string(time_to_skip_));
    return std::chrono::steady_clock::now() +
           std::chrono::seconds(time_to_skip_);
  }

  // Override check interval to speed up tests
  // Instead of waiting 15 seconds, wait only 1 second
  int get_check_interval() override
  {
    return 1;  // Check every 1 second in tests (15x faster than production)
  }

  ~MockHeartbeatListener()
  {
    // Stop the worker thread BEFORE the base destructor runs. listen()
    // calls virtual methods (get_current_time, get_check_interval)
    // through `this`; if the worker is still spinning when this mock's
    // vtable is swapped out for the base, those calls race against the
    // vptr transition. TSan flags this as "data race on vptr".
    stop_connection_heartbeat();
    VDA5050_INFO("MockHeartbeatListener destroyed");
  }

  void trigger_timeout()
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    message_received_.notify_all();
  }

  int time_to_skip_;
};

TEST(HeartbeatListenerTest, HeartbeatListenerInit)
{
  auto hb_listener = MockHeartbeatListener(
    "test_listener", ConnectionHeartbeatInterval,
    [&]() {
      // Timeout callback
      VDA5050_INFO("Timeout callback");
    },
    ConnectionHeartbeatInterval - 1);

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);
  ASSERT_NO_THROW(hb_listener.stop_connection_heartbeat());
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, HeartbeatReceivedNoTimeout)
{
  auto hb_listener = MockHeartbeatListener(
    "test_listener", ConnectionHeartbeatInterval,
    [&]() {
      // Timeout callback
      VDA5050_INFO("Timeout callback");
    },
    ConnectionHeartbeatInterval - 1);

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);
  // std::this_thread::sleep_for(std::chrono::seconds(1));
  hb_listener.received_connection();

  ASSERT_NEAR(
    std::chrono::duration_cast<std::chrono::seconds>(
      hb_listener.get_last_connection_report().time_since_epoch())
      .count(),
    std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count(),
    ConnectionHeartbeatInterval - 1);

  ASSERT_NO_THROW(hb_listener.stop_connection_heartbeat());
}

TEST(HeartbeatListenerTest, HeartbeatNotReceivedTimeout)
{
  std::atomic_bool heartbeat_failed{false};
  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", ConnectionHeartbeatInterval,
    [&heartbeat_failed]() {
      // Timeout callback
      VDA5050_INFO("Timeout callback");
      VDA5050_INFO(
        "Heartbeat_failed before store: " +
        std::to_string(heartbeat_failed.load()));
      heartbeat_failed.store(true);
      VDA5050_INFO(
        "Heartbeat_failed after store: " +
        std::to_string(heartbeat_failed.load()));
      // ASSERT_TRUE(heartbeat_failed->load());
    },
    ConnectionHeartbeatInterval + 1);
  hb_listener->trigger_timeout();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_NO_THROW(hb_listener.reset());
  ASSERT_TRUE(heartbeat_failed.load());
}

TEST(HeartbeatListenerTest, HeartbeatReceivedTimeout)
{
  std::atomic_bool heartbeat_failed{false};
  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", ConnectionHeartbeatInterval,
    [&heartbeat_failed]() {
      // Timeout callback
      VDA5050_INFO("Timeout callback");
      heartbeat_failed.store(true);
    },
    ConnectionHeartbeatInterval + 1);

  hb_listener->trigger_timeout();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  hb_listener->received_connection();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_NO_THROW(hb_listener.reset());
  ASSERT_TRUE(heartbeat_failed.load());
}

TEST(HeartbeatListenerTest, GracefulShutdownDoesNotBlock)
{
  std::atomic_bool callback_called{false};

  // Use a short interval but time_to_skip that won't cause immediate timeout
  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", ConnectionHeartbeatInterval,
    [&callback_called]() { callback_called.store(true); },
    0);  // time_to_skip = 0, so no immediate timeout

  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::RUNNING);

  // This should complete quickly, not block forever
  auto start = std::chrono::steady_clock::now();
  hb_listener->stop_connection_heartbeat();
  auto elapsed = std::chrono::steady_clock::now() - start;

  // Should complete within 2 seconds (1 second check interval + margin)
  ASSERT_LT(
    std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 2)
    << "stop_connection_heartbeat() blocked too long";

  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);
  ASSERT_FALSE(callback_called.load())
    << "Callback should NOT be called during graceful shutdown";
}

TEST(HeartbeatListenerTest, StateIsRunningWhileCallbackExecutes)
{
  std::atomic_bool callback_started{false};
  std::atomic_bool callback_finished{false};
  std::atomic_bool was_running_during_callback{false};

  // We need a raw pointer to check get_state() from within callback
  // Must be atomic because the callback thread may read it while main thread writes
  std::atomic<MockHeartbeatListener*> listener_ptr{nullptr};

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", ConnectionHeartbeatInterval,
    [&callback_started, &callback_finished, &was_running_during_callback,
     &listener_ptr]() {
      callback_started.store(true);

      // Check get_state() during callback execution
      auto* ptr = listener_ptr.load();
      if (ptr)
      {
        was_running_during_callback.store(
          ptr->get_state() == HeartbeatState::RUNNING);
      }

      // Simulate work
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      callback_finished.store(true);
    },
    ConnectionHeartbeatInterval + 1);  // Causes immediate timeout

  listener_ptr = hb_listener.get();

  // Trigger the timeout
  hb_listener->trigger_timeout();

  // Wait for callback to finish
  while (!callback_finished.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Now stop - should be quick since callback already finished
  hb_listener->stop_connection_heartbeat();

  ASSERT_TRUE(callback_started.load()) << "Callback should have started";
  ASSERT_TRUE(callback_finished.load()) << "Callback should have finished";
  ASSERT_TRUE(was_running_during_callback.load())
    << "get_state() should return RUNNING while callback is executing";
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED)
    << "get_state() should return STOPPED after stop completes";
}

TEST(HeartbeatListenerTest, StateIsStoppedOnlyAfterFullStop)
{
  std::atomic_bool stop_initiated{false};
  std::atomic_bool stop_completed{false};
  std::atomic<HeartbeatState> state_during_stop{HeartbeatState::RUNNING};

  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", ConnectionHeartbeatInterval, []() { /* No-op callback */ },
    0);  // No immediate timeout

  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::RUNNING);

  // Start stop in a separate thread so we can observe state
  std::thread stop_thread([&]() {
    stop_initiated.store(true);
    hb_listener->stop_connection_heartbeat();
    stop_completed.store(true);
  });

  // Wait for stop to be initiated
  while (!stop_initiated.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Give time for stop to progress
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!stop_completed.load())
  {
    state_during_stop.store(hb_listener->get_state());
  }

  stop_thread.join();

  // After stop completes, must be STOPPED
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED)
    << "get_state() must be STOPPED after stop_connection_heartbeat() returns";
}

TEST(HeartbeatListenerTest, MultipleStopCallsSafe)
{
  auto hb_listener = std::make_unique<MockHeartbeatListener>(
    "test_listener", ConnectionHeartbeatInterval, []() { /* No-op */ }, 0);

  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::RUNNING);

  ASSERT_NO_THROW(hb_listener->stop_connection_heartbeat());
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);

  ASSERT_NO_THROW(hb_listener->stop_connection_heartbeat());
  ASSERT_EQ(hb_listener->get_state(), HeartbeatState::STOPPED);

  ASSERT_NO_THROW(hb_listener.reset());
}

TEST(HeartbeatListenerTest, MultipleStartCallsDoNotCreateMultipleThreads)
{
  std::atomic<int> callback_count{0};
  HeartbeatListener hb_listener(
    "test_listener", 1,  // 1 second interval
    [&callback_count]() { callback_count.fetch_add(1); });

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);

  // Call start multiple times sequentially
  hb_listener.start_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  hb_listener.start_connection_heartbeat();  // Should be ignored
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  hb_listener.start_connection_heartbeat();  // Should be ignored
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  // Wait for timeout to occur (need > 2 seconds for timeout with 1s interval)
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  // Callback should be called exactly once (only one thread was created)
  ASSERT_EQ(callback_count.load(), 1)
    << "Callback should be called exactly once, not multiple times";

  hb_listener.stop_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, ConcurrentStartCallsDoNotCreateMultipleThreads)
{
  std::atomic<int> callback_count{0};

  HeartbeatListener hb_listener(
    "test_listener", 1,  // 1 second interval
    [&callback_count]() { callback_count.fetch_add(1); });

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);

  // Launch multiple threads trying to start concurrently
  std::vector<std::thread> threads;
  for (int i = 0; i < 10; ++i)
  {
    threads.emplace_back(
      [&hb_listener]() { hb_listener.start_connection_heartbeat(); });
  }

  for (auto& t : threads)
  {
    t.join();
  }

  // Should still be running (only one successful start)
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  // Wait for timeout to occur (need > 2 seconds for timeout with 1s interval)
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  // Callback should be called exactly once (only one thread was created)
  ASSERT_EQ(callback_count.load(), 1)
    << "Callback should be called exactly once even with concurrent starts";

  hb_listener.stop_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}

TEST(HeartbeatListenerTest, ConcurrentStopCallsSafe)
{
  HeartbeatListener hb_listener("test_listener", 1, []() { /* No-op */ });

  hb_listener.start_connection_heartbeat();
  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::RUNNING);

  // Launch multiple threads trying to stop concurrently
  std::vector<std::thread> threads;
  for (int i = 0; i < 10; ++i)
  {
    threads.emplace_back(
      [&hb_listener]() { hb_listener.stop_connection_heartbeat(); });
  }

  for (auto& t : threads)
  {
    t.join();
  }

  ASSERT_EQ(hb_listener.get_state(), HeartbeatState::STOPPED);
}
