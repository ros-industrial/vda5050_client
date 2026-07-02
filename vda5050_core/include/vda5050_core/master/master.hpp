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

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "vda5050_core/types/connection.hpp"
#include "vda5050_core/types/factsheet.hpp"
#include "vda5050_core/types/instant_actions.hpp"
#include "vda5050_core/types/order.hpp"
#include "vda5050_core/types/state.hpp"
#include "vda5050_core/types/visualization.hpp"

#include "vda5050_core/transport/mqtt_client_interface.hpp"

#include "vda5050_core/master/agv.hpp"

using vda5050_core::types::Connection;
using vda5050_core::types::Factsheet;
using vda5050_core::types::InstantActions;
using vda5050_core::types::Order;
using vda5050_core::types::State;
using vda5050_core::types::Visualization;

using vda5050_core::transport::MqttClientInterface;

namespace vda5050_core {

namespace master {

/**
 * @brief VDA5050 Master for multi-AGV fleet management
 *
 * This abstract base class manages VDA5050 communication for multiple AGVs
 * using a single shared MQTT client that creates protocol adapters for each AGV.
 *
 * Features:
 * - Single shared MQTT client that creates protocol adapters for each AGV
 * - AGV onboarding/offboarding with allow list
 * - Message routing based on topic parsing
 * - Protocol adapters for subscribing and publishing to AGVs
 *
 * Thread safety note: Callbacks are invoked on the MQTT client thread.
 * If thread safety is required, the callback implementation should handle
 * synchronization (e.g., using a mutex).
 *
 * Construction requirement: VDA5050Master MUST be constructed via
 * `std::make_shared<MyMaster>(...)`. Each onboarded AGV stores a
 * `std::weak_ptr<VDA5050Master>` back-pointer (populated via
 * `weak_from_this()`) and uses it to dispatch incoming messages to the
 * master's virtual callbacks. `weak_from_this()` only returns a valid
 * weak_ptr if the master is currently inside a `shared_ptr` — stack-
 * allocated or `unique_ptr`-managed masters silently no-op on user
 * callbacks.
 */
class VDA5050Master : public std::enable_shared_from_this<VDA5050Master>
{
public:
  /**
   * @brief Construct a VDA5050 master with shared MQTT client and broker address
   * @param mqtt_client Shared MQTT client for subscriptions
   *
   * The mqtt_client is used to create protocol adapters for each onboarded AGV.
   *
   * IMPORTANT: must be constructed via `std::make_shared<MyMaster>(...)` —
   * see class doc-comment.
   *
   * Example usage:
   * @code
   * auto client = vda5050_core::mqtt_client::create_default_client(broker, "master");
   * auto master = std::make_shared<MyMaster>(client);
   * master->connect();
   * @endcode
   */
  explicit VDA5050Master(std::shared_ptr<MqttClientInterface> mqtt_client);

  /**
   * @brief Virtual destructor - disconnects MQTT client
   */
  virtual ~VDA5050Master();

  // Non-copyable, non-movable
  VDA5050Master(const VDA5050Master&) = delete;
  VDA5050Master& operator=(const VDA5050Master&) = delete;
  VDA5050Master(VDA5050Master&&) = delete;
  VDA5050Master& operator=(VDA5050Master&&) = delete;

  // ============================================================================
  // Connection Management
  // ============================================================================

  /**
   * @brief Connect the MQTT client
   */
  void connect();

  /**
   * @brief Disconnect the MQTT client
   */
  void disconnect();

  /**
   * @brief Check if MQTT client is connected
   */
  bool is_connected() const;

  // ============================================================================
  // AGV Onboarding/Offboarding
  // ============================================================================

  /**
   * @brief Onboard an AGV to allow message routing
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @param max_queue_size Maximum number of outgoing messages to queue (default: 10)
   * @param drop_oldest If true, drop oldest message when queue full; if false, reject new message (default: true)
   *
   * Creates an AGV instance in the allowed list. Messages from this AGV
   * will be routed to the appropriate handlers.
   * The interface name is set to "uagv" by default.
   */
  void onboard_agv(
    const std::string& manufacturer, const std::string& serial_number,
    size_t max_queue_size = 10, bool drop_oldest = true);

  /**
   * @brief Onboard an AGV with a custom interface_name to allow message routing
   * @param interface_name Interface name
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @param max_queue_size Maximum number of outgoing messages to queue (default: 10)
   * @param drop_oldest If true, drop oldest message when queue full; if false, reject new message (default: true)
   *
   * Creates an AGV instance in the allowed list. Messages from this AGV
   * will be routed to the appropriate handlers.
   */
  void onboard_agv(
    const std::string& interface_name, const std::string& manufacturer,
    const std::string& serial_number, size_t max_queue_size = 10,
    bool drop_oldest = true);

  /**
   * @brief Offboard an AGV to stop message routing
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   *
   * Removes the AGV from the allowed list. Messages from this AGV
   * will be ignored with a warning.
   */
  void offboard_agv(
    const std::string& manufacturer, const std::string& serial_number);

  /**
   * @brief Check if an AGV is onboarded
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @return true if AGV is onboarded
   */
  bool is_agv_onboarded(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ============================================================================
  // AGV Access
  // ============================================================================

  /**
   * @brief Get a shared pointer to an onboarded AGV
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @return Shared pointer to AGV, or nullptr if not onboarded
   */
  std::shared_ptr<AGV> get_agv(
    const std::string& manufacturer, const std::string& serial_number) const;

  // ============================================================================
  // Outgoing Messages
  // ============================================================================

  /**
   * @brief Publish an order to a specific AGV
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @param order The order message
   * @return true if queued successfully, false if queue is full
   * @throws std::runtime_error if AGV is not onboarded
   */
  bool publish_order(
    const std::string& manufacturer, const std::string& serial_number,
    const Order& order);

  /**
   * @brief Publish instant actions to a specific AGV
   * @param manufacturer Manufacturer name
   * @param serial_number Serial number
   * @param actions The instant actions message
   * @return true if queued successfully, false if queue is full
   * @throws std::runtime_error if AGV is not onboarded
   */
  bool publish_instant_actions(
    const std::string& manufacturer, const std::string& serial_number,
    const InstantActions& actions);

  // ============================================================================
  // User-Extension Callbacks (override in subclass)
  // ============================================================================
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

  /**
   * @brief Called after a State message arrives and is cached on the AGV.
   * @param agv_id  manufacturer/serial composite ID
   * @param state   the parsed State message
   */
  virtual void on_state(const std::string& agv_id, const State& state);

  /**
   * @brief Called after a Connection message arrives and is cached.
   */
  virtual void on_connection(
    const std::string& agv_id, const Connection& connection);

  /**
   * @brief Called after a Factsheet message arrives and is cached.
   */
  virtual void on_factsheet(
    const std::string& agv_id, const Factsheet& factsheet);

  /**
   * @brief Called after a Visualization message arrives and is cached.
   */
  virtual void on_visualization(
    const std::string& agv_id, const Visualization& visualization);

private:
  // ============================================================================
  // Internal AGV lookup
  // ============================================================================

  std::shared_ptr<AGV> get_agv_by_id(const std::string& agv_id) const;

  // ============================================================================
  // Member Variables
  // ============================================================================

  // Shared MQTT client for protocol adapters
  std::shared_ptr<MqttClientInterface> mqtt_client_;

  // Onboarded AGVs (shared_ptr allows safe access)
  mutable std::mutex agv_mutex_;
  std::unordered_map<std::string, std::shared_ptr<AGV>> agvs_;
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__MASTER_HPP_
