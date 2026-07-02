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

#ifndef VDA5050_CORE__MASTER__STANDARD_NAMES_HPP_
#define VDA5050_CORE__MASTER__STANDARD_NAMES_HPP_

#include <string>
#include <vector>

namespace vda5050_core::master {

/// \brief MQTT QoS level — typed replacement for raw int values.
///
/// Cast to int via static_cast<int>(qos) at the MQTT boundary
/// (MqttClientInterface::publish / subscribe expect int).
enum class QosLevel : int
{
  AtMostOnce = 0,   ///< Fire-and-forget; may drop. Used for state, order, etc.
  AtLeastOnce = 1,  ///< Ack'd by broker; may duplicate. Used for connection.
  ExactlyOnce = 2,  ///< Two-phase handshake; rarely needed.
};

const std::string Version = "v2";                          // NOLINT
const std::string DefaultInterfaceName = "uagv";           // NOLINT
const std::string ConnectionTopic = "connection";          // NOLINT
const std::string FactsheetTopic = "factsheet";            // NOLINT
const std::string OrderTopic = "order";                    // NOLINT
const std::string StateTopic = "state";                    // NOLINT
const std::string InstantActionsTopic = "instantActions";  // NOLINT
const std::string VisualizationTopic = "visualization";    // NOLINT

/// \brief Per-topic QoS levels.
///
/// Connection uses AtLeastOnce because the last-will (CONNECTIONBROKEN) must
/// reach master — drops there would silently miss AGV crashes. All other
/// topics use AtMostOnce because there is an application-level feedback loop
/// (state-message progression catches missed orders / instant actions, etc.)
/// so broker-level acks would just add wasted bandwidth.
constexpr QosLevel ConnectionQos = QosLevel::AtLeastOnce;
constexpr QosLevel FactsheetQos = QosLevel::AtMostOnce;
constexpr QosLevel OrderQos = QosLevel::AtMostOnce;
constexpr QosLevel StateQos = QosLevel::AtMostOnce;
constexpr QosLevel VisualizationQos = QosLevel::AtMostOnce;
constexpr QosLevel InstantActionsQos = QosLevel::AtMostOnce;

/// The spec mandates a connection-topic heartbeat every 15s
/// between the AGV client and the broker. The master does NOT poll for
/// this — the spec's window is enforced by the broker's TCP keepalive
/// (mosquitto: configure to <=15s for spec compliance) plus Paho's
/// `set_automatic_reconnect(2, 32)` (vda5050_core/.../paho_mqtt_client.cpp).
/// The AGV's per-transition Connection messages (last-will + on_connect /
/// on_offline / on_connection_broken) are the master-side
/// observable surface. This constant is documentation-only — referenced
/// by tests as the spec value, never read by library code.
constexpr int ConnectionHeartbeatInterval = 15;  // seconds
constexpr int StateHeartbeatInterval = 30;       // seconds

/// \brief VDA5050 protocol versions accepted by SchemaValidator.
///
/// Master rejects (outgoing) or drops (incoming) any message whose
/// header.version is not in this set. Currently 2.0.0 only; add a
/// 2.1.0 entry when that migration lands.
inline const std::vector<std::string> SupportedSchemaVersions = {"2.0.0"};

}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__STANDARD_NAMES_HPP_
