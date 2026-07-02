/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
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

#ifndef VDA5050_CORE__MASTER__CONNECTION__CONNECTION_EVENT_DETECTOR_HPP_
#define VDA5050_CORE__MASTER__CONNECTION__CONNECTION_EVENT_DETECTOR_HPP_

#include <optional>

#include "vda5050_core/types/connection.hpp"

namespace vda5050_core {
namespace master {

/// \brief Kind of connection-state transition detected.
///
/// CONNECTIONBROKEN = broker last-will (unexpected drop); OFFLINE = graceful.
enum class ConnectionEventKind
{
  NONE,
  CONNECTED,
  OFFLINE,
  CONNECTIONBROKEN,
};

/// \brief Edge-detect a connection-state transition; non-NONE only on a change
/// (or first message). Sustained states return NONE.
ConnectionEventKind detect_connection_transition(
  const std::optional<vda5050_core::types::Connection>& prev,
  const vda5050_core::types::Connection& curr);

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__CONNECTION__CONNECTION_EVENT_DETECTOR_HPP_
