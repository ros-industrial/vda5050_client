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

#include "vda5050_core/master/connection/connection_event_detector.hpp"

namespace vda5050_core {
namespace master {

ConnectionEventKind detect_connection_transition(
  const std::optional<vda5050_core::types::Connection>& prev,
  const vda5050_core::types::Connection& curr)
{
  using vda5050_core::types::ConnectionState;

  if (prev.has_value() && prev->connection_state == curr.connection_state)
  {
    return ConnectionEventKind::NONE;
  }

  switch (curr.connection_state)
  {
    case ConnectionState::ONLINE:
      return ConnectionEventKind::CONNECTED;
    case ConnectionState::OFFLINE:
      return ConnectionEventKind::OFFLINE;
    case ConnectionState::CONNECTIONBROKEN:
      return ConnectionEventKind::CONNECTIONBROKEN;
  }

  return ConnectionEventKind::NONE;
}

}  // namespace master
}  // namespace vda5050_core
