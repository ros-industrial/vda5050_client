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

#ifndef VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTIONS_PUBLISHER_HPP_
#define VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTIONS_PUBLISHER_HPP_

#include "vda5050_core/errors/validation_result.hpp"
#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/types/instant_actions.hpp"

namespace vda5050_core {
namespace validation {
// Forward-declared to avoid pulling agv.hpp (transitive cycle) into this
// header; defined in vda5050_core/validation/pre_send_validator.hpp.
struct PreSendContext;
}  // namespace validation
}  // namespace vda5050_core

namespace vda5050_core {
namespace master {

/// \brief The stateless gate step that rejected the actions, if any.
enum class ActionGateStep
{
  NONE,
  MODE,
  CAPABILITY,
  CONFLICT,
  LIMITS
};

/// \brief Outcome of the shared stateless action gate.
struct ActionGateResult
{
  vda5050_core::errors::ValidationResult result;
  ActionGateStep failed = ActionGateStep::NONE;
};

/// \brief Runs the outgoing instant-action validator chain, then publishes.
///
/// Stateless; safe to call concurrently.
class InstantActionsPublisher
{
public:
  InstantActionsPublisher() = default;

  /// \brief Run the stateless action gate (mode, capability, conflict) shared
  ///        by the synchronous pre-flight and the queue-thread publish.
  static ActionGateResult validate_gate(
    const vda5050_core::validation::PreSendContext& ctx,
    const vda5050_core::types::InstantActions& actions);

  /// \brief Validate instant actions (schema, online, mode gate, capability,
  ///        action conflict) and publish only if all pass.
  ///
  /// \param adapter  per-AGV typed adapter (caller-owned)
  /// \param ctx      AGV readiness snapshot, built by the caller
  /// \param actions  the instant actions message to publish
  /// \return validation result; no fatal errors means published
  vda5050_core::errors::ValidationResult publish(
    vda5050_core::execution::ProtocolAdapter& adapter,
    const vda5050_core::validation::PreSendContext& ctx,
    const vda5050_core::types::InstantActions& actions);
};

}  // namespace master
}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__ACTIONS__INSTANT_ACTIONS_PUBLISHER_HPP_
