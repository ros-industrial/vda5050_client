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

#include "vda5050_core/validation/pre_send_validator.hpp"

#include <fmt/format.h>
#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"
#include "vda5050_core/validation/operating_mode_control.hpp"

namespace vda5050_core::validation {

namespace {

const char* agv_state_name(master::AGVState state)
{
  switch (state)
  {
    case master::AGVState::STATE_UNKNOWN:
      return "STATE_UNKNOWN";
    case master::AGVState::AVAILABLE:
      return "AVAILABLE";
    case master::AGVState::UNAVAILABLE:
      return "UNAVAILABLE";
    case master::AGVState::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

}  // namespace

errors::ValidationResult validate_pre_send(const PreSendContext& ctx)
{
  errors::ValidationResult res;

  auto add_error = [&](const std::string& description) {
    res.add_error(
      errors::create_error(errors::PreSendValidationError, description, {}));
  };

  if (ctx.connection_status != types::ConnectionState::ONLINE)
  {
    add_error("AGV connection_status is not ONLINE");
  }

  if (ctx.operational_state != master::AGVState::AVAILABLE)
  {
    add_error(fmt::format(
      "AGV operational_state is not AVAILABLE ({})",
      agv_state_name(ctx.operational_state)));
  }

  if (!ctx.last_state.has_value())
  {
    add_error("AGV has not yet reported any State");
    return res;
  }

  if (!is_master_in_control(ctx.last_state->operating_mode))
  {
    add_error("AGV operating_mode is not AUTOMATIC or SEMIAUTOMATIC");
  }

  if (ctx.last_state->paused.value_or(false))
  {
    add_error("AGV is paused");
  }

  if (ctx.last_state->safety_state.e_stop != types::EStop::NONE)
  {
    add_error("AGV e-stop is engaged");
  }

  if (
    !ctx.last_state->agv_position.has_value() ||
    !ctx.last_state->agv_position->position_initialized)
  {
    add_error("AGV position is not initialized");
  }

  return res;
}

}  // namespace vda5050_core::validation
