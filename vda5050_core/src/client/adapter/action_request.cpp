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

#include "vda5050_core/client/adapter/action_request.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
const std::string& ActionRequest::action_id() const
{
  return action_id_;
}

//=============================================================================
const std::string& ActionRequest::action_type() const
{
  return action_type_;
}

//=============================================================================
const std::optional<std::vector<types::ActionParameter>>&
ActionRequest::action_parameters() const
{
  return action_parameters_;
}

//=============================================================================
const std::optional<std::string>& ActionRequest::action_description() const
{
  return action_description_;
}

//=============================================================================
const std::optional<std::string>& ActionRequest::order_id() const
{
  return order_id_;
}

//=============================================================================
std::optional<uint32_t> ActionRequest::order_update_id() const
{
  return order_update_id_;
}

//=============================================================================
ActionRequest ActionRequest::from_action(const types::Action& action)
{
  return ActionRequest(
    action.action_id, action.action_type, action.action_parameters,
    action.action_description);
}

//=============================================================================
ActionRequest::ActionRequest(
  const std::string& action_id, const std::string& action_type,
  std::optional<std::vector<types::ActionParameter>> action_parameters,
  std::optional<std::string> action_description)
: action_id_(action_id),
  action_type_(action_type),
  action_parameters_(std::move(action_parameters)),
  action_description_(std::move(action_description))
{
  // Nothing to do here ...
}

//=============================================================================
void ActionRequest::add_order_information(
  const std::string& order_id, uint32_t order_update_id)
{
  order_id_ = order_id;
  order_update_id_ = order_update_id;
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
