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

#ifndef VDA5050_CORE__CLIENT__EVENTS__EXECUTE_ACTION_HPP_
#define VDA5050_CORE__CLIENT__EVENTS__EXECUTE_ACTION_HPP_

#include <memory>
#include <utility>

#include "vda5050_core/client/adapter/action_execution.hpp"
#include "vda5050_core/client/adapter/action_request.hpp"
#include "vda5050_core/execution/base.hpp"

namespace vda5050_core {

namespace client {

/// \brief Event requesting execution of an order action.
///
/// Emitted by `OrderActions` once an action is allowed to start. The action
/// handler should execute `request` and report progress or completion through
/// `execution`.
struct ExecuteActionEvent
: public execution::Initialize<ExecuteActionEvent, execution::EventBase>
{
  adapter::ActionRequest request;
  std::shared_ptr<adapter::ActionExecution> execution;

  ExecuteActionEvent(
    adapter::ActionRequest request,
    std::shared_ptr<adapter::ActionExecution> execution)
  : request(std::move(request)), execution(std::move(execution))
  {
  }
};

}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__EVENTS__EXECUTE_ACTION_HPP_
