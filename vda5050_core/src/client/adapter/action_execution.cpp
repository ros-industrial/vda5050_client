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

#include "vda5050_core/client/adapter/action_execution.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
std::shared_ptr<ActionExecution> ActionExecution::make(
  std::function<void(types::ActionStatus, std::optional<std::string>)>
    status_update_callback)
{
  auto execution = std::shared_ptr<ActionExecution>(
    new ActionExecution(std::move(status_update_callback)));
  return execution;
}

//=============================================================================
void ActionExecution::intializing()
{
  if (status_update_callback_)
    status_update_callback_(types::ActionStatus::INITIALIZING, std::nullopt);
}

//=============================================================================
void ActionExecution::running()
{
  if (status_update_callback_)
    status_update_callback_(types::ActionStatus::RUNNING, std::nullopt);
}

//=============================================================================
void ActionExecution::paused(std::optional<std::string> result_description)
{
  if (status_update_callback_)
    status_update_callback_(
      types::ActionStatus::PAUSED, std::move(result_description));
}

//=============================================================================
void ActionExecution::finished()
{
  Execution::finished();
}

//=============================================================================
void ActionExecution::finished(const std::string& result_description)
{
  if (is_finished()) return;

  if (status_update_callback_)
    status_update_callback_(types::ActionStatus::FINISHED, result_description);

  status_update_callback_ = nullptr;

  Execution::finished();
}

//=============================================================================
ActionExecution::ActionExecution(
  std::function<void(types::ActionStatus, std::optional<std::string>)>
    status_update_callback)
: Execution(
    [cb = status_update_callback]() {
      if (cb) cb(types::ActionStatus::FINISHED, std::nullopt);
    },
    [cb = status_update_callback](std::string reason) {
      if (cb) cb(types::ActionStatus::FAILED, std::move(reason));
    }),
  status_update_callback_(std::move(status_update_callback))
{
  // Nothing to do here ...
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
