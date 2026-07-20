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

#include "vda5050_core/client/adapter/execution.hpp"
#include "vda5050_core/logger/logger.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

//=============================================================================
Execution::~Execution()
{
  if (!completed_)
  {
    VDA5050_WARN(
      "Execution destroyed before finished() or failed() was called");
  }
}

//=============================================================================
void Execution::finished()
{
  if (completed_.exchange(true)) return;

  if (finish_callback_) finish_callback_();
}

//=============================================================================
void Execution::failed(const std::string& reason)
{
  if (completed_.exchange(true)) return;

  failure_reason_ = reason;
  if (fail_callback_) fail_callback_(reason);
}

//=============================================================================
bool Execution::okay() const
{
  return active_;
}

//=============================================================================
bool Execution::is_finished() const
{
  return completed_;
}

//=============================================================================
const std::optional<std::string>& Execution::failure_reason() const
{
  return failure_reason_;
}

//=============================================================================
Execution::Execution(
  std::function<void()> finish_callback,
  std::function<void(std::string)> fail_callback)
: finish_callback_(std::move(finish_callback)),
  fail_callback_(std::move(fail_callback)),
  completed_(false),
  active_(false)
{
  // Nothing to do here ...
}

//=============================================================================
void Execution::deactivate()
{
  active_ = false;
}

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core
