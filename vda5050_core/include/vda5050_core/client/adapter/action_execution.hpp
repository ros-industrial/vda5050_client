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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__ACTION_EXECUTION_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__ACTION_EXECUTION_HPP_

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "vda5050_core/client/adapter/execution.hpp"
#include "vda5050_core/types/action_status.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

class ActionExecution : public Execution
{
public:
  static std::shared_ptr<ActionExecution> make(
    std::function<void(types::ActionStatus, std::optional<std::string>)>
      status_update_callback);

  void intializing();

  void running();

  void paused(std::optional<std::string> result_description = std::nullopt);

  void finished();

  void finished(const std::string& result_description);

private:
  ActionExecution(
    std::function<void(types::ActionStatus, std::optional<std::string>)>
      status_update_callback);

  std::function<void(types::ActionStatus, std::optional<std::string>)>
    status_update_callback_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__ACTION_EXECUTION_HPP_
