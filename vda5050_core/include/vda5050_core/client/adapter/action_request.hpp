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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__ACTION_REQUEST_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__ACTION_REQUEST_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "vda5050_core/types/action.hpp"
#include "vda5050_core/types/action_parameter.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

class ActionRequest
{
public:
  const std::string& action_id() const;

  const std::string& action_type() const;

  const std::optional<std::vector<types::ActionParameter>>& action_parameters()
    const;

  const std::optional<std::string>& action_description() const;

  const std::optional<std::string>& order_id() const;

  std::optional<uint32_t> order_update_id() const;

private:
  friend class Adapter;

  static ActionRequest from_action(const types::Action& action);

  ActionRequest(
    const std::string& action_id, const std::string& action_type,
    std::optional<std::vector<types::ActionParameter>> action_parameters =
      std::nullopt,
    std::optional<std::string> action_description = std::nullopt);

  void add_order_information(
    const std::string& order_id, uint32_t order_update_id);

  std::string action_id_;
  std::string action_type_;
  std::optional<std::vector<types::ActionParameter>> action_parameters_;
  std::optional<std::string> action_description_;

  std::optional<std::string> order_id_;
  std::optional<uint32_t> order_update_id_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__ACTION_REQUEST_HPP_
