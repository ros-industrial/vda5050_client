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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__ORDER_EXECUTION_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__ORDER_EXECUTION_HPP_

#include <functional>
#include <memory>
#include <string>

#include "vda5050_core/client/adapter/execution.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

class OrderExecution : public Execution
{
public:
  static std::shared_ptr<OrderExecution> make(
    const std::string& order_id, uint32_t order_update_id,
    std::function<void()> finish_callback,
    std::function<void(std::string)> fail_callback);

  const std::string& order_id() const;

  uint32_t order_update_id() const;

private:
  OrderExecution(
    const std::string& order_id, uint32_t order_update_id,
    std::function<void()> finish_callback,
    std::function<void(std::string)> fail_callback);

  std::string order_id_;
  uint32_t order_update_id_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__ORDER_EXECUTION_HPP_
