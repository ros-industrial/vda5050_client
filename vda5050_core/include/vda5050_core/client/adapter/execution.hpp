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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__EXECUTION_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__EXECUTION_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace vda5050_core {

namespace client {

namespace adapter {

class Execution : public std::enable_shared_from_this<Execution>
{
public:
  virtual ~Execution();

  void finished();

  void failed(const std::string& reason);

  bool okay() const;

  bool is_finished() const;

  const std::optional<std::string>& failure_reason() const;

protected:
  Execution(
    std::function<void()> finish_callback,
    std::function<void(std::string)> fail_callback);

private:
  friend class Adapter;

  void deactivate();

  std::function<void()> finish_callback_;
  std::function<void(std::string)> fail_callback_;

  std::optional<std::string> failure_reason_;

  std::atomic_bool completed_;
  std::atomic_bool active_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__EXECUTION_HPP_
