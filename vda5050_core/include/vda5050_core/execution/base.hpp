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

#ifndef VDA5050_CORE__EXECUTION__BASE_HPP_
#define VDA5050_CORE__EXECUTION__BASE_HPP_

#include <typeindex>

namespace vda5050_core {

namespace execution {

struct Base
{
  virtual ~Base() = default;
  virtual std::type_index get_type() const = 0;
};

struct EventBase : public Base
{};

struct UpdateBase : public Base
{};

struct ResourceBase : public Base
{};

template <typename DerivedT, typename BaseT>
struct Initialize : public BaseT
{
  std::type_index get_type() const override
  {
    // Computed once and cached (as before), but as a function-local static so
    // typeid(DerivedT) lives in a deferred function body. A static *member*
    // initializer is instantiated eagerly by MSVC while DerivedT is still an
    // incomplete CRTP base, which fails with error C2027.
    static const std::type_index type = std::type_index(typeid(DerivedT));
    return type;
  }
};

}  // namespace execution
}  // namespace vda5050_core

#endif  // VDA5050_CORE__EXECUTION__BASE_HPP_
