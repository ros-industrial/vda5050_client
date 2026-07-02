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

#ifndef VDA5050_MASTER_ROS2__INTERNAL__TO_MSG_HPP_
#define VDA5050_MASTER_ROS2__INTERNAL__TO_MSG_HPP_

#include "nlohmann/json.hpp"
#include "vda5050_core/json_utils/serialization.hpp"

namespace vda5050_master_ros2 {
namespace internal {

// Convert a vda5050_core::types::T value to its vda5050_interfaces::msg::T
// counterpart via a JSON intermediate. Both directions of the JSON
// adapters are provided by vda5050_json_utils when ENABLE_ROS2 is set
// at compile time (vda5050_master/CMakeLists.txt does this on the master
// library target). ADL resolves `to_json` to the vda5050_types overload
// and `from_json` to the vda5050_interfaces::msg overload.
template <typename TypesT, typename MsgT>
MsgT to_msg(const TypesT& src)
{
  nlohmann::json j;
  to_json(j, src);
  MsgT dst;
  from_json(j, dst);
  return dst;
}

// Mirror of to_msg<>: convert a vda5050_interfaces::msg::T value to
// its vda5050_core::types::T counterpart via a JSON intermediate. Used by
// write-path services (e.g., AssignOrder) that receive a ROS msg from
// the FMS and need to feed it into the master's C++ pipeline.
// ADL resolves `to_json` to the vda5050_interfaces::msg overload and
// `from_json` to the vda5050_types overload.
template <typename MsgT, typename TypesT>
TypesT from_msg(const MsgT& src)
{
  nlohmann::json j;
  to_json(j, src);
  TypesT dst;
  from_json(j, dst);
  return dst;
}

}  // namespace internal
}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__INTERNAL__TO_MSG_HPP_
