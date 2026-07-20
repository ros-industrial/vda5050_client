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

#ifndef VDA5050_CORE__VALIDATION__OPERATING_MODE_CONTROL_HPP_
#define VDA5050_CORE__VALIDATION__OPERATING_MODE_CONTROL_HPP_

#include "vda5050_core/types/operating_mode.hpp"

namespace vda5050_core {
namespace validation {

/// \brief True when the master drives the AGV in this mode (AUTOMATIC or
///        SEMIAUTOMATIC); false for MANUAL / SERVICE / TEACHIN.
bool is_master_in_control(vda5050_core::types::OperatingMode mode);

}  // namespace validation
}  // namespace vda5050_core

#endif  // VDA5050_CORE__VALIDATION__OPERATING_MODE_CONTROL_HPP_
