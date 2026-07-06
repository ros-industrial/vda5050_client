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

#ifndef VDA5050_CORE__ERRORS__ERROR_FACTORY_HPP_
#define VDA5050_CORE__ERRORS__ERROR_FACTORY_HPP_

#include <string>
#include <vector>

#include <vda5050_core/types/error.hpp>

namespace vda5050_core {

namespace errors {

/// \brief Helper to create an error struct
///
/// \param type Error type
/// \param description Brief description of error
/// \param refs List of error references
/// \param level Error level. Default is FATAL — create_error is used for
///        rejections, which invalidate the result. Pass ErrorLevel::WARNING
///        explicitly for advisory entries that should not invalidate it.
///
/// \return Constructed error struct
inline vda5050_core::types::Error create_error(
  const std::string& type, const std::string& description,
  const std::vector<vda5050_core::types::ErrorReference>& refs,
  vda5050_core::types::ErrorLevel level =
    vda5050_core::types::ErrorLevel::FATAL)
{
  return vda5050_core::types::Error{type, refs, description, level};
}

}  // namespace errors
}  // namespace vda5050_core

#endif  // VDA5050_CORE__ERRORS__ERROR_FACTORY_HPP_
