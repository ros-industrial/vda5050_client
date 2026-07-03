/*
 * Copyright (C) 2025 ROS-Industrial Consortium Asia Pacific
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

#ifndef VDA5050_CORE__ERRORS__VALIDATION_RESULT_HPP_
#define VDA5050_CORE__ERRORS__VALIDATION_RESULT_HPP_

#include <utility>
#include <vector>

#include "vda5050_core/types/error.hpp"

namespace vda5050_core {

namespace errors {

/// \brief Outcome of a validation pass: hard failures and advisories.
///
/// FATAL entries invalidate the result; WARNING entries are advisory and the
/// consumer decides whether to surface them. Entries are added via add_error(),
/// which routes each one to the correct vector by its error_level.
class ValidationResult
{
public:
  /// \brief Record an entry, routing it by its error_level.
  ///
  /// \param error WARNING entries go to warnings(); everything else is treated
  ///        as a hard failure and goes to fatal_errors().
  void add_error(vda5050_core::types::Error error)
  {
    if (error.error_level == vda5050_core::types::ErrorLevel::WARNING)
    {
      warnings_.push_back(std::move(error));
    }
    else
    {
      fatal_errors_.push_back(std::move(error));
    }
  }

  /// \brief Hard failures that invalidate the result.
  const std::vector<vda5050_core::types::Error>& fatal_errors() const
  {
    return fatal_errors_;
  }

  /// \brief Advisory entries that do not invalidate the result.
  const std::vector<vda5050_core::types::Error>& warnings() const
  {
    return warnings_;
  }

  /// \brief True iff there is at least one fatal error.
  bool has_fatal() const
  {
    return !fatal_errors_.empty();
  }

  /// \brief True iff there is at least one advisory warning.
  bool has_warnings() const
  {
    return !warnings_.empty();
  }

  /// \brief Allows use in boolean contexts.
  ///
  /// \return True iff there are no fatal errors. WARNING-level advisories do
  /// not make the result invalid.
  explicit operator bool() const
  {
    return fatal_errors_.empty();
  }

private:
  std::vector<vda5050_core::types::Error> fatal_errors_;
  std::vector<vda5050_core::types::Error> warnings_;
};

}  // namespace errors
}  // namespace vda5050_core

#endif  // VDA5050_CORE__ERRORS__VALIDATION_RESULT_HPP_
