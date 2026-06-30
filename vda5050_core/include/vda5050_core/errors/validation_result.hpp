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

#include <algorithm>
#include <iterator>
#include <vector>

#include "vda5050_core/types/error.hpp"

namespace vda5050_core {

namespace errors {

struct ValidationResult
{
  /// \brief Entries produced by validation. FATAL entries are hard failures
  /// (see has_fatal()); WARNING entries are advisory. Empty when validation
  /// passed with nothing to report.
  std::vector<vda5050_core::types::Error> errors;

  /// \brief True iff any entry is FATAL.
  bool has_fatal() const
  {
    return std::any_of(
      errors.begin(), errors.end(), [](const vda5050_core::types::Error& e) {
        return e.error_level == vda5050_core::types::ErrorLevel::FATAL;
      });
  }

  /// \brief The advisory (WARNING-level) entries. The consumer decides whether
  /// to surface them; validation itself does not log them.
  std::vector<vda5050_core::types::Error> warnings() const
  {
    std::vector<vda5050_core::types::Error> out;
    std::copy_if(
      errors.begin(), errors.end(), std::back_inserter(out),
      [](const vda5050_core::types::Error& e) {
        return e.error_level == vda5050_core::types::ErrorLevel::WARNING;
      });
    return out;
  }

  /// \brief Allows use in boolean contexts.
  ///
  /// \return True iff there are no entries at all (fully clean). Consumers that
  /// tolerate advisory WARNING entries should gate on has_fatal() instead.
  explicit operator bool() const
  {
    return errors.empty();
  }
};

}  // namespace errors
}  // namespace vda5050_core

#endif  // VDA5050_CORE__ERRORS__VALIDATION_RESULT_HPP_
