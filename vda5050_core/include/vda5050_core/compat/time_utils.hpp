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

#ifndef VDA5050_CORE__COMPAT__TIME_UTILS_HPP_
#define VDA5050_CORE__COMPAT__TIME_UTILS_HPP_

#include <ctime>

namespace vda5050_core {

namespace compat {

/// \brief Portable wrapper around the POSIX `gmtime_r` / Windows `gmtime_s`.
///
/// Converts a `std::time_t` to a broken-down UTC `std::tm` using the
/// thread-safe variant available on the current platform.
///
/// \param time Seconds since the Unix epoch
/// \return Broken-down time in UTC
inline std::tm to_gmtime(std::time_t time)
{
  std::tm result{};
#if defined(_WIN32)
  gmtime_s(&result, &time);
#else
  gmtime_r(&time, &result);
#endif
  return result;
}

/// \brief Portable wrapper around the POSIX `localtime_r` / Windows
/// `localtime_s`.
///
/// Converts a `std::time_t` to a broken-down local `std::tm` using the
/// thread-safe variant available on the current platform.
///
/// \param time Seconds since the Unix epoch
/// \return Broken-down time in local time
inline std::tm to_localtime(std::time_t time)
{
  std::tm result{};
#if defined(_WIN32)
  localtime_s(&result, &time);
#else
  localtime_r(&time, &result);
#endif
  return result;
}

/// \brief Portable wrapper around the POSIX `timegm` / Windows `_mkgmtime`.
///
/// Converts a broken-down UTC `std::tm` back to a `std::time_t`, interpreting
/// the fields as UTC (unlike `std::mktime`, which uses local time).
///
/// \param tm Broken-down time in UTC; may be normalised in place
/// \return Seconds since the Unix epoch
inline std::time_t timegm_utc(std::tm* tm)
{
#if defined(_WIN32)
  return _mkgmtime(tm);
#else
  return timegm(tm);
#endif
}

}  // namespace compat
}  // namespace vda5050_core

#endif  // VDA5050_CORE__COMPAT__TIME_UTILS_HPP_
