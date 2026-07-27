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

#ifndef VDA5050_CORE__TYPES__PROTOCOL_VERSION_HPP_
#define VDA5050_CORE__TYPES__PROTOCOL_VERSION_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace vda5050_core {

namespace types {

/// \brief Strongly-typed VDA5050 protocol version.
///
/// Canonical representation of the VDA5050 protocol versions supported by
/// this library. Single source of truth for which versions are supported,
/// conversion to the full semantic version string used in message headers
/// (e.g. "2.0.0"), and conversion to the MQTT topic version segment (e.g.
/// "v2").
class ProtocolVersion
{
public:
  static const ProtocolVersion V2_0_0;

  /// \brief Parses a semantic version string (e.g. "2.0.0").
  /// \return the matching ProtocolVersion, or std::nullopt if `version` is
  ///   not a supported version.
  static std::optional<ProtocolVersion> from_string(const std::string& version);

  /// \brief All VDA5050 protocol versions supported by this library.
  static const std::array<ProtocolVersion, 1>& supported_versions();

  /// \brief Full semantic version string, e.g. "2.0.0".
  std::string to_string() const;

  /// \brief MQTT topic version segment, e.g. "v2".
  std::string to_topic_version() const;

  /// \brief Equality operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is equal?
  constexpr bool operator==(const ProtocolVersion& other) const
  {
    if (this->major_ != other.major_) return false;
    if (this->minor_ != other.minor_) return false;
    if (this->patch_ != other.patch_) return false;
    return true;
  }

  /// \brief Inequality operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is not equal?
  constexpr bool operator!=(const ProtocolVersion& other) const
  {
    return !(this->operator==(other));
  }

  /// \brief Less-than operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is less than?
  constexpr bool operator<(const ProtocolVersion& other) const
  {
    if (this->major_ != other.major_) return this->major_ < other.major_;
    if (this->minor_ != other.minor_) return this->minor_ < other.minor_;
    return this->patch_ < other.patch_;
  }

  /// \brief Less-than-or-equal operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is less than or equal?
  constexpr bool operator<=(const ProtocolVersion& other) const
  {
    return !(other < *this);
  }

  /// \brief Greater-than operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is greater than?
  constexpr bool operator>(const ProtocolVersion& other) const
  {
    return other < *this;
  }

  /// \brief Greater-than-or-equal operator
  ///
  /// \param other The other object to compare to
  ///
  /// \return is greater than or equal?
  constexpr bool operator>=(const ProtocolVersion& other) const
  {
    return !(*this < other);
  }

private:
  constexpr ProtocolVersion(uint8_t major, uint8_t minor, uint8_t patch)
  : major_(major), minor_(minor), patch_(patch)
  {
    // Nothing to do here ...
  }

  static const std::array<ProtocolVersion, 1> kSupportedVersions_;

  uint8_t major_;
  uint8_t minor_;
  uint8_t patch_;
};

inline constexpr ProtocolVersion ProtocolVersion::V2_0_0{2, 0, 0};

inline constexpr std::array<ProtocolVersion, 1>
  ProtocolVersion::kSupportedVersions_{V2_0_0};

inline const std::array<ProtocolVersion, 1>&
ProtocolVersion::supported_versions()
{
  return kSupportedVersions_;
}

inline std::string ProtocolVersion::to_string() const
{
  return std::to_string(major_) + "." + std::to_string(minor_) + "." +
         std::to_string(patch_);
}

inline std::string ProtocolVersion::to_topic_version() const
{
  return "v" + std::to_string(major_);
}

inline std::optional<ProtocolVersion> ProtocolVersion::from_string(
  const std::string& version)
{
  for (const auto& supported : supported_versions())
  {
    if (supported.to_string() == version) return supported;
  }

  return std::nullopt;
}

}  // namespace types
}  // namespace vda5050_core

#endif  // VDA5050_CORE__TYPES__PROTOCOL_VERSION_HPP_
