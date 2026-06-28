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

#ifndef VDA5050_MASTER_ROS2__INTERNAL__ROS2_TOPIC_NAMING_HPP_
#define VDA5050_MASTER_ROS2__INTERNAL__ROS2_TOPIC_NAMING_HPP_

#include <string>

namespace vda5050_master_ros2 {
namespace internal {

// ASCII-only character tests. ROS 2 topic segments are restricted to the
// ASCII set, so locale-dependent std::isalnum / std::isdigit (which can flag
// bytes 128-255 as alphanumeric under a non-C global locale) must not be used.
inline bool is_ascii_digit(char c)
{
  return c >= '0' && c <= '9';
}

inline bool is_ascii_alnum(char c)
{
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || is_ascii_digit(c);
}

// ROS 2 topic name segments must match ^[A-Za-z_][A-Za-z0-9_]*$. VDA5050
// allows a broader character set in serial_number (A-Z, a-z, 0-9, '_', '.',
// ':', '-') and no explicit restriction on manufacturer. Spec-legal vendor
// identities such as "001", "KION-001", or "agv.42" therefore break ROS 2
// topic creation when spliced into a per-AGV topic path.
//
// to_ros2_topic_segment() makes a minimal transformation:
//   1. Replace any character outside [A-Za-z0-9_] with '_'.
//   2. If the result then starts with a digit, prepend '_'.
// Letter-leading alphanumeric segments pass through unchanged so that
// operators correlating ROS 2 topics with VDA5050 wire identities can still
// recognise the original string in the common case.
//
// Examples:
//   "KION"      -> "KION"
//   "S001"      -> "S001"
//   "001"       -> "_001"
//   "KION-001"  -> "KION_001"
//   "agv.42"    -> "agv_42"
//   "3M"        -> "_3M"
//   ""          -> "_"
//
// Note on collisions: distinct raw identities can sanitize to the same ROS 2
// segment (e.g. "KION-001" and "KION_001" both -> "KION_001"). Master keeps
// the raw form everywhere (MQTT subscriptions, cache key, service requests);
// only the per-AGV ROS 2 topic builders apply this transform.
inline std::string to_ros2_topic_segment(const std::string& s)
{
  if (s.empty())
  {
    return "_";
  }
  std::string out;
  out.reserve(s.size() + 1);
  for (char c : s)
  {
    if (is_ascii_alnum(c) || c == '_')
    {
      out += c;
    }
    else
    {
      out += '_';
    }
  }
  if (is_ascii_digit(out[0]))
  {
    out = "_" + out;
  }
  return out;
}

// True when to_ros2_topic_segment(s) would alter s. Useful for one-shot
// logging without paying for the std::string allocation on the happy path.
inline bool needs_topic_sanitization(const std::string& s)
{
  if (s.empty())
  {
    return true;
  }
  if (is_ascii_digit(s[0]))
  {
    return true;
  }
  for (char c : s)
  {
    if (!is_ascii_alnum(c) && c != '_')
    {
      return true;
    }
  }
  return false;
}

// Build the per-AGV ROS 2 topic "/<namespace>/<mfg>/<serial>/<leaf>", with the
// manufacturer and serial sanitized into legal ROS 2 segments. An empty
// namespace publishes at the node's own namespace.
inline std::string build_per_agv_topic(
  const std::string& topic_namespace, const std::string& manufacturer,
  const std::string& serial_number, const std::string& leaf)
{
  std::string topic = "/";
  if (!topic_namespace.empty())
  {
    topic += topic_namespace;
    topic += "/";
  }
  topic += to_ros2_topic_segment(manufacturer);
  topic += "/";
  topic += to_ros2_topic_segment(serial_number);
  topic += "/";
  topic += leaf;
  return topic;
}

}  // namespace internal
}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__INTERNAL__ROS2_TOPIC_NAMING_HPP_
