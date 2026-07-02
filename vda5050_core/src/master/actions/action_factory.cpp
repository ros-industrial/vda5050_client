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

#include "vda5050_core/master/actions/action_factory.hpp"

#include <array>
#include <cstdint>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace vda5050_core::master {

namespace {

// Per RFC 4122 §3: textual UUID layout is 8-4-4-4-12 hex digits with dashes.
constexpr std::array<int, 5> kUuidGroupHexLengths = {8, 4, 4, 4, 12};

// Thread-local 64-bit Mersenne Twister seeded once per thread from
// random_device. UUID generation is multi-thread-safe without locking.
std::mt19937_64& thread_local_rng()
{
  thread_local std::mt19937_64 rng{[] {
    std::random_device rd;
    std::seed_seq seed{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
    return std::mt19937_64{seed};
  }()};
  return rng;
}

}  // namespace

vda5050_core::types::Action ActionFactory::build_custom(
  const std::string& action_type, const std::string& action_id,
  vda5050_core::types::BlockingType blocking_type,
  const std::string& description,
  const std::vector<vda5050_core::types::ActionParameter>& parameters)
{
  vda5050_core::types::Action a;
  a.action_type = action_type;
  a.action_id = action_id;
  a.blocking_type = blocking_type;
  if (!description.empty())
  {
    a.action_description = description;
  }
  if (!parameters.empty())
  {
    a.action_parameters = parameters;
  }
  return a;
}

vda5050_core::types::Action ActionFactory::build_state_request(
  const std::string& action_id, const std::string& description)
{
  // stateRequest takes no parameters; NONE blocking.
  return build_custom(
    "stateRequest", action_id, vda5050_core::types::BlockingType::NONE,
    description, {});
}

vda5050_core::types::Action ActionFactory::build_factsheet_request(
  const std::string& action_id, const std::string& description)
{
  // factsheetRequest takes no parameters; NONE blocking.
  return build_custom(
    "factsheetRequest", action_id, vda5050_core::types::BlockingType::NONE,
    description, {});
}

std::string ActionFactory::generate_action_id()
{
  // 16 random bytes, then patch RFC 4122 v4 + variant bits.
  std::uint8_t bytes[16];
  auto& rng = thread_local_rng();
  for (int i = 0; i < 16; i += 8)
  {
    const std::uint64_t r = rng();
    for (int j = 0; j < 8; ++j)
    {
      bytes[i + j] = static_cast<std::uint8_t>((r >> (j * 8)) & 0xFFu);
    }
  }
  // Version 4 (random): set high nibble of byte 6 to 0x4.
  bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0Fu) | 0x40u);
  // Variant 1 (RFC 4122): set top two bits of byte 8 to 0b10.
  bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3Fu) | 0x80u);

  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(36);
  int byte_idx = 0;
  for (std::size_t group = 0; group < kUuidGroupHexLengths.size(); ++group)
  {
    if (group != 0) out.push_back('-');
    const int hex_chars = kUuidGroupHexLengths[group];
    for (int k = 0; k < hex_chars; k += 2)
    {
      const std::uint8_t b = bytes[byte_idx++];
      out.push_back(kHex[(b >> 4) & 0x0Fu]);
      out.push_back(kHex[b & 0x0Fu]);
    }
  }
  return out;
}

}  // namespace vda5050_core::master
