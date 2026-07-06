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

#ifndef VDA5050_CORE__CLIENT__ADAPTER__EDGE_REQUEST_HPP_
#define VDA5050_CORE__CLIENT__ADAPTER__EDGE_REQUEST_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "vda5050_core/types/edge.hpp"
#include "vda5050_core/types/trajectory.hpp"

namespace vda5050_core {

namespace client {

namespace adapter {

class EdgeRequest
{
public:
  const std::string& edge_id() const;

  uint32_t sequence_id() const;

  const std::optional<types::Trajectory>& trajectory();

  std::optional<double> max_speed() const;

  std::optional<double> min_height() const;

  std::optional<double> max_height() const;

  std::optional<bool> rotation_allowed() const;

  std::optional<double> max_rotation_speed() const;

  std::optional<double> length() const;

private:
  friend class Adapter;

  static EdgeRequest from_edge(const types::Edge& edge);

  EdgeRequest(
    const std::string& edge_id, uint32_t sequence_id,
    std::optional<types::Trajectory> trajectory = std::nullopt,
    std::optional<double> max_speed = std::nullopt,
    std::optional<double> min_height = std::nullopt,
    std::optional<double> max_height = std::nullopt,
    std::optional<bool> rotation_allowed = std::nullopt,
    std::optional<double> max_rotation_speed = std::nullopt,
    std::optional<double> length = std::nullopt);

  std::string edge_id_;
  uint32_t sequence_id_;

  std::optional<types::Trajectory> trajectory_;

  std::optional<double> max_speed_;
  std::optional<double> min_height_;
  std::optional<double> max_height_;

  std::optional<bool> rotation_allowed_;
  std::optional<double> max_rotation_speed_;

  std::optional<double> length_;
};

}  // namespace adapter
}  // namespace client
}  // namespace vda5050_core

#endif  // VDA5050_CORE__CLIENT__ADAPTER__EDGE_REQUEST_HPP_
