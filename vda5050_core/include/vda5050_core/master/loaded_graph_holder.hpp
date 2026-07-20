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

#ifndef VDA5050_CORE__MASTER__LOADED_GRAPH_HOLDER_HPP_
#define VDA5050_CORE__MASTER__LOADED_GRAPH_HOLDER_HPP_

#include <mutex>
#include <utility>

#include "vda5050_core/layout/graph.hpp"

namespace vda5050_core {

namespace master {

/// \brief Thread-safe holder for the fleet's loaded layout graph, shared by
///        the master (writer) and each AGV queue thread (reader).
///
/// Held via shared_ptr by the master and every AGV, so it outlives them all;
/// the mutex is a leaf, so readers never couple to the master's lifetime.
class LoadedGraphHolder
{
public:
  vda5050_core::layout::Graph::ConstPtr get() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return graph_;
  }

  void set(vda5050_core::layout::Graph::ConstPtr graph)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    graph_ = std::move(graph);
  }

private:
  mutable std::mutex mutex_;
  vda5050_core::layout::Graph::ConstPtr graph_;
};

}  // namespace master

}  // namespace vda5050_core

#endif  // VDA5050_CORE__MASTER__LOADED_GRAPH_HOLDER_HPP_
