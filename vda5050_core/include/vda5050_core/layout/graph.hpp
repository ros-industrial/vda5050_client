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

#ifndef VDA5050_CORE__LAYOUT__GRAPH_HPP_
#define VDA5050_CORE__LAYOUT__GRAPH_HPP_

#include <cstddef>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "vda5050_core/layout/lif.hpp"

namespace vda5050_core::layout {

/// \brief Master-side wrapper over a validated LIF.
///
/// Adds hash-backed indices and per-node adjacency lists. Built once from a
/// LIF; mutations update the indices in place to keep them consistent with the
/// underlying LIF.
class Graph
{
public:
  using Ptr = std::shared_ptr<Graph>;
  using ConstPtr = std::shared_ptr<const Graph>;

  /// \brief Factory; takes ownership of `lif`.
  ///
  /// Validates `lif` with validate_layout before wrapping it, so the same rules
  /// apply whether the LIF comes from the loader or is built in memory.
  ///
  /// \param lif Layout to wrap.
  /// \throws std::invalid_argument if the LIF fails validation (empty,
  ///   duplicate IDs, or dangling edge / station references).
  static Ptr from_lif(LIF lif);

  Graph(const Graph&) = default;
  Graph(Graph&&) noexcept = default;
  Graph& operator=(const Graph&) = default;
  Graph& operator=(Graph&&) noexcept = default;
  ~Graph() = default;

  /// \brief Check whether a node / edge / station exists.
  ///
  /// \param id Entity id to look up.
  /// \return True if the entity is present.
  bool has_node(const std::string& id) const noexcept;
  bool has_edge(const std::string& id) const noexcept;
  bool has_station(const std::string& id) const noexcept;

  /// \brief Look up a node / edge / station by id.
  ///
  /// \param id Entity id.
  /// \return Reference to the entity.
  /// \throws std::out_of_range if absent.
  const Node& get_node(const std::string& id) const;
  const Edge& get_edge(const std::string& id) const;
  const Station& get_station(const std::string& id) const;

  /// \brief Look up a node / edge / station / layout by id; nullable.
  ///
  /// Returned pointers are invalidated by any subsequent mutation
  /// (`add_*`, `delete_*`, `update_*_id`, `prune`). Treat them as short-lived,
  /// similar to std::vector iterators.
  ///
  /// \param id Entity id.
  /// \return Pointer to the entity, or nullptr if absent.
  const Node* find_node(const std::string& id) const noexcept;
  const Edge* find_edge(const std::string& id) const noexcept;
  const Station* find_station(const std::string& id) const noexcept;
  const Layout* find_layout(const std::string& layout_id) const noexcept;

  /// \brief Edges incident to a node; multi-edge safe (vector, since LIF allows
  /// multiple parallel edges between the same node pair).
  ///
  /// \param node_id Node whose incident edges to return.
  /// \return Ids of edges leaving / entering the node.
  /// \throws std::out_of_range if `node_id` doesn't exist.
  const std::vector<std::string>& outbound_edges(
    const std::string& node_id) const;
  const std::vector<std::string>& inbound_edges(
    const std::string& node_id) const;

  /// \brief Return edge ids for edges from `from_node_id` to `to_node_id`.
  ///
  /// \param from_node_id Source node id.
  /// \param to_node_id Target node id.
  /// \return Ids of edges connecting the two nodes.
  std::vector<std::string> edges_between(
    const std::string& from_node_id, const std::string& to_node_id) const;

  /// \brief Iterate over nodes / edges / stations.
  ///
  /// Visitor return value is ignored; iteration does not short-circuit. The
  /// `_ordered` variants visit entities sorted by id.
  ///
  /// \param visitor Callable invoked with each entity.
  template <typename V>
  void for_each_node(V&& visitor) const;
  template <typename V>
  void for_each_edge(V&& visitor) const;
  template <typename V>
  void for_each_station(V&& visitor) const;
  template <typename V>
  void for_each_node_ordered(V&& visitor) const;
  template <typename V>
  void for_each_edge_ordered(V&& visitor) const;
  template <typename V>
  void for_each_station_ordered(V&& visitor) const;

  /// \brief Return graph size / state (entity counts and emptiness).
  std::size_t node_count() const noexcept;
  std::size_t edge_count() const noexcept;
  std::size_t station_count() const noexcept;
  bool empty() const noexcept;

  /// \brief Underlying LIF.
  ///
  /// No non-const accessor by design — the owned LIF is mutable only through
  /// the typed mutation methods so that indices and adjacency stay in sync.
  ///
  /// \return Const reference to the wrapped LIF.
  const LIF& lif() const noexcept
  {
    return lif_;
  }

  /// \brief Return ids of unconnected nodes.
  ///
  /// A node is considered unconnected when it has no inbound edges, no outbound
  /// edges, and is not referenced by any station's interactionNodeIds.
  ///
  /// \return Ids of the unconnected nodes.
  std::vector<std::string> unconnected_nodes() const;

  /// \brief Add a node / edge / station to a layout.
  ///
  /// All mutations update the indices and invalidate any previously-returned
  /// const T* / const Layout*.
  ///
  /// \param node Entity to add (edge / station for the other overloads).
  /// \param layout_id Layout to add it to.
  /// \throws std::invalid_argument on duplicate id, missing layout, or
  ///   unresolved reference.
  void add_node(Node node, const std::string& layout_id);
  void add_edge(Edge edge, const std::string& layout_id);
  void add_station(Station station, const std::string& layout_id);

  /// \brief Delete a node / edge / station.
  ///
  /// \param id Entity id to delete.
  /// \throws std::invalid_argument if not found; delete_node also throws if the
  ///   node is still referenced by an edge (start/end) or station
  ///   (interactionNodeIds).
  void delete_node(const std::string& id);
  void delete_edge(const std::string& id);
  void delete_station(const std::string& id);

  /// \brief Update an entity's id, rewriting it AND all references to it.
  ///
  /// \param old_id Current id.
  /// \param new_id Replacement id.
  /// \throws std::invalid_argument if new_id already exists.
  void update_node_id(const std::string& old_id, const std::string& new_id);
  void update_edge_id(const std::string& old_id, const std::string& new_id);
  void update_station_id(const std::string& old_id, const std::string& new_id);

  /// \brief Delete all nodes returned by unconnected_nodes().
  ///
  /// \return Ids of the nodes removed.
  std::vector<std::string> prune();

  /// \brief Deep equality check.
  ///
  /// Structural and order-sensitive. Two graphs with the same node IDs and
  /// properties in different layout order are not equal.
  ///
  /// \param rhs Graph to compare against.
  bool operator==(const Graph& rhs) const;
  bool operator!=(const Graph& rhs) const;

  /// \brief Dump the graph in Graphviz DOT format.
  ///
  /// Edge labels are edge IDs.
  ///
  /// \param os Stream to write the DOT output to.
  void dump_dot(std::ostream& os) const;

private:
  Graph() = default;
  void rebuild_indices();

  LIF lif_;
  // layout_id -> index into lif_.layouts.
  std::unordered_map<std::string, std::size_t> layout_index_;
  // (layout_idx in lif_.layouts, element_idx in that layout's vector).
  std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>
    node_index_;
  std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>
    edge_index_;
  std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>
    station_index_;
  std::unordered_map<std::string, std::vector<std::string>> node_outbound_;
  std::unordered_map<std::string, std::vector<std::string>> node_inbound_;
  // node_id -> station_ids whose interactionNodeIds reference that node.
  std::unordered_map<std::string, std::vector<std::string>> node_stations_;
};

template <typename V>
void Graph::for_each_node(V&& visitor) const
{
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& node : layout.nodes)
    {
      visitor(node);
    }
  }
}

template <typename V>
void Graph::for_each_edge(V&& visitor) const
{
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& edge : layout.edges)
    {
      visitor(edge);
    }
  }
}

template <typename V>
void Graph::for_each_station(V&& visitor) const
{
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& station : layout.stations)
    {
      visitor(station);
    }
  }
}

template <typename V>
void Graph::for_each_node_ordered(V&& visitor) const
{
  std::map<std::string, const Node*> ordered;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& node : layout.nodes)
    {
      ordered.emplace(node.node_id, &node);
    }
  }
  for (const auto& [_, n] : ordered)
  {
    visitor(*n);
  }
}

template <typename V>
void Graph::for_each_edge_ordered(V&& visitor) const
{
  std::map<std::string, const Edge*> ordered;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& edge : layout.edges)
    {
      ordered.emplace(edge.edge_id, &edge);
    }
  }
  for (const auto& [_, e] : ordered)
  {
    visitor(*e);
  }
}

template <typename V>
void Graph::for_each_station_ordered(V&& visitor) const
{
  std::map<std::string, const Station*> ordered;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& station : layout.stations)
    {
      ordered.emplace(station.station_id, &station);
    }
  }
  for (const auto& [_, s] : ordered)
  {
    visitor(*s);
  }
}

}  // namespace vda5050_core::layout

#endif  // VDA5050_CORE__LAYOUT__GRAPH_HPP_
