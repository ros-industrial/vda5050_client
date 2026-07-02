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

#include "vda5050_core/layout/graph.hpp"

#include <algorithm>
#include <map>
#include <ostream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "vda5050_core/layout/validate_layout.hpp"

namespace vda5050_core::layout {

Graph::Ptr Graph::from_lif(LIF lif)
{
  auto errors = validate_layout(lif);
  if (!errors.empty())
  {
    throw std::invalid_argument(
      "Graph::from_lif: " + errors.front().description);
  }
  auto g = Ptr(new Graph());
  g->lif_ = std::move(lif);
  g->rebuild_indices();
  return g;
}

void Graph::rebuild_indices()
{
  layout_index_.clear();
  node_index_.clear();
  edge_index_.clear();
  station_index_.clear();
  node_outbound_.clear();
  node_inbound_.clear();
  node_stations_.clear();

  for (std::size_t li = 0; li < lif_.layouts.size(); ++li)
  {
    const auto& layout = lif_.layouts[li];
    layout_index_.emplace(layout.layout_id, li);
    for (std::size_t ni = 0; ni < layout.nodes.size(); ++ni)
    {
      const auto& node = layout.nodes[ni];
      node_index_.emplace(node.node_id, std::make_pair(li, ni));
      // Ensure adjacency entries exist (empty for isolated nodes).
      node_outbound_[node.node_id];
      node_inbound_[node.node_id];
    }
    for (std::size_t ei = 0; ei < layout.edges.size(); ++ei)
    {
      const auto& edge = layout.edges[ei];
      edge_index_.emplace(edge.edge_id, std::make_pair(li, ei));
      node_outbound_[edge.start_node_id].push_back(edge.edge_id);
      node_inbound_[edge.end_node_id].push_back(edge.edge_id);
    }
    for (std::size_t si = 0; si < layout.stations.size(); ++si)
    {
      const auto& station = layout.stations[si];
      station_index_.emplace(station.station_id, std::make_pair(li, si));
      for (const auto& nid : station.interaction_node_ids)
      {
        node_stations_[nid].push_back(station.station_id);
      }
    }
  }
}

bool Graph::has_node(const std::string& id) const noexcept
{
  return node_index_.find(id) != node_index_.end();
}

bool Graph::has_edge(const std::string& id) const noexcept
{
  return edge_index_.find(id) != edge_index_.end();
}

bool Graph::has_station(const std::string& id) const noexcept
{
  return station_index_.find(id) != station_index_.end();
}

const Node& Graph::get_node(const std::string& id) const
{
  const auto* n = find_node(id);
  if (n == nullptr)
  {
    throw std::out_of_range("Graph::get_node: id '" + id + "' not found");
  }
  return *n;
}

const Edge& Graph::get_edge(const std::string& id) const
{
  const auto* e = find_edge(id);
  if (e == nullptr)
  {
    throw std::out_of_range("Graph::get_edge: id '" + id + "' not found");
  }
  return *e;
}

const Station& Graph::get_station(const std::string& id) const
{
  const auto* s = find_station(id);
  if (s == nullptr)
  {
    throw std::out_of_range("Graph::get_station: id '" + id + "' not found");
  }
  return *s;
}

const Node* Graph::find_node(const std::string& id) const noexcept
{
  auto it = node_index_.find(id);
  if (it == node_index_.end()) return nullptr;
  const auto [li, ni] = it->second;
  return &lif_.layouts[li].nodes[ni];
}

const Edge* Graph::find_edge(const std::string& id) const noexcept
{
  auto it = edge_index_.find(id);
  if (it == edge_index_.end()) return nullptr;
  const auto [li, ei] = it->second;
  return &lif_.layouts[li].edges[ei];
}

const Station* Graph::find_station(const std::string& id) const noexcept
{
  auto it = station_index_.find(id);
  if (it == station_index_.end()) return nullptr;
  const auto [li, si] = it->second;
  return &lif_.layouts[li].stations[si];
}

const Layout* Graph::find_layout(const std::string& layout_id) const noexcept
{
  auto it = layout_index_.find(layout_id);
  if (it == layout_index_.end()) return nullptr;
  return &lif_.layouts[it->second];
}

const std::vector<std::string>& Graph::outbound_edges(
  const std::string& node_id) const
{
  auto it = node_outbound_.find(node_id);
  if (it == node_outbound_.end())
  {
    throw std::out_of_range(
      "Graph::outbound_edges: node '" + node_id + "' not found");
  }
  return it->second;
}

const std::vector<std::string>& Graph::inbound_edges(
  const std::string& node_id) const
{
  auto it = node_inbound_.find(node_id);
  if (it == node_inbound_.end())
  {
    throw std::out_of_range(
      "Graph::inbound_edges: node '" + node_id + "' not found");
  }
  return it->second;
}

std::vector<std::string> Graph::edges_between(
  const std::string& from_node_id, const std::string& to_node_id) const
{
  std::vector<std::string> out;
  const auto& outbound = outbound_edges(from_node_id);
  for (const auto& edge_id : outbound)
  {
    if (get_edge(edge_id).end_node_id == to_node_id)
    {
      out.push_back(edge_id);
    }
  }
  return out;
}

std::size_t Graph::node_count() const noexcept
{
  return node_index_.size();
}

std::size_t Graph::edge_count() const noexcept
{
  return edge_index_.size();
}

std::size_t Graph::station_count() const noexcept
{
  return station_index_.size();
}

bool Graph::empty() const noexcept
{
  return node_index_.empty() && edge_index_.empty() && station_index_.empty();
}

std::vector<std::string> Graph::unconnected_nodes() const
{
  // Collect station-referenced node IDs first (for O(1) lookup below).
  std::unordered_set<std::string> station_refs;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& station : layout.stations)
    {
      for (const auto& nid : station.interaction_node_ids)
      {
        station_refs.insert(nid);
      }
    }
  }

  std::vector<std::string> result;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& node : layout.nodes)
    {
      const auto& out = node_outbound_.at(node.node_id);
      const auto& in = node_inbound_.at(node.node_id);
      if (
        out.empty() && in.empty() &&
        station_refs.find(node.node_id) == station_refs.end())
      {
        result.push_back(node.node_id);
      }
    }
  }
  return result;
}

namespace {

using PosIndex =
  std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>;

// Remove the first occurrence of `value` from `vec`.
void vec_erase_value(std::vector<std::string>& vec, const std::string& value)
{
  auto it = std::find(vec.begin(), vec.end(), value);
  if (it != vec.end()) vec.erase(it);
}

// Replace every occurrence of `from` with `to` in `vec`.
void vec_replace_value(
  std::vector<std::string>& vec, const std::string& from, const std::string& to)
{
  std::replace(vec.begin(), vec.end(), from, to);
}

// Move the entry under `from` to `to`, preserving its value (no-op if `from`
// is absent). `to` must not already be present.
template <typename Map>
void rekey(Map& map, const std::string& from, const std::string& to)
{
  auto it = map.find(from);
  if (it == map.end()) return;
  map.emplace(to, std::move(it->second));
  map.erase(it);
}

// Remove element at `pos` from `vec` by swap-and-pop (O(1), reorders) and keep
// `index` consistent: drop the removed element's entry, and if a different
// element was swapped into `pos`, repoint its entry to (li, pos).
template <typename T, typename IdFn>
void erase_swap(
  std::vector<T>& vec, std::size_t pos, std::size_t li, PosIndex& index,
  IdFn id_of)
{
  index.erase(id_of(vec[pos]));
  const std::size_t last = vec.size() - 1;
  if (pos != last)
  {
    vec[pos] = std::move(vec[last]);
    index[id_of(vec[pos])] = std::make_pair(li, pos);
  }
  vec.pop_back();
}

}  // namespace

void Graph::add_node(Node node, const std::string& layout_id)
{
  if (has_node(node.node_id))
  {
    throw std::invalid_argument(
      "Graph::add_node: duplicate nodeId '" + node.node_id + "'");
  }
  auto lit = layout_index_.find(layout_id);
  if (lit == layout_index_.end())
  {
    throw std::invalid_argument(
      "Graph::add_node: layout '" + layout_id + "' not found");
  }
  const std::size_t li = lit->second;
  lif_.layouts[li].nodes.push_back(std::move(node));
  auto& nodes = lif_.layouts[li].nodes;
  const std::size_t ni = nodes.size() - 1;
  const std::string& nid = nodes[ni].node_id;
  node_index_.emplace(nid, std::make_pair(li, ni));
  // Ensure adjacency entries exist (empty for an isolated node).
  node_outbound_[nid];
  node_inbound_[nid];
}

void Graph::add_edge(Edge edge, const std::string& layout_id)
{
  if (has_edge(edge.edge_id))
  {
    throw std::invalid_argument(
      "Graph::add_edge: duplicate edgeId '" + edge.edge_id + "'");
  }
  auto lit = layout_index_.find(layout_id);
  if (lit == layout_index_.end())
  {
    throw std::invalid_argument(
      "Graph::add_edge: layout '" + layout_id + "' not found");
  }
  const std::size_t li = lit->second;
  // startNodeId must be in the specified layout.
  auto sit = node_index_.find(edge.start_node_id);
  if (sit == node_index_.end() || sit->second.first != li)
  {
    throw std::invalid_argument(
      "Graph::add_edge: startNodeId '" + edge.start_node_id +
      "' not in layout '" + layout_id + "'");
  }
  // endNodeId can be in any layout (cross-layout elevator transitions).
  if (!has_node(edge.end_node_id))
  {
    throw std::invalid_argument(
      "Graph::add_edge: endNodeId '" + edge.end_node_id +
      "' does not exist in any layout");
  }
  const std::string start = edge.start_node_id;
  const std::string end = edge.end_node_id;
  lif_.layouts[li].edges.push_back(std::move(edge));
  auto& edges = lif_.layouts[li].edges;
  const std::size_t ei = edges.size() - 1;
  const std::string& eid = edges[ei].edge_id;
  edge_index_.emplace(eid, std::make_pair(li, ei));
  node_outbound_[start].push_back(eid);
  node_inbound_[end].push_back(eid);
}

void Graph::add_station(Station station, const std::string& layout_id)
{
  if (has_station(station.station_id))
  {
    throw std::invalid_argument(
      "Graph::add_station: duplicate stationId '" + station.station_id + "'");
  }
  auto lit = layout_index_.find(layout_id);
  if (lit == layout_index_.end())
  {
    throw std::invalid_argument(
      "Graph::add_station: layout '" + layout_id + "' not found");
  }
  const std::size_t li = lit->second;
  for (const auto& nid : station.interaction_node_ids)
  {
    if (!has_node(nid))
    {
      throw std::invalid_argument(
        "Graph::add_station: interactionNodeId '" + nid +
        "' does not exist in any layout");
    }
  }
  lif_.layouts[li].stations.push_back(std::move(station));
  auto& stations = lif_.layouts[li].stations;
  const std::size_t si = stations.size() - 1;
  const std::string& sid = stations[si].station_id;
  station_index_.emplace(sid, std::make_pair(li, si));
  for (const auto& nid : stations[si].interaction_node_ids)
  {
    node_stations_[nid].push_back(sid);
  }
}

void Graph::delete_node(const std::string& id)
{
  if (!has_node(id))
  {
    throw std::invalid_argument(
      "Graph::delete_node: nodeId '" + id + "' not found");
  }
  // A self-loop edge appears in both adjacency lists, so dedupe the list.
  const auto& out = node_outbound_.at(id);
  const auto& in = node_inbound_.at(id);
  if (!out.empty() || !in.empty())
  {
    std::vector<std::string> ref_edges;
    std::unordered_set<std::string> seen;
    for (const auto& e : out)
    {
      if (seen.insert(e).second) ref_edges.push_back(e);
    }
    for (const auto& e : in)
    {
      if (seen.insert(e).second) ref_edges.push_back(e);
    }
    std::string list;
    for (const auto& e : ref_edges)
    {
      list += (list.empty() ? "" : ", ") + e;
    }
    throw std::invalid_argument(
      "Graph::delete_node: nodeId '" + id +
      "' is referenced by edges: " + list);
  }
  // Check no station references it.
  auto sit = node_stations_.find(id);
  if (sit != node_stations_.end() && !sit->second.empty())
  {
    std::string list;
    for (const auto& s : sit->second)
    {
      list += (list.empty() ? "" : ", ") + s;
    }
    throw std::invalid_argument(
      "Graph::delete_node: nodeId '" + id +
      "' is referenced by stations: " + list);
  }

  const auto [li, ni] = node_index_.at(id);
  auto& nodes = lif_.layouts[li].nodes;
  erase_swap(
    nodes, ni, li, node_index_, [](const Node& n) { return n.node_id; });
  node_outbound_.erase(id);
  node_inbound_.erase(id);
  node_stations_.erase(id);
}

void Graph::delete_edge(const std::string& id)
{
  if (!has_edge(id))
  {
    throw std::invalid_argument(
      "Graph::delete_edge: edgeId '" + id + "' not found");
  }
  const auto [li, ei] = edge_index_.at(id);
  auto& edges = lif_.layouts[li].edges;
  // Capture endpoints before the swap overwrites this slot.
  const std::string start = edges[ei].start_node_id;
  const std::string end = edges[ei].end_node_id;
  vec_erase_value(node_outbound_[start], id);
  vec_erase_value(node_inbound_[end], id);
  erase_swap(
    edges, ei, li, edge_index_, [](const Edge& e) { return e.edge_id; });
}

void Graph::delete_station(const std::string& id)
{
  if (!has_station(id))
  {
    throw std::invalid_argument(
      "Graph::delete_station: stationId '" + id + "' not found");
  }
  const auto [li, si] = station_index_.at(id);
  auto& stations = lif_.layouts[li].stations;
  for (const auto& nid : stations[si].interaction_node_ids)
  {
    auto it = node_stations_.find(nid);
    if (it != node_stations_.end()) vec_erase_value(it->second, id);
  }
  erase_swap(stations, si, li, station_index_, [](const Station& s) {
    return s.station_id;
  });
}

void Graph::update_node_id(const std::string& old_id, const std::string& new_id)
{
  if (!has_node(old_id))
  {
    throw std::invalid_argument(
      "Graph::update_node_id: nodeId '" + old_id + "' not found");
  }
  if (old_id == new_id) return;
  if (has_node(new_id))
  {
    throw std::invalid_argument(
      "Graph::update_node_id: new nodeId '" + new_id + "' already exists");
  }
  // Rewrite the node itself.
  const auto [li, ni] = node_index_.at(old_id);
  lif_.layouts[li].nodes[ni].node_id = new_id;
  // Rewrite the edges incident to old_id via the adjacency lists (global keys,
  // so cross-layout edges and self-loops are covered).
  for (const auto& eid : node_outbound_.at(old_id))
  {
    const auto [eli, eei] = edge_index_.at(eid);
    lif_.layouts[eli].edges[eei].start_node_id = new_id;
  }
  for (const auto& eid : node_inbound_.at(old_id))
  {
    const auto [eli, eei] = edge_index_.at(eid);
    lif_.layouts[eli].edges[eei].end_node_id = new_id;
  }
  // Rewrite station references via the reverse index.
  auto sit = node_stations_.find(old_id);
  if (sit != node_stations_.end())
  {
    for (const auto& station_id : sit->second)
    {
      const auto [sli, ssi] = station_index_.at(station_id);
      for (auto& nid : lif_.layouts[sli].stations[ssi].interaction_node_ids)
      {
        if (nid == old_id) nid = new_id;
      }
    }
  }
  // Adjacency entries exist for every node, so rekey them even when empty.
  rekey(node_index_, old_id, new_id);
  rekey(node_outbound_, old_id, new_id);
  rekey(node_inbound_, old_id, new_id);
  rekey(node_stations_, old_id, new_id);
}

void Graph::update_edge_id(const std::string& old_id, const std::string& new_id)
{
  if (!has_edge(old_id))
  {
    throw std::invalid_argument(
      "Graph::update_edge_id: edgeId '" + old_id + "' not found");
  }
  if (old_id == new_id) return;
  if (has_edge(new_id))
  {
    throw std::invalid_argument(
      "Graph::update_edge_id: new edgeId '" + new_id + "' already exists");
  }
  const auto [li, ei] = edge_index_.at(old_id);
  auto& edge = lif_.layouts[li].edges[ei];
  edge.edge_id = new_id;
  vec_replace_value(node_outbound_[edge.start_node_id], old_id, new_id);
  vec_replace_value(node_inbound_[edge.end_node_id], old_id, new_id);
  rekey(edge_index_, old_id, new_id);
}

void Graph::update_station_id(
  const std::string& old_id, const std::string& new_id)
{
  if (!has_station(old_id))
  {
    throw std::invalid_argument(
      "Graph::update_station_id: stationId '" + old_id + "' not found");
  }
  if (old_id == new_id) return;
  if (has_station(new_id))
  {
    throw std::invalid_argument(
      "Graph::update_station_id: new stationId '" + new_id +
      "' already exists");
  }
  const auto [li, si] = station_index_.at(old_id);
  auto& station = lif_.layouts[li].stations[si];
  station.station_id = new_id;
  for (const auto& nid : station.interaction_node_ids)
  {
    auto it = node_stations_.find(nid);
    if (it != node_stations_.end())
      vec_replace_value(it->second, old_id, new_id);
  }
  rekey(station_index_, old_id, new_id);
}

std::vector<std::string> Graph::prune()
{
  auto victims = unconnected_nodes();
  for (const auto& id : victims)
  {
    delete_node(id);
  }
  return victims;
}

bool Graph::operator==(const Graph& rhs) const
{
  return lif_ == rhs.lif_;
}

bool Graph::operator!=(const Graph& rhs) const
{
  return !(*this == rhs);
}

void Graph::dump_dot(std::ostream& os) const
{
  os << "digraph G {\n";
  // Deterministic ordering for stable output (tests + diffability).
  std::map<std::string, const Node*> ordered_nodes;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& n : layout.nodes)
    {
      ordered_nodes.emplace(n.node_id, &n);
    }
  }
  for (const auto& [id, _] : ordered_nodes)
  {
    os << "  \"" << id << "\";\n";
  }
  std::map<std::string, const Edge*> ordered_edges;
  for (const auto& layout : lif_.layouts)
  {
    for (const auto& e : layout.edges)
    {
      ordered_edges.emplace(e.edge_id, &e);
    }
  }
  for (const auto& [id, ep] : ordered_edges)
  {
    os << "  \"" << ep->start_node_id << "\" -> \"" << ep->end_node_id
       << "\" [label=\"" << id << "\"];\n";
  }
  os << "}\n";
}

}  // namespace vda5050_core::layout
