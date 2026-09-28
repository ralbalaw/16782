#pragma once

#include <cstddef>
#include <vector>
#include <unordered_map>

#include "planalg/graph/types.hpp"

namespace planalg
{
namespace graph
{
enum class Map
{
  kRomanian,
  kAtlanta,
};

class Graph
{
public:
  Graph() = default;
  Graph(const AdjacencySpec& adjacency_spec);

  static Graph fromJson(const Map& map);

  EdgeListView getOutgoingEdges(VertexIndex vertex) const
  { return EdgeListView{ adjacency_list_.at(vertex) }; }

  const VertexKey& getVertexKey(VertexIndex vertex) const
  { return vertex_to_key_.at(vertex); }

  VertexIndex getVertexIndex(const VertexKey& key) const
  { return key_to_vertex_.at(key); }

  std::size_t getVertexCount() const
  { return adjacency_list_.size(); }

private:
  void buildVertexMappings(const AdjacencySpec& adjacency_spec);
  void buildAdjacencyList(const AdjacencySpec& adjacency_spec);

  std::vector<EdgeList> adjacency_list_;
  std::unordered_map<VertexKey, VertexIndex> key_to_vertex_;
  std::vector<VertexKey> vertex_to_key_;
};
}  // namespace graph
}  // namespace planalg
