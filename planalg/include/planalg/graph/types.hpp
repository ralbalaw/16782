#pragma once

#include <span>
#include <string>
#include <vector>
#include <unordered_map>

namespace planalg
{
namespace graph
{
using VertexKey = std::string;
using VertexIndex = std::size_t;

struct Edge
{
  VertexIndex target;
  double weight;
};

struct EdgeSpec
{
  VertexKey target;
  double weight;
};

using EdgeList = std::vector<Edge>;
using EdgeListView = std::span<const Edge>;
using AdjacencySpec = std::unordered_map<VertexKey, std::vector<EdgeSpec>>;

using Path = std::vector<VertexIndex>;
using KeyPath = std::vector<VertexKey>;
}  // namespace graph
}  // namespace planalg
