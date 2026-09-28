#include "planalg/graph/graph.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "planalg/graph/types.hpp"

namespace planalg
{
namespace graph
{
namespace
{
#ifndef PLANNER_DATA_DIR
#define PLANNER_DATA_DIR "."
#endif

using Json = nlohmann::json;

std::filesystem::path getJsonPath(const Map map)
{
  switch (map)
  {
    case Map::kRomanian:
    {
      return std::filesystem::path{ PLANNER_DATA_DIR } / "romania_graph.json";
    }
    case Map::kAtlanta:
    {
      return std::filesystem::path{ PLANNER_DATA_DIR } / "atlanta_osm.json";
    }
  }

  throw std::invalid_argument{ "Unknown map" };
}

Json readJsonFile(const std::filesystem::path& path)
{
  std::ifstream file{ path };
  if (!file)
  {
    throw std::runtime_error{ "Failed to open graph JSON file: " + path.string() };
  }

  return Json::parse(file);
}

AdjacencySpec toAdjacencySpec(const Json& data)
{
  AdjacencySpec adjacency_spec;

  for (const Json& node : data.at("nodes"))
  {
    adjacency_spec.try_emplace(node.at("key").get<VertexKey>());
  }

  const bool directed = data.at("directed").get<bool>();

  for (const Json& edge : data.at("edges"))
  {
    const VertexKey source = edge.at("source").get<VertexKey>();
    const VertexKey target = edge.at("target").get<VertexKey>();
    const double weight = edge.at("attributes").at("weight").get<double>();

    adjacency_spec[source].push_back(EdgeSpec{ target, weight });
    adjacency_spec.try_emplace(target);

    if (!directed && source != target)
    {
      adjacency_spec[target].push_back(EdgeSpec{ source, weight });
    }
  }

  return adjacency_spec;
}
}  // namespace

Graph::Graph(const AdjacencySpec& adjacency_spec)
{
  buildVertexMappings(adjacency_spec);
  buildAdjacencyList(adjacency_spec);
}

void Graph::buildAdjacencyList(const AdjacencySpec& adjacency_spec)
{
  adjacency_list_.clear();
  adjacency_list_.resize(vertex_to_key_.size());

  for (const auto& [vertex_key, edge_specs] : adjacency_spec)
  {
    const VertexIndex vertex_index = key_to_vertex_.at(vertex_key);
    EdgeList& edge_list = adjacency_list_.at(vertex_index);

    edge_list.reserve(edge_specs.size());

    for (const EdgeSpec& edge_spec : edge_specs)
    {
      const VertexIndex target_index = key_to_vertex_.at(edge_spec.target);
      edge_list.push_back(Edge{ target_index, edge_spec.weight });
    }

    std::ranges::sort(edge_list, {}, &Edge::target);
  }
}

void Graph::buildVertexMappings(const AdjacencySpec& adjacency_spec)
{
  vertex_to_key_.clear();
  key_to_vertex_.clear();

  std::size_t key_count = adjacency_spec.size();

  for (const auto& entry : adjacency_spec)
  {
    key_count += entry.second.size();
  }

  vertex_to_key_.reserve(key_count);

  for (const auto& [vertex_key, edge_specs] : adjacency_spec)
  {
    vertex_to_key_.push_back(vertex_key);

    for (const EdgeSpec& edge_spec : edge_specs)
    {
      vertex_to_key_.push_back(edge_spec.target);
    }
  }

  std::ranges::sort(vertex_to_key_);

  const auto duplicate_tail = std::ranges::unique(vertex_to_key_);
  vertex_to_key_.erase(duplicate_tail.begin(), duplicate_tail.end());

  key_to_vertex_.reserve(vertex_to_key_.size());

  for (VertexIndex vertex = 0; vertex < vertex_to_key_.size(); ++vertex)
  {
    key_to_vertex_.emplace(vertex_to_key_[vertex], vertex);
  }
}

Graph Graph::fromJson(const Map& map)
{
  const Json json = readJsonFile(getJsonPath(map));
  return Graph{ toAdjacencySpec(json) };
}
}  // namespace graph
}  // namespace planalg
