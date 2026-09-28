#pragma once

#include <algorithm>
#include <memory>
#include <utility>

#include "planalg/search/types.hpp"

namespace planalg
{
namespace search
{
template <typename State>
PathUPtr<State> reconstructPath(const PredecessorMap<State>& predecessor_map,
                                const State& goal_state)
{
  auto path = std::make_unique<Path<State>>();
  const State* current = &goal_state;

  while (current != nullptr)
  {
    path->emplace_back(*current);

    auto it = predecessor_map.find(*current);

    if (it != predecessor_map.end())
    {
      current = &it->second;
      continue;
    }
    current = nullptr;
  }

  std::reverse(path->begin(), path->end());

  return std::move(path);
}
}  // namespace search
}  // namespace planalg