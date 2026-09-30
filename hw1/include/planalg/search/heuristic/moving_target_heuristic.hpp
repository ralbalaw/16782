#pragma once

#include <algorithm>
#include <limits>
#include "planalg/search/heuristic/distance_metrics.hpp"
#include <span>

namespace planalg
{
namespace search
{
namespace heuristic
{
template <typename Predicate>
int binarySearchFirstTrue(int low, int high, Predicate predicate)
{
  if (low > high)
  {
    return low;
  }

  if (low == high)
  {
    return predicate(low) ? low : low + 1;
  }

  const int idx = low + (high - low) / 2;

  if (predicate(idx))
  {
    return binarySearchFirstTrue(low, idx, predicate);
  }

  return binarySearchFirstTrue(idx + 1, high, predicate);
}

template <typename State>
double movingTargetHeuristic(const State& state, std::span<const State> trajectory)
{
  const double inf = std::numeric_limits<double>::infinity();
  if (trajectory.empty())
  {
    return inf;
  }

  const int current_time = state.t;
  const int target_count = static_cast<int>(trajectory.size());

  auto predicate = [&](int trajectory_index) {
    const State& target_state = trajectory[trajectory_index];
    const double heuristic = chebyshevDistance(state, target_state);
    const int dt = target_state.t - current_time;
    return dt >= heuristic;
  };

  const int low = std::max(0, current_time - trajectory.front().t);
  if (low >= target_count)
  {
    return inf;
  }

  const int trajectory_index = binarySearchFirstTrue(low, target_count - 1, predicate);
  if (trajectory_index < target_count)
  {
    return trajectory[trajectory_index].t - current_time;
  }

  return inf;
}
}  // namespace heuristic
}  // namespace search
}  // namespace planalg
