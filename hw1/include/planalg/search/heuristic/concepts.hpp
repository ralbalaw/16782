#pragma once

#include <concepts>
#include <vector>

namespace planalg
{
namespace search
{
namespace heuristic
{
template <typename ComputeTargetsHeuristic, typename State>
concept TargetsHeuristicProvider =
    requires(ComputeTargetsHeuristic compute_heuristic, const State& state, const std::vector<State>& targets) {
      { compute_heuristic(state, targets) } -> std::convertible_to<double>;
    };

}  // namespace heuristic
}  // namespace search
}  // namespace planalg
