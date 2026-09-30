#pragma once

#include <concepts>

namespace planalg
{
namespace search
{
template <typename ForEachSuccessor, typename State>
concept SuccessorProvider = requires(ForEachSuccessor for_each_successor, const State& state) {
  for_each_successor(state, [](State) {});
};

template <typename ComputeStepCost, typename State>
concept StepCostProvider =
    requires(ComputeStepCost compute_step_cost, const State& from, const State& to) {
      { compute_step_cost(from, to) } -> std::convertible_to<double>;
    };

template <typename ComputeTransitionHeuristic, typename State>
concept TransitionHeuristicProvider =
    requires(ComputeTransitionHeuristic compute_heuristic, const State& from, const State& to) {
      { compute_heuristic(from, to) } -> std::convertible_to<double>;
    };

template <typename ComputeStateHeuristic, typename State>
concept StateHeuristicProvider =
    requires(ComputeStateHeuristic compute_heuristic, const State& state) {
      { compute_heuristic(state) } -> std::convertible_to<double>;
    };

template <typename IsGoal, typename State>
concept GoalTestProvider = requires(IsGoal is_goal, const State& state) {
  { is_goal(state) } -> std::convertible_to<bool>;
};

template <typename StopCondition>
concept StopConditionProvider = requires(StopCondition stop_condition) {
  { stop_condition() } -> std::convertible_to<bool>;
};
}  // namespace search
}  // namespace planalg
