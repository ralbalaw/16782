#pragma once

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <optional>
#include <chrono>

#include "planalg/search/types.hpp"
#include "planalg/search/concepts.hpp"
#include "planalg/search/path.hpp"
#include "planalg/search/utility.hpp"


namespace planalg
{
namespace search
{
template <typename State, typename ForEachSuccessor, typename ComputeStepCost,
          typename ComputeStateHeuristic, typename IsGoal, typename StopCondition>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           StateHeuristicProvider<ComputeStateHeuristic, State> &&
           GoalTestProvider<IsGoal, State> && StopConditionProvider<StopCondition>
PathUPtr<State>
aStarSearch(SearchData<State>& search_data, IsGoal is_goal,
            ForEachSuccessor for_each_successor, ComputeStepCost compute_step_cost,
            ComputeStateHeuristic compute_heuristic, StopCondition stop_condition,
            double heuristic_weight = 1.0, std::optional<std::size_t> max_expansions = std::nullopt)
{
  if (heuristic_weight < 1)
  {
    throw std::invalid_argument("Heuristic Weight can't be less than 1");
  }

  ClosedSet<State>& closed_set = search_data.closed_set;
  OpenQueue<State>& open_queue = search_data.open_queue;
  PredecessorMap<State>& predecessor_map = search_data.predecessor_map;
  GValueMap<State>& g_map = search_data.g_value_map;

  int order_count = 0;
  std::size_t expanded = 0;
  while (!open_queue.empty() && (!max_expansions || expanded < *max_expansions))
  {
    const auto [state_f, _, state] = open_queue.top();
    open_queue.pop();

    if (is_goal(state))
    {
      return reconstructPath(predecessor_map, state);
    }

    if (!closed_set.insert(state).second)
    {
      continue;
    }

    ++expanded;

    const double state_g = g_map.at(state);

    for_each_successor(state, [&](State successor) {
      if (closed_set.contains(successor))
      {
        return;
      }

      const double successor_g = state_g + compute_step_cost(state, successor);
      const double successor_h = compute_heuristic(successor);
      const double successor_f = successor_g + heuristic_weight * successor_h;

      auto [it, inserted] = g_map.try_emplace(successor, successor_g);
      if (!inserted)
      {
        double& best_known_g = it->second;

        if (successor_g >= best_known_g)
        {
          return;
        }
        best_known_g = successor_g;
      }

      predecessor_map.insert_or_assign(successor, state);
      
      if (successor_h == std::numeric_limits<double>::infinity())
      {
        return;
      }

      open_queue.emplace(successor_f, ++order_count, std::move(successor));
    });

    if (stop_condition())
    {
      break;
    }
  }

  while (!open_queue.empty() && closed_set.contains(open_queue.top().state))
  {
    open_queue.pop();
  }

  return nullptr;
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost,
          typename ComputeStateHeuristic, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           StateHeuristicProvider<ComputeStateHeuristic, State> && GoalTestProvider<IsGoal, State>
PathUPtr<State>
aStarSearch(SearchData<State>& search_data, IsGoal is_goal,
            ForEachSuccessor for_each_successor, ComputeStepCost compute_step_cost,
            ComputeStateHeuristic compute_heuristic, double heuristic_weight = 1.0,
            const std::optional<std::chrono::milliseconds>& max_duration = std::nullopt,
            std::optional<std::size_t> max_expansions = std::nullopt)
{
  using Clock = std::chrono::steady_clock;

  if (max_duration)
  {
    const auto deadline = Clock::now() + *max_duration;

    return aStarSearch(
        search_data, is_goal, for_each_successor, compute_step_cost, compute_heuristic,
        [deadline]() { return Clock::now() >= deadline; }, heuristic_weight, max_expansions);
  }

  return aStarSearch(
      search_data, is_goal, for_each_successor, compute_step_cost, compute_heuristic,
      []() { return false; }, heuristic_weight, max_expansions);
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost,
          typename ComputeStateHeuristic, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           StateHeuristicProvider<ComputeStateHeuristic, State> && GoalTestProvider<IsGoal, State>
PathUPtr<State>
aStarSearch(const State& start_state, IsGoal is_goal,
            ForEachSuccessor for_each_successor, ComputeStepCost compute_step_cost,
            ComputeStateHeuristic compute_heuristic, double heuristic_weight = 1.0,
            std::optional<std::chrono::milliseconds> max_duration = std::nullopt,
            std::optional<std::size_t> max_expansions = std::nullopt)
{
  SearchData<State> search_data;
  initializeSearchData(start_state, search_data);
  return aStarSearch(search_data, is_goal, for_each_successor, compute_step_cost,
                     compute_heuristic, heuristic_weight, max_duration, max_expansions);
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost,
          typename ComputeStateHeuristic, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           StateHeuristicProvider<ComputeStateHeuristic, State> && GoalTestProvider<IsGoal, State>
PathUPtr<State>
aStarSearch(const StateSet<State>& start_set, IsGoal is_goal,
            ForEachSuccessor for_each_successor, ComputeStepCost compute_step_cost,
            ComputeStateHeuristic compute_heuristic, double heuristic_weight = 1.0,
            std::optional<std::chrono::milliseconds> max_duration = std::nullopt,
            std::optional<std::size_t> max_expansions = std::nullopt)
{
  SearchData<State> search_data;
  initializeSearchData(start_set, search_data);
  return aStarSearch(search_data, is_goal, for_each_successor, compute_step_cost,
                     compute_heuristic, heuristic_weight, max_duration, max_expansions);
}

}  // namespace search
}  // namespace planalg