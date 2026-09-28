#pragma once

#include <algorithm>
#include <chrono>
#include <optional>
#include <utility>

#include "planalg/search/types.hpp"
#include "planalg/search/concepts.hpp"
#include "planalg/search/path.hpp"

namespace planalg
{
namespace search
{
template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal,
          typename StopCondition>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State> && StopConditionProvider<StopCondition>
PathUPtr<State> dijkstraSearch(SearchData<State>& search_data, IsGoal is_goal,
                               ForEachSuccessor for_each_successor,
                               ComputeStepCost compute_step_cost, StopCondition stop_condition,
                               std::optional<std::size_t> max_expansions = std::nullopt)
{
  ClosedSet<State>& closed_set = search_data.closed_set;
  OpenQueue<State>& open_queue = search_data.open_queue;
  PredecessorMap<State>& predecessor_map = search_data.predecessor_map;
  GValueMap<State>& g_map = search_data.g_value_map;

  int order_count = 0;
  std::size_t expanded = 0;

  while (!open_queue.empty() && (!max_expansions || expanded < *max_expansions))
  {
    const auto [state_g, _, state] = open_queue.top();
    open_queue.pop();

    if (!closed_set.insert(state).second)
    {
      continue;
    }

    if (is_goal(state))
    {
      return reconstructPath(predecessor_map, state);
    }

    ++expanded;

    for_each_successor(state, [&](State successor) {
      if (closed_set.contains(successor))
      {
        return;
      }

      const double successor_g = state_g + compute_step_cost(state, successor);

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

      open_queue.emplace(successor_g, ++order_count, std::move(successor));
    });

    if (stop_condition())
    {
      break;
    }
  }

  return nullptr;
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State>
PathUPtr<State>
dijkstraSearch(SearchData<State>& search_data, IsGoal is_goal, ForEachSuccessor for_each_successor,
               ComputeStepCost compute_step_cost,
               const std::optional<std::chrono::milliseconds>& max_duration = std::nullopt,
               std::optional<std::size_t> max_expansions = std::nullopt)
{
  using Clock = std::chrono::steady_clock;

  if (max_duration)
  {
    const auto deadline = Clock::now() + *max_duration;

    return dijkstraSearch(
        search_data, is_goal, for_each_successor, compute_step_cost,
        [deadline]() { return Clock::now() >= deadline; }, max_expansions);
  }

  return dijkstraSearch(
      search_data, is_goal, for_each_successor, compute_step_cost, []() { return false; },
      max_expansions);
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State>
PathUPtr<State> dijkstraSearch(const State& start_state, IsGoal is_goal,
                               ForEachSuccessor for_each_successor,
                               ComputeStepCost compute_step_cost,
                               std::optional<std::chrono::milliseconds> max_duration = std::nullopt,
                               std::optional<std::size_t> max_expansions = std::nullopt)
{
  SearchData<State> search_data;
  initializeSearchData(start_state, search_data);
  return dijkstraSearch(search_data, is_goal, for_each_successor, compute_step_cost, max_duration,
                        max_expansions);
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State>
PathUPtr<State> dijkstraSearch(const StateSet<State>& start_set, IsGoal is_goal,
                               ForEachSuccessor for_each_successor,
                               ComputeStepCost compute_step_cost,
                               std::optional<std::chrono::milliseconds> max_duration = std::nullopt,
                               std::optional<std::size_t> max_expansions = std::nullopt)
{
  SearchData<State> search_data;
  initializeSearchData(start_set, search_data);
  return dijkstraSearch(search_data, is_goal, for_each_successor, compute_step_cost, max_duration,
                        max_expansions);
}
}  // namespace search
}  // namespace planalg
