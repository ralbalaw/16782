#pragma once

#include <algorithm>
#include <utility>

#include "planalg/search/types.hpp"
#include "planalg/search/concepts.hpp"
#include "planalg/search/path.hpp"

namespace planalg
{
namespace search
{

template <typename State, typename ForEachSuccessor, typename IsGoal, typename StopCondition>
  requires SuccessorProvider<ForEachSuccessor, State> && GoalTestProvider<IsGoal, State> &&
           StopConditionProvider<StopCondition>
PathUPtr<State> breadthFirstSearch(BfsSearchData<State>& search_data, IsGoal is_goal,
                                   ForEachSuccessor for_each_successor,
                                   StopCondition stop_condition,
                                   std::optional<std::size_t> max_expansions = std::nullopt)
{
  ClosedMap<State>& closed_map = search_data.closed_map;
  OpenFifoQueue<State>& open_queue = search_data.open_queue;
  PredecessorMap<State>& predecessor_map = search_data.predecessor_map;

  std::size_t expanded = 0;

  while (!open_queue.empty() && (!max_expansions || expanded < *max_expansions))
  {
    const State state = std::move(open_queue.front());
    open_queue.pop();

    if (is_goal(state))
    {
      return reconstructPath(predecessor_map, state);
    }

    const std::size_t state_steps = closed_map.at(state);

    for_each_successor(state, [&](const State& successor) {
      auto [_, inserted] = closed_map.try_emplace(successor, state_steps + 1);

      if (!inserted)
      {
        return;
      }

      predecessor_map.emplace(successor, state);
      open_queue.push(successor);
    });

    ++expanded;

    if (stop_condition())
    {
      break;
    }
  }

  return nullptr;
}

template <typename State, typename ForEachSuccessor, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && GoalTestProvider<IsGoal, State>
PathUPtr<State>
breadthFirstSearch(BfsSearchData<State>& search_data, IsGoal is_goal,
                   ForEachSuccessor for_each_successor,
                   const std::optional<std::chrono::milliseconds>& max_duration = std::nullopt,
                   std::optional<std::size_t> max_expansions = std::nullopt)
{
  using Clock = std::chrono::steady_clock;

  if (max_duration)
  {
    const auto deadline = Clock::now() + *max_duration;

    return breadthFirstSearch(
        search_data, is_goal, for_each_successor, [deadline]() { return Clock::now() >= deadline; },
        max_expansions);
  }

  return breadthFirstSearch(
      search_data, is_goal, for_each_successor, []() { return false; }, max_expansions);
}

template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State>
PathUPtr<State>
breadthFirstSearch(const State& start_state, IsGoal is_goal, ForEachSuccessor for_each_successor,
                   std::optional<std::chrono::milliseconds> max_duration = std::nullopt,
                   std::optional<std::size_t> max_expansions = std::nullopt)
{
  BfsSearchData<State> search_data;
  initializeBfsSearchData(start_state, search_data);
  return breadthFirstSearch(search_data, is_goal, for_each_successor, max_duration, max_expansions);
}

template <typename State, typename ForEachSuccessor, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && GoalTestProvider<IsGoal, State>
PathUPtr<State>
breadthFirstSearch(const StateSet<State>& start_set, IsGoal is_goal,
                   ForEachSuccessor for_each_successor,
                   std::optional<std::chrono::milliseconds> max_duration = std::nullopt,
                   std::optional<std::size_t> max_expansions = std::nullopt)
{
  BfsSearchData<State> search_data;
  initializeBfsSearchData(start_set, search_data);
  return breadthFirstSearch(search_data, is_goal, for_each_successor, max_duration, max_expansions);
}

}  // namespace search
}  // namespace planalg