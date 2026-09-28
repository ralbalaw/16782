#pragma once

#include <cmath>
#include <cstddef>
#include <utility>

#include "planalg/search/concepts.hpp"
#include "planalg/search/real_time_heuristic_search.hpp"

namespace planalg
{
namespace search
{

template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State>
class RealTimeAdaptiveAStar
  : public RealTimeHeuristicSearch<State, ForEachSuccessor, ComputeStepCost, IsGoal>
{
  using Base = RealTimeHeuristicSearch<State, ForEachSuccessor, ComputeStepCost, IsGoal>;

public:
  RealTimeAdaptiveAStar(IsGoal is_goal, ForEachSuccessor for_each_successor,
                        ComputeStepCost compute_step_cost,
                        HeuristicManagerUPtr<State> heuristic_manager,
                        std::size_t max_expansions = 100, double heuristic_weight = 1.0)
    : Base(std::move(is_goal), std::move(for_each_successor), std::move(compute_step_cost),
           std::move(heuristic_manager), max_expansions, heuristic_weight)
  {
  }

private:
  double computeOrLookupHeuristic(const State& state) override
  { return this->heuristicManager()->computeOrLookupHeuristic(state); }

  void learnHeuristic(const TimeLimit& time_limit) override
  {
    const auto& closed_set = this->closedSet();
    if (closed_set.empty())
    {
      return;
    }
    const auto& g_value_map = this->gValueMap();

    double f_min = 0.0;
    if (!this->currentPath())
    {
      auto& open_queue = this->openQueue();
      if (open_queue.empty())
      {
        return;
      }
      const auto entry = open_queue.top();
      f_min = entry.priority;
    }
    else
    {
      f_min = g_value_map.at(this->currentPath()->back());
    }

    for (const State& state : closed_set)
    {
      if (time_limit.expired())
      {
        return;
      }

      const double g = g_value_map.at(state);
      const double h = f_min - g;
      this->heuristicManager()->updateHeuristicValue(state, h);
    }
  }

  void performAdditionalLearning(const State& current_state,
                                 const TimeLimit& time_limit) override
  { this->heuristicManager()->learn(current_state, time_limit); }

  double heuristic_weight = 1.0;
};
}  // namespace search
}  // namespace planalg
