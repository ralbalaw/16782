#pragma once

#include <cstddef>
#include <utility>

#include "planalg/search/types.hpp"
#include "planalg/search/concepts.hpp"
#include "planalg/search/dijkstra_search.hpp"
#include "planalg/search/real_time_heuristic_search.hpp"
#include <limits>

namespace planalg
{
namespace search
{

template <typename State, typename ForEachSuccessor, typename ForEachPredecessor,
          typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> &&
           SuccessorProvider<ForEachPredecessor, State> &&
           StepCostProvider<ComputeStepCost, State> && GoalTestProvider<IsGoal, State>
class LearningRealTimeAStar
  : public RealTimeHeuristicSearch<State, ForEachSuccessor, ComputeStepCost, IsGoal>
{
  using Base = RealTimeHeuristicSearch<State, ForEachSuccessor, ComputeStepCost, IsGoal>;

public:
  LearningRealTimeAStar(IsGoal is_goal, ForEachSuccessor for_each_successor,
                        ForEachPredecessor for_each_predecessor, ComputeStepCost compute_step_cost,
                        HeuristicManagerUPtr<State> heuristic_manager,
                        std::size_t max_expansions = 100, double heuristic_weight = 1.0)
    : Base(std::move(is_goal), std::move(for_each_successor), std::move(compute_step_cost),
           std::move(heuristic_manager), max_expansions, heuristic_weight)
    , for_each_predecessor_(std::move(for_each_predecessor))
  {
  }

private:
  double computeOrLookupHeuristic(const State& state) override
  { return this->heuristicManager()->computeOrLookupHeuristic(state); }

  void learnHeuristic() override
  {
    if (this->closedSet().empty())
    {
      return;
    }
    initializeBackwardSearch();
    runBackwardSearch();
    applyHeuristicUpdates();

    backward_search_data_.clear();
  }

  void initializeBackwardSearch()
  {
    std::vector<OpenEntry<State>> seeds;
    seeds.reserve(this->openQueue().size() + 1);

    for (const auto& [state, _] : this->gValueMap())
    {
      if (this->closedSet().contains(state))
      {
        continue;
      }

      const double h = this->heuristicEvaluator()(state);

      backward_search_data_.g_value_map.emplace(state, h);

      seeds.push_back(OpenEntry<State>{ h, 0.0, state });
    }

    if (this->currentPath())
    {
      const State& goal = (*this->currentPath()).back();

      auto it = backward_search_data_.g_value_map.try_emplace(goal, 0.0);
      if (it.second)
      {
        seeds.push_back(OpenEntry<State>{ 0.0, 0.0, goal });
      }
    }

    backward_search_data_.open_queue = OpenQueue<State>(CompareOpenEntry<State>{}, std::move(seeds));
  }

  void runBackwardSearch()
  {
    auto is_goal = [](const State&) { return false; };

    auto predecessor_callback = [this](const State& state, const auto& callback) {
      this->for_each_predecessor_(state, [&](const State& predecessor) {
        if (this->closedSet().contains(predecessor))
        {
          callback(predecessor);
        }
      });
    };

    auto backward_cost = [this](const State& state, const State& predecessor) {
      return this->computeStepCost()(predecessor, state);
    };

    dijkstraSearch(backward_search_data_, is_goal, predecessor_callback, backward_cost);
  }

  void applyHeuristicUpdates()
  {
    const auto& g_value_map = backward_search_data_.g_value_map;

    for (const State& state : this->closedSet())
    {
      const auto it = g_value_map.find(state);

      const double h =
          (it != g_value_map.end()) ? it->second : std::numeric_limits<double>::infinity();

      this->heuristicManager()->updateHeuristicValue(state, h);
    }
  }

  SearchData<State> backward_search_data_;
  ForEachPredecessor for_each_predecessor_;
};
}  // namespace search
}  // namespace planalg
