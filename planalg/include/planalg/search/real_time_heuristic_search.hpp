#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <utility>

#include "planalg/search/types.hpp"
#include "planalg/search/concepts.hpp"
#include "planalg/search/path.hpp"
#include "planalg/search/a_star_search.hpp"
#include "planalg/search/time_limit.hpp"
#include "planalg/search/heuristic/heuristic_manager.hpp"

namespace planalg
{
namespace search
{
template <typename State>
using HeuristicManagerUPtr = heuristic::HeuristicManagerUPtr<State>;

template <typename State, typename ForEachSuccessor, typename ComputeStepCost, typename IsGoal>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           GoalTestProvider<IsGoal, State>
class RealTimeHeuristicSearch
{
public:
  RealTimeHeuristicSearch(IsGoal is_goal, ForEachSuccessor for_each_successor,
                          ComputeStepCost compute_step_cost,
                          HeuristicManagerUPtr<State> heuristic_manager,
                          std::size_t max_expansions = 100, double heuristic_weight = 1.0)
    : is_goal_(std::move(is_goal))
    , for_each_successor_(std::move(for_each_successor))
    , compute_step_cost_(std::move(compute_step_cost))
    , heuristic_manager_(std::move(heuristic_manager))
    , max_expansions_(max_expansions)
    , heuristic_weight_(heuristic_weight)
  {
  }

  virtual ~RealTimeHeuristicSearch() = default;

  RealTimeHeuristicSearch(const RealTimeHeuristicSearch&) = delete;
  RealTimeHeuristicSearch& operator=(const RealTimeHeuristicSearch&) = delete;
  RealTimeHeuristicSearch(RealTimeHeuristicSearch&&) = default;
  RealTimeHeuristicSearch& operator=(RealTimeHeuristicSearch&&) = default;

  State planNextMove(const State& current_state,
                     std::optional<std::chrono::milliseconds> time_limit = std::nullopt,
                     std::optional<std::size_t> step_max_expansions = std::nullopt)
  {
    const TimeLimit planning_time_limit{ time_limit };

    if (!step_max_expansions)
    {
      step_max_expansions = max_expansions_;
    }

    if (is_goal_(current_state))
    {
      return current_state;
    }

    if (time_limit)
    {
      performAdditionalLearning(current_state, planning_time_limit.portion(3, 4));
    }

    search_data.clear();
    current_path =
        performLookaheadSearch(current_state, step_max_expansions, planning_time_limit);

    learnHeuristic(planning_time_limit);

    const State next_state = chooseNextState(current_state);

    if (time_limit && !planning_time_limit.expired())
    {
      performAdditionalLearning(current_state, planning_time_limit);
    }

    return next_state;
  }

protected:
  virtual PathUPtr<State>
  performLookaheadSearch(const State& current_state,
                         std::optional<std::size_t> max_expansions,
                         const TimeLimit& time_limit)
  {
    initializeSearchData(current_state, search_data);
    return aStarSearch(search_data, is_goal_, for_each_successor_, compute_step_cost_,
                       heuristicEvaluator(), heuristic_weight_, time_limit, max_expansions);
  }

  virtual void performAdditionalLearning(const State&, const TimeLimit&)
  {
  }

  virtual double computeOrLookupHeuristic(const State& state)
  {
    const auto it = h_value_map.find(state);
    if (it != h_value_map.end())
    {
      return it->second;
    }

    const double h = heuristic_manager_->computeOrLookupHeuristic(state);
    h_value_map.emplace(state, h);

    return h;
  }

  virtual void learnHeuristic(const TimeLimit&)
  {
  }

  virtual State chooseNextState(const State& current_state)
  {
    if (!current_path)
    {
      if (search_data.open_queue.empty())
      {
        return current_state;
      }

      const auto entry = search_data.open_queue.top();
      current_path = reconstructPath(search_data.predecessor_map, entry.state);
    }

    if (current_path->size() < 2)
    {
      return current_state;
    }
    return (*current_path)[1];
  }

  auto heuristicEvaluator()
  {
    return [this](const State& state) { return computeOrLookupHeuristic(state); };
  }

  SearchData<State>& searchData()
  { return search_data; }

  OpenQueue<State>& openQueue()
  { return search_data.open_queue; }

  ClosedSet<State>& closedSet()
  { return search_data.closed_set; }

  GValueMap<State>& gValueMap()
  { return search_data.g_value_map; }

  PathUPtr<State>& currentPath()
  { return current_path; }

  HValueMap<State>& heuristicValueMap()
  { return h_value_map; }

  const IsGoal& isGoal() const
  { return is_goal_; }

  const ForEachSuccessor& forEachSuccessor() const
  { return for_each_successor_; }

  const ComputeStepCost& computeStepCost() const
  { return compute_step_cost_; }

  const std::unique_ptr<heuristic::HeuristicManager<State>>& heuristicManager() const
  { return heuristic_manager_; }

  std::size_t maxExpansions() const
  { return max_expansions_; }

private:
  HValueMap<State> h_value_map;
  SearchData<State> search_data;
  IsGoal is_goal_;
  ForEachSuccessor for_each_successor_;
  ComputeStepCost compute_step_cost_;
  HeuristicManagerUPtr<State> heuristic_manager_;
  double heuristic_weight_;

  PathUPtr<State> current_path;

  std::size_t max_expansions_ = 100;
};
}  // namespace search
}  // namespace planalg
