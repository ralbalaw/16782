#pragma once

#include <cmath>
#include <limits>
#include <span>

#include "planalg/search/heuristic/concepts.hpp"
#include "planalg/search/heuristic/heuristic_manager.hpp"
#include "planalg/search/heuristic/types.hpp"
#include "planalg/search/types.hpp"
#include "planalg/search/concepts.hpp"
#include "planalg/search/dijkstra_search.hpp"
#include "planalg/search/utility.hpp"

namespace planalg::search::heuristic
{
namespace
{
template <typename State>
std::span<const State> sliceAt(const std::vector<State>& trajectory, std::size_t start_index)
{ return std::span<const State>(trajectory.data() + start_index, trajectory.size() - start_index); }
}  // namespace

template <typename State, typename ForEachSuccessor, typename ComputeStepCost,
          typename ComputeTargetsHeuristic>
  requires SuccessorProvider<ForEachSuccessor, State> && StepCostProvider<ComputeStepCost, State> &&
           TargetsHeuristicProvider<ComputeTargetsHeuristic, State>

class MovingTargetGridHeuristic : public HeuristicManager<State>
{
public:
  MovingTargetGridHeuristic(Trajectory<State> trajectory, ForEachSuccessor for_each_neighbor,
                            ComputeStepCost compute_step_cost,
                            ComputeTargetsHeuristic compute_heuristic, const State& start_state,
                            int width, int height)
    : for_each_neighbor_(for_each_neighbor)
    , compute_step_cost_(compute_step_cost)
    , compute_heuristic_(compute_heuristic)
    , trajectory_(std::move(trajectory))
    , width_(width)
    , height_(height)
  { initializeDepartureAndSpatialSearchData(start_state); }

  double computeOrLookupHeuristic(const State& state) override
  {
    const double departure_cost = departureCost(state);

    if (departure_cost == std::numeric_limits<double>::infinity())
    {
      return departure_cost;
    }

    const double spatial_cost = spatialCost(fromState(state));
    if (spatial_cost == std::numeric_limits<double>::infinity())
    {
      return spatial_cost;
    }

    return std::max(geometricLowerBound(state), spatial_cost);
  }

  void updateHeuristicValue(const State& state, double value) override
  { geometric_lower_bound_.insert_or_assign(state, value); }

  void learn(const State& state, const TimeLimit& time_limit) override
  {
    if (!shouldContinueLearningDeparture(state.t))
    {
      departure_search_data_.open_queue = {};
    }

    if (!departure_search_data_.open_queue.empty())
    {
      backwardDepartureLearning(time_limit);

      if (time_limit.expired())
      {
        return;
      }
    }

    if (!spatial_search_data_.open_queue.empty())
    {
      backwardSpatialCostLearning(time_limit);
    }
  }

  bool finishedLearning(const State& state) override
  { return !shouldContinueLearningDeparture(state.t) && !shouldContinueLearningSpatial(); }

  bool shouldContinueLearningDeparture(int current_state_time) const
  {
    const auto& open_queue = departure_search_data_.open_queue;

    return !open_queue.empty() && std::isfinite(open_queue.top().priority) &&
           open_queue.top().priority <= horizon_ - current_state_time;
  }

  bool shouldContinueLearningSpatial() const
  { return !spatial_search_data_.open_queue.empty(); }

  void backwardSpatialCostLearning(const TimeLimit& time_limit)
  {
    auto is_goal = [](int) { return false; };

    auto compute_backward_step_cost = [this](int to, int from) {
      return compute_step_cost_(toState(from), toState(to));
    };

    auto neighbor_callback = [this](int index, const auto& callback) {
      for_each_neighbor_(toState(index),
                         [&](const State& predecessor) { callback(fromState(predecessor)); });
    };

    auto backward_path = dijkstraSearch(spatial_search_data_, is_goal, neighbor_callback,
                                        compute_backward_step_cost, time_limit);
  }

  void backwardDepartureLearning(const TimeLimit& time_limit)
  {
    auto is_goal = [](int) { return false; };

    auto neighbor_callback = [this](int index, const auto& callback) {
      for_each_neighbor_(toState(index),
                         [&](const State& predecessor) { callback(fromState(predecessor)); });
    };

    auto compute_step_cost = [](const int&, const int&) -> double { return 1.0; };

    dijkstraSearch(departure_search_data_, is_goal, neighbor_callback, compute_step_cost,
                   time_limit);
  }

private:
  double geometricLowerBound(const State& state)
  {
    const auto it = geometric_lower_bound_.find(state);
    if (it != geometric_lower_bound_.end())
    {
      return it->second;
    }

    const double h = compute_heuristic_(state, sliceAt(trajectory_, first_feasible_target_index_));
    geometric_lower_bound_.emplace(state, h);

    return h;
  }

  double spatialCost(int index) const
  {
    const ClosedSet<int>& closed_set = spatial_search_data_.closed_set;
    if (closed_set.contains(index))
    {
      return spatial_search_data_.g_value_map.at(index);
    }
    if (spatial_search_data_.open_queue.empty())
    {
      return std::numeric_limits<double>::infinity();
    }
    return spatial_search_data_.open_queue.top().priority;
  }

  double departureCost(const State& state) const
  {
    const int index = fromState(state);
    const ClosedSet<int>& closed_set = departure_search_data_.closed_set;

    if (closed_set.contains(index))
    {
      const double g = departure_search_data_.g_value_map.at(index);
      const double latest_departure_time = horizon_ - g;

      if (latest_departure_time >= state.t)
      {
        return 0.0;
      }
      return std::numeric_limits<double>::infinity();
    }

    if (departure_search_data_.open_queue.empty())
    {
      return std::numeric_limits<double>::infinity();
    }

    const double g = departure_search_data_.open_queue.top().priority;
    const double latest_departure_time = horizon_ - g;

    if (latest_departure_time >= state.t)
    {
      return 0.0;
    }

    return std::numeric_limits<double>::infinity();
  }

  void initializeDepartureAndSpatialSearchData(const State& start_state)
  {
    const auto inf = std::numeric_limits<double>::infinity();
    const double first_feasible_duration = compute_heuristic_(start_state, trajectory_);
    if (first_feasible_duration == inf)
    {
      first_feasible_target_index_ = static_cast<int>(trajectory_.size());
    }
    else
    {
      const int first_feasible_target_time =
          start_state.t + static_cast<int>(first_feasible_duration);
      first_feasible_target_index_ = first_feasible_target_time - trajectory_.front().t;
    }

    std::span<const State> feasible_targets = sliceAt(trajectory_, first_feasible_target_index_);
    std::vector<int> feasible_targets_int;
    feasible_targets_int.reserve(feasible_targets.size());

    horizon_ = trajectory_.size() - 1;
    for (const auto& [x, y, target_time] : feasible_targets)
    {
      const double seed_cost = horizon_ - target_time;
      const int index = toIndex(x, y);

      feasible_targets_int.push_back(index);

      auto [it, inserted] = departure_search_data_.g_value_map.try_emplace(index, seed_cost);
      if (!inserted && seed_cost >= it->second)
      {
        continue;
      }

      it->second = seed_cost;
      departure_search_data_.open_queue.emplace(seed_cost, 0, index);
    }

    initializeSearchData(feasible_targets_int, spatial_search_data_);
  }

  int toIndex(int x, int y) const
  { return (y - 1) * width_ + (x - 1); }

  State toState(int index) const
  { return State{ index % width_ + 1, index / width_ + 1, 0 }; }

  int fromState(const State& state) const
  { return toIndex(state.x, state.y); }

  Trajectory<State> trajectory_;
  std::span<const State> feasible_targets_;

  ForEachSuccessor for_each_neighbor_;
  ComputeStepCost compute_step_cost_;
  ComputeTargetsHeuristic compute_heuristic_;

  HValueMap<State> geometric_lower_bound_;

  SearchData<int> spatial_search_data_;
  PathUPtr<int> spatial_path_ = nullptr;

  SearchData<int> departure_search_data_;

  int width_;
  int height_;
  double horizon_;
  int first_feasible_target_index_ = 0;
};
}  // namespace planalg::search::heuristic
