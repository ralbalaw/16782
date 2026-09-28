/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>

#include "utility.hpp"
#include "types.hpp"
#include "planalg/search/types.hpp"
#include "planalg/search/rtaa.hpp"
#include "planalg/search/lss_lrta.hpp"
#include "planalg/search/heuristic/moving_target_grid_heuristic.hpp"
#include "planalg/search/heuristic/moving_target_heuristic.hpp"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y - 1) * XSIZE + (X - 1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using planalg::search::LearningRealTimeAStar;
using planalg::search::RealTimeAdaptiveAStar;
using namespace planalg::search::heuristic;
using namespace utility;

enum class PlannerAlgorithm
{
  RealTimeAdaptiveAStar,
  LearningRealTimeAStar
};

template <PlannerAlgorithm algorithm>
auto initializePlanner(const State& start_state, int* map, int collision_thresh, int x_size,
                       int y_size, ActionSet action_set, int target_steps, int* target_traj,
                       double heuristic_weight)
{
  using namespace planalg::search::heuristic;
  using namespace utility;

  Trajectory<State> trajectory;
  trajectory.reserve(target_steps);
  for (int time = 0; time < target_steps; ++time)
  {
    trajectory.push_back(State{ target_traj[time], target_traj[time + target_steps], time });
  }

  auto for_each_successor = [collision_thresh, map, x_size, y_size, action_set,
                             target_steps](const State& state, auto callback) {
    return forEachSuccessor(state, collision_thresh, map, x_size, y_size, action_set, target_steps,
                            callback);
  };

  auto for_each_predecessor = [collision_thresh, map, x_size, y_size, action_set,
                               target_steps](const State& state, auto callback) {
    return forEachPredecessor(state, collision_thresh, map, x_size, y_size, action_set,
                              target_steps, callback);
  };

  auto compute_step_cost = [map, x_size](const State& from, const State& to) {
    return static_cast<double>(getStateCost(from, x_size, map));
  };

  auto compute_state_heuristic = [](const State& state, std::span<const State> trajectory) {
    return movingTargetHeuristic(state, trajectory);
  };

  auto is_goal = [target_steps, target_traj](const State& state) {
    return isGoal(state, target_steps, target_traj);
  };

  auto heuristic_manager = std::make_unique<planalg::search::heuristic::MovingTargetGridHeuristic<
      State, decltype(for_each_successor), decltype(compute_step_cost),
      decltype(compute_state_heuristic)>>(std::move(trajectory), for_each_successor,
                                          compute_step_cost, compute_state_heuristic, start_state,
                                          x_size, y_size);

  if constexpr (algorithm == PlannerAlgorithm::RealTimeAdaptiveAStar)
  {
    return RealTimeAdaptiveAStar<State, decltype(for_each_successor), decltype(compute_step_cost),
                                 decltype(is_goal)>{
      is_goal, for_each_successor, compute_step_cost, std::move(heuristic_manager),
      100,     heuristic_weight
    };
  }
  else
  {
    return LearningRealTimeAStar<State, decltype(for_each_successor), decltype(for_each_predecessor),
                                 decltype(compute_step_cost), decltype(is_goal)>{
      is_goal,           for_each_successor,           for_each_predecessor,
      compute_step_cost, std::move(heuristic_manager), 100,
      heuristic_weight
    };
  }
}

void planner(int* map, int collision_thresh, int x_size, int y_size, int robotposeX, int robotposeY,
             int target_steps, int* target_traj, int targetposeX, int targetposeY, int curr_time,
             int* action_ptr)
{
  static ActionSet action_set{
    { { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, -1 }, { 0, 1 }, { 1, -1 }, { 1, 0 }, { 1, 1 }, { 0, 0 } }
  };

  static double heuristic_weight = 1.0;
  static auto planning_algorithm = initializePlanner<PlannerAlgorithm::RealTimeAdaptiveAStar>(
      State{ robotposeX, robotposeY, curr_time }, map, collision_thresh, x_size, y_size, action_set,
      target_steps, target_traj, heuristic_weight);

  const State current_state{ robotposeX, robotposeY, curr_time };

  const State next_state =
      planning_algorithm.planNextMove(current_state, std::chrono::milliseconds(950), 1000);

  action_ptr[0] = next_state.x;
  action_ptr[1] = next_state.y;

  return;
}
