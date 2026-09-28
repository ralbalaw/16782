#pragma once

#include <types.hpp>
#include "planalg/search/types.hpp"

namespace utility
{
int getStateCost(const State& state, int x_size, const int* map);

bool isStateValid(const State& state, int threshold, const int* map, int x_size, int y_size,
                  int target_steps);

State getSuccessor(const State& state, const Action& action);

State getPredecessor(const State& state, const Action& action);

bool isGoal(const State& state, int target_steps, int* target_trajectory);

template <typename Callback>
void forEachSuccessor(const State& state, int threshold, const int* map, int x_size, int y_size,
                      const ActionSet& action_set, int target_steps, Callback&& callback)
{
  for (const auto& action : action_set)
  {
    const State successor = getSuccessor(state, action);
    if (isStateValid(successor, threshold, map, x_size, y_size, target_steps))
      callback(successor);
  }
}

template <typename Callback>
void forEachPredecessor(const State& state, int threshold, const int* map, int x_size, int y_size,
                        const ActionSet& action_set, int target_steps, Callback&& callback)
{
  for (const auto& action : action_set)
  {
    const State predecessor = getPredecessor(state, action);
    if (isStateValid(predecessor, threshold, map, x_size, y_size, target_steps))
      callback(predecessor);
  }
}

planalg::search::GoalSet<State> constructGoalSet(int target_steps, int* target_traj);
}  // namespace utility