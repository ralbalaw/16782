#include "utility.hpp"
#include <cmath>
#include <stdexcept>
#include <limits>

namespace utility
{

int getStateCost(const State& state, int x_size, const int* map)
{ return map[(state.y - 1) * x_size + (state.x - 1)]; }

bool isStateValid(const State& state, int threshold, const int* map, int x_size, int y_size,
                  int target_steps)
{
  if (state.x < 1 || state.x > x_size || state.y < 1 || state.y > y_size)
  {
    return false;
  }
  if (state.t < 0 || state.t >= target_steps)
  {
    return false;
  }
  const int state_cost = getStateCost(state, x_size, map);

  return state_cost >= 0 && state_cost < threshold;
}

State getSuccessor(const State& state, const Action& action)
{
  const int successor_x = state.x + action.dx;
  const int successor_y = state.y + action.dy;
  const int successor_t = state.t + 1;

  return State{ successor_x, successor_y, successor_t };
}

State getPredecessor(const State& state, const Action& action)
{
  const int predeccessor_x = state.x + action.dx;
  const int predeccessor_y = state.y + action.dy;
  const int predeccessor_t = state.t - 1;

  return State{ predeccessor_x, predeccessor_y, predeccessor_t };
}

State getTargetAtTime(int current_time, int lookahead_time, const int* target_trajectory,
                      int trajectory_length)
{
  const int target_time = current_time + lookahead_time;
  if (target_time >= trajectory_length)
  {
    throw std::invalid_argument("target time is out of range; max valid target time is " +
                                std::to_string(trajectory_length - 1));
  }
  const int target_x = target_trajectory[target_time];
  const int target_y = target_trajectory[target_time + trajectory_length];

  return State{ target_x, target_y, target_time };
}

bool isGoal(const State& state, int target_steps, int* target_trajectory)
{
  if (state.t >= target_steps)
  {
    return false;
  }
  return target_trajectory[state.t] == state.x &&
         target_trajectory[state.t + target_steps] == state.y;
}

planalg::search::GoalSet<State> constructGoalSet(int target_steps, int* target_traj)
{
  planalg::search::GoalSet<State> goal_set;
  for (int i = 0; i < target_steps; ++i)
  {
    const State state = getTargetAtTime(0, i, target_traj, target_steps);
    goal_set.insert(state);
  }
  return goal_set;
}

}  // namespace utility