#pragma once

#include <memory>

#include "planalg/search/time_limit.hpp"

namespace planalg::search::heuristic
{
template <typename State>
class HeuristicManager
{
public:
  virtual ~HeuristicManager() = default;

  virtual double computeOrLookupHeuristic(const State& state) = 0;
  virtual void updateHeuristicValue(const State& state, double value) = 0;
  virtual void learn(const State& state, const TimeLimit& time_limit) = 0;
  virtual bool finishedLearning(const State& state) = 0;
};
template <typename State>
using HeuristicManagerUPtr = std::unique_ptr<heuristic::HeuristicManager<State>>;
}  // namespace planalg::search::heuristic
