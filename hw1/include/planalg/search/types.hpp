#pragma once

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cstddef>
#include <memory>

namespace planalg
{
namespace search
{
template <typename State>
struct OpenEntry
{
  double priority;
  double secondary_priority;
  State state;
};

template <typename State>
struct CompareOpenEntry
{
  bool operator()(const OpenEntry<State>& a, const OpenEntry<State>& b) const
  {
    if (a.priority != b.priority)
    {
      return a.priority > b.priority;
    }

    return a.secondary_priority > b.secondary_priority;
  }
};

template <typename State>
using OpenQueue =
    std::priority_queue<OpenEntry<State>, std::vector<OpenEntry<State>>, CompareOpenEntry<State>>;

template <typename State>
using OpenFifoQueue = std::queue<State>;

template <typename State>
using ClosedSet = std::unordered_set<State>;

template <typename State>
using ClosedMap = std::unordered_map<State, std::size_t>;

template <typename State>
using PredecessorMap = std::unordered_map<State, State>;

template <typename State>
using GValueMap = std::unordered_map<State, double>;

template <typename State>
using HValueMap = GValueMap<State>;

template <typename State>
using Path = std::vector<State>;

template <typename State>
using PathUPtr = std::unique_ptr<Path<State>>;

template <typename State>
using StateSet = std::unordered_set<State>;

template <typename State>
using GoalSet = StateSet<State>;

template <typename State>
struct SearchData
{
  ClosedSet<State> closed_set;
  OpenQueue<State> open_queue;
  PredecessorMap<State> predecessor_map;
  GValueMap<State> g_value_map;

  void clear()
  {
    closed_set.clear();
    open_queue = {};
    predecessor_map.clear();
    g_value_map.clear();
  }
};

template <typename State>
struct BfsSearchData
{
  ClosedMap<State> closed_map;
  OpenFifoQueue<State> open_queue;
  PredecessorMap<State> predecessor_map;

    void clear()
  {
    closed_map.clear();
    open_queue = {};
    predecessor_map.clear();
  }

};

}  // namespace search
}  // namespace planalg