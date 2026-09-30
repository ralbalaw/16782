#pragma once

#include "planalg/search/types.hpp"
#include <ranges>

namespace planalg
{
namespace search
{
template <typename State>
void initializeSearchData(const State& state, SearchData<State>& search_data)
{
  search_data.open_queue.emplace(0.0, 0, state);
  search_data.g_value_map.emplace(state, 0.0);
}

template <std::ranges::input_range Range>
void initializeSearchData(const Range& states,
                          SearchData<std::ranges::range_value_t<Range>>& search_data)
{
  for (const auto& state : states)
  {
    search_data.open_queue.emplace(0.0, 0, state);
    search_data.g_value_map.emplace(state, 0.0);
  }
}

template <typename State>
void initializeBfsSearchData(const State& start_state, BfsSearchData<State>& search_data)
{
  search_data.open_queue.emplace(start_state);
  search_data.closed_map.emplace(start_state, 0);
}

template <std::ranges::input_range Range>
void initializeBfsSearchData(const Range& states,
                             BfsSearchData<std::ranges::range_value_t<Range>>& search_data)
{
  for (const auto& state : states)
  {
    search_data.open_queue.emplace(state);
    search_data.closed_map.emplace(state, 0);
  }
}

}  // namespace search
}  // namespace planalg