#pragma once

#include <array>
#include <compare>
#include <functional>
struct State
{
  int x, y, t;

  auto operator<=>(const State&) const = default;
};

struct Action
{
  int dx;
  int dy;
};

using ActionSet = std::array<Action, 9>;

template <>
struct std::hash<State>
{
  std::size_t operator()(const State& s) const noexcept
  {
    std::size_t result = 0;

    const auto combine = [&](int value) {
      result ^= std::hash<int>{}(value) + 0x9e3779b9u + (result << 6) + (result >> 2);
    };

    combine(s.x);
    combine(s.y);
    combine(s.t);
    return result;
  }
};