#pragma once

#include <cmath>
#include <algorithm>

namespace planalg
{
namespace search
{
namespace heuristic
{
template <typename Position>
double octileDistance(const Position& p1, const Position& p2)
{
  const auto dx = std::abs(p1.x - p2.x);
  const auto dy = std::abs(p1.y - p2.y);

  const int a = std::max(dx, dy);
  const int b = std::min(dx, dy);

  return std::sqrt(2.0) * b + (a - b);
}

template <typename Position>
double chebyshevDistance(const Position& p1, const Position& p2)
{
  const int dx = std::abs(p1.x - p2.x);
  const int dy = std::abs(p1.y - p2.y);

  return std::max(dx, dy);
}

template <typename Position>
double manhattanDistance(const Position& p1, const Position& p2)
{
  const int dx = std::abs(p1.x - p2.x);
  const int dy = std::abs(p1.y - p2.y);

  return dx + dy;
}

template <typename Position>
double euclideanDistance(const Position& p1, const Position& p2)
{
  const int dx = std::abs(p1.x - p2.x);
  const int dy = std::abs(p1.y - p2.y);

  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace heuristic
}  // namespace search
}  // namespace planalg