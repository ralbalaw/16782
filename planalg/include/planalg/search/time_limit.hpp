#pragma once

#include <chrono>
#include <optional>

namespace planalg::search
{
class TimeLimit
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::milliseconds;

  explicit TimeLimit(std::optional<Duration> duration = std::nullopt)
    : deadline_(duration ? Clock::now() + *duration : Clock::time_point::max())
  {
  }

  bool expired() const
  { return Clock::now() >= deadline_; }

  TimeLimit portion(double fraction) const
  {
    if (deadline_ == Clock::time_point::max())
    {
      return *this;
    }

    const auto now = Clock::now();
    const auto remaining = deadline_ > now ? deadline_ - now : Clock::duration::zero();
    const auto portion = std::chrono::duration_cast<Clock::duration>(remaining * fraction);

    return TimeLimit{ now + portion };
  }

  auto stopCondition() const
  {
    return [deadline = deadline_]() { return Clock::now() >= deadline; };
  }

private:
  explicit TimeLimit(Clock::time_point deadline) : deadline_(deadline)
  {
  }

  Clock::time_point deadline_;
};
}  // namespace planalg::search
