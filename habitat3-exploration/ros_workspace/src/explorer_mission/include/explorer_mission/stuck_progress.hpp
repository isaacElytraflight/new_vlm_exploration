#pragma once

#include <cmath>

namespace explorer_mission
{

/// Default Nav2 give-up policy for frontier navigation.
constexpr double kNavTotalTimeoutS = 300.0;
constexpr double kNavStuckTimeoutS = 60.0;
constexpr double kNavStuckDistanceM = 1.0;

/// Pure stuck/progress tracker: give up only when XY has not advanced by
/// stuck_distance_m within stuck_timeout_s (clock is caller-supplied seconds).
class StuckProgressTracker
{
public:
  void reset()
  {
    have_anchor_ = false;
  }

  void noteProgress(double x, double y, double stuck_distance_m, double now_s)
  {
    if (!have_anchor_) {
      progress_x_ = x;
      progress_y_ = y;
      have_anchor_ = true;
      last_progress_time_s_ = now_s;
      return;
    }
    const double dx = x - progress_x_;
    const double dy = y - progress_y_;
    if (std::hypot(dx, dy) >= stuck_distance_m) {
      progress_x_ = x;
      progress_y_ = y;
      last_progress_time_s_ = now_s;
    }
  }

  bool isStuck(double stuck_timeout_s, double now_s) const
  {
    if (!have_anchor_) {
      return false;
    }
    return (now_s - last_progress_time_s_) > stuck_timeout_s;
  }

  bool haveAnchor() const {return have_anchor_;}

private:
  bool have_anchor_{false};
  double progress_x_{0.0};
  double progress_y_{0.0};
  double last_progress_time_s_{0.0};
};

}  // namespace explorer_mission
