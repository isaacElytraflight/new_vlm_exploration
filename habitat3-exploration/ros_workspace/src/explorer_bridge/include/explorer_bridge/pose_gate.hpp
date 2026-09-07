#pragma once

#include <cmath>
#include <optional>

namespace explorer_bridge
{

constexpr double kPi = 3.14159265358979323846;

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double wrapAngleAbs(double dyaw)
{
  dyaw = std::abs(dyaw);
  while (dyaw > kPi) {
    dyaw = std::abs(dyaw - 2.0 * kPi);
  }
  return dyaw;
}

/// Integrate when first pose, or Δxy / Δyaw exceed thresholds.
inline bool shouldIntegratePose(
  const Pose2D & pose,
  const std::optional<Pose2D> & last,
  double min_xy_m,
  double min_yaw_rad)
{
  if (!last.has_value()) {
    return true;
  }
  const double dx = pose.x - last->x;
  const double dy = pose.y - last->y;
  if (std::hypot(dx, dy) >= min_xy_m) {
    return true;
  }
  return wrapAngleAbs(pose.yaw - last->yaw) >= min_yaw_rad;
}

}  // namespace explorer_bridge
