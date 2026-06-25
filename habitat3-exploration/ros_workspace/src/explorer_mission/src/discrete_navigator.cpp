#include "explorer_mission/discrete_navigator.hpp"

#include <cmath>

namespace explorer_mission
{

double normalizeAngleDeg(double angle_deg)
{
  while (angle_deg > 180.0) {
    angle_deg -= 360.0;
  }
  while (angle_deg < -180.0) {
    angle_deg += 360.0;
  }
  return angle_deg;
}

double shortestTurnDeg(double current_yaw_deg, double target_yaw_deg)
{
  double diff = normalizeAngleDeg(target_yaw_deg - current_yaw_deg);
  return diff;
}

static void appendTurnSteps(std::vector<NavigationStep> & plan, double turn_deg)
{
  if (std::abs(turn_deg) < 1e-6) {
    return;
  }
  const uint32_t steps = static_cast<uint32_t>(std::lround(std::abs(turn_deg) / TURN_DEG));
  if (steps == 0) {
    return;
  }
  NavigationStep step;
  step.direction = (turn_deg > 0.0) ? DIR_TURN_LEFT : DIR_TURN_RIGHT;
  step.steps = steps;
  plan.push_back(step);
}

std::vector<NavigationStep> planToPose(
  double cx, double cy, double cyaw_deg,
  double gx, double gy, double gyaw_deg)
{
  std::vector<NavigationStep> plan;

  const double dx = gx - cx;
  const double dy = gy - cy;
  const double distance = std::hypot(dx, dy);

  if (distance > 1e-6) {
    const double bearing_deg = normalizeAngleDeg(std::atan2(dy, dx) * 180.0 / M_PI);
    appendTurnSteps(plan, shortestTurnDeg(cyaw_deg, bearing_deg));

    const uint32_t forward_steps = static_cast<uint32_t>(std::lround(distance / STEP_M));
    if (forward_steps > 0) {
      plan.push_back({DIR_FORWARD, forward_steps});
    }

    appendTurnSteps(plan, shortestTurnDeg(bearing_deg, gyaw_deg));
  } else {
    appendTurnSteps(plan, shortestTurnDeg(cyaw_deg, gyaw_deg));
  }

  return plan;
}

}  // namespace explorer_mission
