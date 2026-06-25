#pragma once

#include <cstdint>
#include <vector>

namespace explorer_mission
{

constexpr double STEP_M = 0.25;
constexpr double TURN_DEG = 10.0;

constexpr uint8_t DIR_FORWARD = 0;
constexpr uint8_t DIR_BACKWARD = 1;
constexpr uint8_t DIR_TURN_LEFT = 2;
constexpr uint8_t DIR_TURN_RIGHT = 3;

struct NavigationStep
{
  uint8_t direction{0};
  uint32_t steps{0};
};

/// Plan discrete base moves from current pose to goal pose.
std::vector<NavigationStep> planToPose(
  double cx, double cy, double cyaw_deg,
  double gx, double gy, double gyaw_deg);

/// Normalize angle to [-180, 180] degrees.
double normalizeAngleDeg(double angle_deg);

/// Shortest signed turn from current yaw to target yaw (degrees).
double shortestTurnDeg(double current_yaw_deg, double target_yaw_deg);

}  // namespace explorer_mission
