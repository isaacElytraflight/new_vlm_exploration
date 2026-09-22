#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace explorer_mission
{

constexpr double STEP_M = 0.25;
constexpr double TURN_DEG = 10.0;
constexpr int YAW_BINS = 36;  // 360 / TURN_DEG

constexpr uint8_t DIR_FORWARD = 0;
constexpr uint8_t DIR_BACKWARD = 1;
constexpr uint8_t DIR_TURN_LEFT = 2;
constexpr uint8_t DIR_TURN_RIGHT = 3;

struct NavigationStep
{
  uint8_t direction{0};
  uint32_t steps{0};
};

struct DiscreteNavConfig
{
  double step_m{STEP_M};
  double turn_deg{TURN_DEG};
  /// Soft footprint used to reject poses near occupied cells.
  double robot_radius_m{0.20};
  double goal_tol_m{0.40};
  /// Match Nav2 exploration: unknown is traversable until proven occupied.
  bool allow_unknown{true};
  int8_t occupied_threshold{50};
  std::size_t max_expansions{250000};
};

struct DiscretePlanResult
{
  std::vector<NavigationStep> steps;
  bool ok{false};
  /// Expansions used (diagnostics).
  std::size_t expansions{0};
};

/// Obstacle-unaware geometric plan (turn → drive → turn). Kept for unit tests /
/// yaw-only moves; production navigation should use planOnOccupancy.
std::vector<NavigationStep> planToPose(
  double cx, double cy, double cyaw_deg,
  double gx, double gy, double gyaw_deg);

/// Lattice A* in DiscreteMove action space on an OccupancyGrid.
/// Actions: FORWARD/BACKWARD by step_m, TURN_LEFT/RIGHT by turn_deg.
DiscretePlanResult planOnOccupancy(
  const nav_msgs::msg::OccupancyGrid & grid,
  double cx, double cy, double cyaw_deg,
  double gx, double gy, double /*gyaw_deg*/,
  const DiscreteNavConfig & cfg = {});

/// True when a discrete lattice path exists (same planner, no step list needed).
bool discretePathExists(
  const nav_msgs::msg::OccupancyGrid & grid,
  double cx, double cy, double cyaw_deg,
  double gx, double gy,
  const DiscreteNavConfig & cfg = {});

/// Normalize angle to [-180, 180] degrees.
double normalizeAngleDeg(double angle_deg);

/// Shortest signed turn from current yaw to target yaw (degrees).
double shortestTurnDeg(double current_yaw_deg, double target_yaw_deg);

/// Merge consecutive unit actions into batched NavigationStep entries.
std::vector<NavigationStep> compactDiscreteActions(
  const std::vector<uint8_t> & unit_actions);

}  // namespace explorer_mission
