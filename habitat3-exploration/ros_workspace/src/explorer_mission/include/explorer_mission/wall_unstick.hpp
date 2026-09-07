#pragma once

#include <cstdint>
#include <optional>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>

namespace explorer_mission
{

// Match nav2_msgs/action/ComputePathToPose error codes (also used on NavigateToPose).
constexpr uint16_t kNavErrorNone = 0;
constexpr uint16_t kNavErrorStartOccupied = 205;
constexpr uint16_t kNavErrorGoalOccupied = 206;
constexpr uint16_t kNavErrorNoValidPath = 208;

/// DiscreteMove direction values (explorer_msgs/DiscreteMove).
constexpr int kUnstickForward = 0;
constexpr int kUnstickTurnLeft = 2;
constexpr int kUnstickTurnRight = 3;

struct UnstickDecision
{
  bool attempt_unstick{false};
  bool mark_frontier_dead{false};
};

/// Safer recovery policy:
/// - Unstick if START_OCCUPIED or clearance < min.
/// - Mark frontier dead mainly on GOAL_OCCUPIED, or NO_VALID_PATH after unstick
///   (also mark other non-start failures so stuck/timeout still progress the tree).
UnstickDecision decideNavFailureRecovery(
  uint16_t error_code,
  double clearance_m,
  double min_clearance_m,
  bool already_unstuck);

/// Distance (m) to nearest occupied cell (>=50). nullopt if pose off-map / bad grid.
std::optional<double> clearanceToOccupiedM(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x_m,
  double y_m);

/// Unit gradient of clearance (map frame). nullopt if unavailable / flat.
std::optional<cv::Point2f> clearanceGradientDir(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x_m,
  double y_m);

struct DiscreteUnstickStep
{
  int direction{kUnstickForward};
  int steps{1};
};

/// Turn toward gradient, or step forward once aligned within align_tol_rad.
std::optional<DiscreteUnstickStep> nextUnstickStep(
  double robot_yaw_rad,
  const cv::Point2f & gradient_dir_unit,
  double align_tol_rad = 0.35);

}  // namespace explorer_mission
