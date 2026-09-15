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
constexpr int kUnstickBackward = 1;
constexpr int kUnstickTurnLeft = 2;
constexpr int kUnstickTurnRight = 3;

/// Default outer unstick budget before marking the destination frontier dead.
constexpr int kDefaultMaxUnstickAttempts = 5;

/// Scale DiscreteMove step counts for Habitat (1 step ≈ 0.25 m). Keep modest —
/// thrash is only for stuck-robot recovery, not every unplannable frontier.
constexpr int kUnstickStepScale = 1;

struct UnstickDecision
{
  bool attempt_unstick{false};
  /// After thrash budget exhausted: one NavigateToPose with zero costmap inflation.
  bool attempt_zero_inflation{false};
  bool mark_frontier_dead{false};
};

/// Recovery policy after NavigateToPose failure.
///
/// @param unstick_attempts how many thrash recoveries already ran for this goal
/// @param max_unstick_attempts budget before zero-inflation last ditch (default 5)
/// @param zero_inflation_done whether the deflated-costmap retry already ran
///
/// Thrash DiscreteMove (back/forward, growing steps) on START_OCCUPIED / low
/// clearance / NO_VALID_PATH while attempts remain. After thrash budget, try
/// zero inflation once. Mark on GOAL_OCCUPIED, exhausted recovery, or generic stuck.
UnstickDecision decideNavFailureRecovery(
  uint16_t error_code,
  double clearance_m,
  double min_clearance_m,
  int unstick_attempts = 0,
  int max_unstick_attempts = kDefaultMaxUnstickAttempts,
  bool zero_inflation_done = false);

/// One recovery DiscreteMove: alternate BACKWARD / FORWARD with growing steps.
/// attempt 0: BACK×1, 1: FWD×1, 2: BACK×2, 3: FWD×2, 4: BACK×3, …
struct UnstickThrashMotion
{
  int direction{kUnstickBackward};
  int steps{1};
};

UnstickThrashMotion unstickThrashMotion(int attempt_index);

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
/// Retained for tests / optional map-based recovery; explore uses reverse steps.
std::optional<DiscreteUnstickStep> nextUnstickStep(
  double robot_yaw_rad,
  const cv::Point2f & gradient_dir_unit,
  double align_tol_rad = 0.35,
  int forward_steps = 1);

}  // namespace explorer_mission
