#include "explorer_mission/wall_unstick.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace explorer_mission
{
namespace
{

bool worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x_m, double y_m, int * col, int * row)
{
  if (grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0) {
    return false;
  }
  const double res = grid.info.resolution;
  const int c = static_cast<int>(std::floor(
      (x_m - grid.info.origin.position.x) / res));
  const int r = static_cast<int>(std::floor(
      (y_m - grid.info.origin.position.y) / res));
  if (c < 0 || r < 0 ||
    c >= static_cast<int>(grid.info.width) ||
    r >= static_cast<int>(grid.info.height))
  {
    return false;
  }
  *col = c;
  *row = r;
  return true;
}

/// CV_32F distance (pixels) to nearest occupied; empty if no occupied cells.
cv::Mat occupiedDistancePx(const nav_msgs::msg::OccupancyGrid & grid, bool * have_occ)
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  cv::Mat binary(h, w, CV_8U, cv::Scalar(255));
  *have_occ = false;
  if (static_cast<int>(grid.data.size()) < w * h) {
    return {};
  }
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      const int8_t v = grid.data[static_cast<size_t>(r) * static_cast<size_t>(w) +
        static_cast<size_t>(c)];
      if (v >= 50) {
        binary.at<uint8_t>(r, c) = 0;
        *have_occ = true;
      }
    }
  }
  if (!*have_occ) {
    return {};
  }
  cv::Mat dist;
  // Distance to nearest zero (occupied).
  cv::distanceTransform(binary, dist, cv::DIST_L2, 3);
  return dist;
}

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

}  // namespace

UnstickDecision decideNavFailureRecovery(
  uint16_t error_code,
  double clearance_m,
  double min_clearance_m,
  bool already_unstuck)
{
  const bool low_clearance =
    std::isfinite(clearance_m) && clearance_m < min_clearance_m;
  const bool start_invalid =
    error_code == kNavErrorStartOccupied || low_clearance;
  const bool goal_invalid = error_code == kNavErrorGoalOccupied;
  const bool no_path = error_code == kNavErrorNoValidPath;

  if (goal_invalid) {
    return UnstickDecision{false, true};
  }
  if (start_invalid && !already_unstuck) {
    return UnstickDecision{true, false};
  }
  if (no_path) {
    // Unreachable path (OK start, or still unreachable after unstick) → mark.
    return UnstickDecision{false, true};
  }
  if (start_invalid && already_unstuck) {
    return UnstickDecision{false, false};
  }
  // Stuck / timeout / unknown: keep prior tree-progress behavior (mark).
  return UnstickDecision{false, true};
}

std::optional<double> clearanceToOccupiedM(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x_m,
  double y_m)
{
  int col = 0;
  int row = 0;
  if (!worldToGrid(grid, x_m, y_m, &col, &row)) {
    return std::nullopt;
  }
  bool have_occ = false;
  const cv::Mat dist = occupiedDistancePx(grid, &have_occ);
  if (dist.empty()) {
    // No walls documented → treat as large clearance.
    return std::numeric_limits<double>::infinity();
  }
  const float px = dist.at<float>(row, col);
  return static_cast<double>(px) * grid.info.resolution;
}

std::optional<cv::Point2f> clearanceGradientDir(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x_m,
  double y_m)
{
  int col = 0;
  int row = 0;
  if (!worldToGrid(grid, x_m, y_m, &col, &row)) {
    return std::nullopt;
  }
  bool have_occ = false;
  const cv::Mat dist = occupiedDistancePx(grid, &have_occ);
  if (dist.empty()) {
    return std::nullopt;
  }

  const int w = dist.cols;
  const int h = dist.rows;
  const int c0 = std::clamp(col, 1, w - 2);
  const int r0 = std::clamp(row, 1, h - 2);
  const float dx = dist.at<float>(r0, c0 + 1) - dist.at<float>(r0, c0 - 1);
  const float dy = dist.at<float>(r0 + 1, c0) - dist.at<float>(r0 - 1, c0);
  const float norm = std::hypot(dx, dy);
  if (norm < 1e-3f) {
    return std::nullopt;
  }
  return cv::Point2f(dx / norm, dy / norm);
}

std::optional<DiscreteUnstickStep> nextUnstickStep(
  double robot_yaw_rad,
  const cv::Point2f & gradient_dir_unit,
  double align_tol_rad)
{
  const float n = std::hypot(gradient_dir_unit.x, gradient_dir_unit.y);
  if (n < 1e-3f) {
    return std::nullopt;
  }
  const double desired = std::atan2(gradient_dir_unit.y, gradient_dir_unit.x);
  const double err = normalizeAngle(desired - robot_yaw_rad);
  DiscreteUnstickStep step;
  step.steps = 1;
  if (std::fabs(err) > align_tol_rad) {
    step.direction = err > 0.0 ? kUnstickTurnLeft : kUnstickTurnRight;
  } else {
    step.direction = kUnstickForward;
  }
  return step;
}

}  // namespace explorer_mission
