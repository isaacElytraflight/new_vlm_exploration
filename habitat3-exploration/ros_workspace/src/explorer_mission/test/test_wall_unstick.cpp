#include <cmath>

#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explorer_mission/wall_unstick.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeEmptyGrid(int w, int h, double res = 0.05)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(w);
  grid.info.height = static_cast<uint32_t>(h);
  grid.info.resolution = res;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.assign(static_cast<size_t>(w * h), 0);
  return grid;
}

void setOcc(nav_msgs::msg::OccupancyGrid & grid, int col, int row, int8_t v = 100)
{
  grid.data[static_cast<size_t>(row) * grid.info.width + static_cast<size_t>(col)] = v;
}

}  // namespace

TEST(WallUnstickHarness, PositiveControl)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(WallUnstickHarness, NegativeControl)
{
  const bool negative_control = (1 == 2);
  EXPECT_FALSE(negative_control);
}

TEST(DecideNavFailure, StartOccupiedRequestsUnstickNoMark_Positive)
{
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorStartOccupied, 1.0, 0.25, false);
  EXPECT_TRUE(d.attempt_unstick);
  EXPECT_FALSE(d.mark_frontier_dead);
}

TEST(DecideNavFailure, LowClearanceRequestsUnstickEvenWithoutCode_Positive)
{
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorNone, 0.10, 0.25, false);
  EXPECT_TRUE(d.attempt_unstick);
  EXPECT_FALSE(d.mark_frontier_dead);
}

TEST(DecideNavFailure, GoalOccupiedMarksWithoutUnstick_Positive)
{
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorGoalOccupied, 1.0, 0.25, false);
  EXPECT_FALSE(d.attempt_unstick);
  EXPECT_TRUE(d.mark_frontier_dead);
}

TEST(DecideNavFailure, NoPathWithGoodClearanceMarks_Positive)
{
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorNoValidPath, 1.0, 0.25, false);
  EXPECT_FALSE(d.attempt_unstick);
  EXPECT_TRUE(d.mark_frontier_dead);
}

TEST(DecideNavFailure, NoPathAfterUnstickMarks_Positive)
{
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorNoValidPath, 0.10, 0.25, true);
  EXPECT_FALSE(d.attempt_unstick);
  EXPECT_TRUE(d.mark_frontier_dead);
}

TEST(DecideNavFailure, StartStillBadAfterUnstickDoesNotMark_Negative)
{
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorStartOccupied, 0.05, 0.25, true);
  EXPECT_FALSE(d.attempt_unstick);
  EXPECT_FALSE(d.mark_frontier_dead);
}

TEST(DecideNavFailure, AdequateClearanceNoSpecialCodeMarks_Negative)
{
  // Generic abort / stuck with OK clearance: progress tree (existing behavior).
  const auto d = explorer_mission::decideNavFailureRecovery(
    explorer_mission::kNavErrorNone, 0.50, 0.25, false);
  EXPECT_FALSE(d.attempt_unstick);
  EXPECT_TRUE(d.mark_frontier_dead);
}

TEST(Clearance, FarFromWallHasLargeClearance_Positive)
{
  auto grid = makeEmptyGrid(40, 40);
  setOcc(grid, 0, 0);
  const auto c = explorer_mission::clearanceToOccupiedM(grid, 1.0, 1.0);
  ASSERT_TRUE(c.has_value());
  EXPECT_GT(*c, 0.5);
}

TEST(Clearance, AdjacentToWallHasSmallClearance_Positive)
{
  auto grid = makeEmptyGrid(40, 40);
  setOcc(grid, 20, 20);
  // One cell away at 0.05 m res → ~0.05 m
  const auto c = explorer_mission::clearanceToOccupiedM(grid, 21 * 0.05, 20 * 0.05);
  ASSERT_TRUE(c.has_value());
  EXPECT_LT(*c, 0.15);
}

TEST(Clearance, OffMapReturnsNullopt_Negative)
{
  auto grid = makeEmptyGrid(10, 10);
  EXPECT_FALSE(explorer_mission::clearanceToOccupiedM(grid, -1.0, -1.0).has_value());
}

TEST(ClearanceGradient, PointsAwayFromWall_Positive)
{
  auto grid = makeEmptyGrid(40, 40);
  // Vertical wall on the left
  for (int r = 0; r < 40; ++r) {
    setOcc(grid, 5, r);
  }
  const auto g = explorer_mission::clearanceGradientDir(grid, 8 * 0.05, 20 * 0.05);
  ASSERT_TRUE(g.has_value());
  EXPECT_GT(g->x, 0.0f);  // away from wall → +x
}

TEST(ClearanceGradient, EmptyMapNoGradient_Negative)
{
  auto grid = makeEmptyGrid(20, 20);
  // No occupied cells → distance field flat / undefined for "away from walls"
  const auto g = explorer_mission::clearanceGradientDir(grid, 0.5, 0.5);
  EXPECT_FALSE(g.has_value());
}

TEST(NextUnstickStep, TurnsWhenMisaligned_Positive)
{
  cv::Point2f grad(1.0f, 0.0f);  // want +x
  const auto step = explorer_mission::nextUnstickStep(/*yaw=*/M_PI / 2.0, grad, 0.35);
  ASSERT_TRUE(step.has_value());
  EXPECT_EQ(step->direction, explorer_mission::kUnstickTurnRight);
}

TEST(NextUnstickStep, ForwardsWhenAligned_Positive)
{
  cv::Point2f grad(1.0f, 0.0f);
  const auto step = explorer_mission::nextUnstickStep(/*yaw=*/0.05, grad, 0.35);
  ASSERT_TRUE(step.has_value());
  EXPECT_EQ(step->direction, explorer_mission::kUnstickForward);
}

TEST(NextUnstickStep, ZeroGradientRejected_Negative)
{
  cv::Point2f grad(0.0f, 0.0f);
  EXPECT_FALSE(explorer_mission::nextUnstickStep(0.0, grad).has_value());
}
