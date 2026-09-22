#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "explorer_mission/discrete_navigator.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeEmptyGrid(
  int w, int h, double res = 0.05, double origin_x = -5.0, double origin_y = -5.0)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(w);
  grid.info.height = static_cast<uint32_t>(h);
  grid.info.resolution = res;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.data.assign(static_cast<size_t>(w * h), 0);
  return grid;
}

void setOcc(nav_msgs::msg::OccupancyGrid & grid, int col, int row, int8_t v = 100)
{
  grid.data[static_cast<size_t>(row) * grid.info.width + static_cast<size_t>(col)] = v;
}

void paintWallVertical(
  nav_msgs::msg::OccupancyGrid & grid, int col, int row0, int row1)
{
  for (int r = row0; r <= row1; ++r) {
    setOcc(grid, col, r);
  }
}

}  // namespace

TEST(DiscreteNavigatorHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(DiscreteNavigatorHarness, IntentionalFailureIsDetectable_NegativeControl)
{
  EXPECT_FALSE(1 == 2);
}

TEST(DiscreteNavigator, PlanToPoseTurnsThenMoves)
{
  const auto plan = explorer_mission::planToPose(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  ASSERT_FALSE(plan.empty());
  bool has_forward = false;
  for (const auto & step : plan) {
    if (step.direction == explorer_mission::DIR_FORWARD && step.steps > 0) {
      has_forward = true;
    }
  }
  EXPECT_TRUE(has_forward);
}

TEST(DiscreteNavigator, ShortestTurnWraps)
{
  const double turn = explorer_mission::shortestTurnDeg(170.0, -170.0);
  EXPECT_NEAR(turn, 20.0, 1e-6);
}

TEST(DiscreteNavigator, StepConstants)
{
  EXPECT_DOUBLE_EQ(explorer_mission::STEP_M, 0.25);
  EXPECT_DOUBLE_EQ(explorer_mission::TURN_DEG, 10.0);
}

TEST(DiscreteNavigator, CompactMergesRuns_Positive)
{
  const auto plan = explorer_mission::compactDiscreteActions(
    {explorer_mission::DIR_TURN_LEFT, explorer_mission::DIR_TURN_LEFT,
      explorer_mission::DIR_FORWARD, explorer_mission::DIR_FORWARD,
      explorer_mission::DIR_FORWARD});
  ASSERT_EQ(plan.size(), 2u);
  EXPECT_EQ(plan[0].direction, explorer_mission::DIR_TURN_LEFT);
  EXPECT_EQ(plan[0].steps, 2u);
  EXPECT_EQ(plan[1].direction, explorer_mission::DIR_FORWARD);
  EXPECT_EQ(plan[1].steps, 3u);
}

TEST(DiscreteNavigator, LatticeOpenMapReachesGoal_Positive)
{
  auto grid = makeEmptyGrid(200, 200, 0.05, -5.0, -5.0);
  explorer_mission::DiscreteNavConfig cfg;
  cfg.robot_radius_m = 0.15;
  cfg.goal_tol_m = 0.35;
  const auto plan = explorer_mission::planOnOccupancy(
    grid, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, cfg);
  EXPECT_TRUE(plan.ok);
  ASSERT_FALSE(plan.steps.empty());
  bool has_forward = false;
  for (const auto & s : plan.steps) {
    if (s.direction == explorer_mission::DIR_FORWARD) {
      has_forward = true;
    }
  }
  EXPECT_TRUE(has_forward);
}

TEST(DiscreteNavigator, LatticeGoesAroundWall_Positive)
{
  auto grid = makeEmptyGrid(200, 200, 0.05, -5.0, -5.0);
  // Wall at x≈0.5m blocking straight line from (0,0) to (1.5,0).
  // col = floor((0.5 - (-5)) / 0.05) = floor(5.5/0.05) = 110
  paintWallVertical(grid, 110, 80, 120);
  paintWallVertical(grid, 111, 80, 120);
  paintWallVertical(grid, 112, 80, 120);

  explorer_mission::DiscreteNavConfig cfg;
  cfg.robot_radius_m = 0.15;
  cfg.goal_tol_m = 0.40;
  cfg.max_expansions = 400000;
  const auto plan = explorer_mission::planOnOccupancy(
    grid, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, cfg);
  EXPECT_TRUE(plan.ok) << "expansions=" << plan.expansions;
  EXPECT_FALSE(plan.steps.empty());
}

TEST(DiscreteNavigator, LatticeBlockedCorridorUnreachable_Negative)
{
  auto grid = makeEmptyGrid(200, 200, 0.05, -5.0, -5.0);
  // Seal a vertical wall across the whole map so goal is unreachable.
  for (int r = 0; r < 200; ++r) {
    setOcc(grid, 110, r);
    setOcc(grid, 111, r);
  }
  explorer_mission::DiscreteNavConfig cfg;
  cfg.robot_radius_m = 0.15;
  cfg.goal_tol_m = 0.35;
  cfg.max_expansions = 100000;
  const auto plan = explorer_mission::planOnOccupancy(
    grid, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, cfg);
  EXPECT_FALSE(plan.ok);
  EXPECT_TRUE(plan.steps.empty());
}

TEST(DiscreteNavigator, PathExistsMatchesPlan_Positive)
{
  auto grid = makeEmptyGrid(200, 200, 0.05, -5.0, -5.0);
  explorer_mission::DiscreteNavConfig cfg;
  cfg.robot_radius_m = 0.15;
  EXPECT_TRUE(explorer_mission::discretePathExists(
    grid, 0.0, 0.0, 0.0, 1.0, 0.0, cfg));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
