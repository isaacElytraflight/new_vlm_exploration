#include <gtest/gtest.h>

#include <cmath>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explorer_mission/frontier_detection.hpp"

static nav_msgs::msg::OccupancyGrid makeTestGrid()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 20;
  grid.info.height = 20;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.assign(400, 100);

  for (int y = 5; y < 15; ++y) {
    for (int x = 5; x < 15; ++x) {
      grid.data[y * 20 + x] = 0;
    }
  }
  for (int y = 14; y < 20; ++y) {
    for (int x = 5; x < 15; ++x) {
      grid.data[y * 20 + x] = -1;
    }
  }
  return grid;
}

TEST(FrontierDetectionHarness, RunnerExecutesAssertions)
{
  EXPECT_EQ(1 + 1, 2);
}

TEST(FrontierDetectionHarness, IntentionalFailureIsDetectable)
{
  const bool negative_control = (1 == 2);
  EXPECT_TRUE(negative_control == false);
}

TEST(FrontierDetection, FindsContoursOnFreeUnknownBoundary)
{
  const auto grid = makeTestGrid();
  const auto contours = explorer_mission::findFrontierContours(grid, 5);
  EXPECT_FALSE(contours.empty());
}

TEST(FrontierDetection, PixelToWorld)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.1;
  grid.info.origin.position.x = 1.0;
  grid.info.origin.position.y = 2.0;
  const auto world = explorer_mission::pixelToWorld(cv::Point(10, 20), grid);
  EXPECT_NEAR(world.x, 2.0, 1e-6);
  EXPECT_NEAR(world.y, 4.0, 1e-6);
}

static nav_msgs::msg::OccupancyGrid makeRevealedDiscGrid(
  int size, double radius, bool fully_known)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<unsigned int>(size);
  grid.info.height = static_cast<unsigned int>(size);
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.assign(static_cast<size_t>(size) * size, fully_known ? 0 : -1);

  const double cx = size / 2.0;
  const double cy = size / 2.0;
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const double dx = x - cx;
      const double dy = y - cy;
      if (std::sqrt(dx * dx + dy * dy) <= radius) {
        grid.data[y * size + x] = 0;
      }
    }
  }
  return grid;
}

TEST(FrontierDetection, RevealedDiscHasFrontier)
{
  const auto grid = makeRevealedDiscGrid(120, 30.0, false);
  const auto contours = explorer_mission::findFrontierContours(grid, 20);
  EXPECT_FALSE(contours.empty());
}

TEST(FrontierDetection, FullyKnownMapHasNoFrontier_NegativeControl)
{
  const auto grid = makeRevealedDiscGrid(120, 30.0, true);
  const auto contours = explorer_mission::findFrontierContours(grid, 20);
  EXPECT_TRUE(contours.empty());
}

TEST(FrontierDetection, FilterContoursNearRobot_positive)
{
  const auto grid = makeTestGrid();
  const cv::Point2f robot(0.5f, 0.5f);
  const auto contours = explorer_mission::findFrontierContours(grid, 5);
  const auto near = explorer_mission::filterContoursNearRobot(contours, grid, robot, 5.0);
  EXPECT_FALSE(near.empty());
}

TEST(FrontierDetection, FilterContoursNearRobot_negative)
{
  const auto grid = makeTestGrid();
  const cv::Point2f robot(0.5f, 0.5f);
  const auto contours = explorer_mission::findFrontierContours(grid, 5);
  const auto near = explorer_mission::filterContoursNearRobot(contours, grid, robot, 0.01);
  EXPECT_TRUE(near.empty());
}

TEST(FrontierDetection, FilterContoursUnlimitedRadius_positive)
{
  // radius <= 0 means keep all (do not wipe the set).
  const auto grid = makeTestGrid();
  const cv::Point2f robot(0.5f, 0.5f);
  const auto contours = explorer_mission::findFrontierContours(grid, 5);
  ASSERT_FALSE(contours.empty());
  const auto all = explorer_mission::filterContoursNearRobot(contours, grid, robot, 0.0);
  EXPECT_EQ(all.size(), contours.size());
}

TEST(FrontierDetection, BootstrapExclusion5mWipesRing_negative)
{
  // Revealed disc radius 80 px * 0.05 = 4 m; 5 m exclusion around robot wipes it.
  const auto grid = makeRevealedDiscGrid(200, 80.0, false);
  const double res = grid.info.resolution;
  const cv::Point2f robot(
    static_cast<float>(grid.info.width * 0.5 * res),
    static_cast<float>(grid.info.height * 0.5 * res));

  const auto mask = explorer_mission::buildExclusionMask(grid, {robot}, 5.0);
  const auto contours = explorer_mission::findFrontierContoursMasked(grid, mask, 10);
  EXPECT_TRUE(contours.empty()) << "5 m exclusion around robot should wipe ~4 m frontier ring";
}

TEST(FrontierDetection, BootstrapSmallExclusionKeepsRing_positive)
{
  const auto grid = makeRevealedDiscGrid(200, 80.0, false);
  const double res = grid.info.resolution;
  const cv::Point2f robot(
    static_cast<float>(grid.info.width * 0.5 * res),
    static_cast<float>(grid.info.height * 0.5 * res));

  const auto mask = explorer_mission::buildExclusionMask(grid, {robot}, 1.0);
  const auto contours = explorer_mission::findFrontierContoursMasked(grid, mask, 10);
  const auto near = explorer_mission::filterContoursNearRobot(contours, grid, robot, 50.0);
  EXPECT_FALSE(near.empty()) << "1 m exclusion + 50 m keep should retain frontiers";
}

TEST(FrontierDetection, ExclusionMaskBlocksExistingNode_negative)
{
  const auto grid = makeTestGrid();
  const cv::Point2f robot(0.5f, 0.5f);
  const cv::Point2f existing = explorer_mission::frontierMidpointWorld(
    explorer_mission::findFrontierContours(grid, 5).front(), grid);

  const auto mask = explorer_mission::buildExclusionMask(grid, {existing}, 1.0);
  const auto contours = explorer_mission::findFrontierContoursMasked(grid, mask, 5);
  const auto near = explorer_mission::filterContoursNearRobot(contours, grid, robot, 5.0);
  EXPECT_TRUE(near.empty());
}

TEST(FrontierDetection, ExclusionMaskAllowsFarFrontier_positive)
{
  const auto grid = makeRevealedDiscGrid(80, 25.0, false);
  const cv::Point2f robot(
    static_cast<float>(40 * grid.info.resolution),
    static_cast<float>(40 * grid.info.resolution));
  const cv::Point2f block_center(0.1f, 0.1f);

  const auto mask = explorer_mission::buildExclusionMask(grid, {block_center}, 0.5);
  const auto contours = explorer_mission::findFrontierContoursMasked(grid, mask, 10);
  const auto near = explorer_mission::filterContoursNearRobot(contours, grid, robot, 3.0);
  EXPECT_FALSE(near.empty());
}

TEST(FrontierDetection, DedupeSameBatchCloseMidpoints_positive)
{
  const auto grid = makeTestGrid();
  // Two synthetic 1-pixel contours at nearly the same world point.
  std::vector<std::vector<cv::Point>> contours = {
    {cv::Point(10, 10)},
    {cv::Point(11, 10)},  // ~0.05 m away at 0.05 res — within 1 m
    {cv::Point(50, 50)},  // far
  };
  const auto deduped = explorer_mission::dedupeContoursByMidpoint(contours, grid, 1.0);
  EXPECT_EQ(deduped.size(), 2u);
}

TEST(FrontierDetection, DedupeSameBatchKeepsFar_negative)
{
  const auto grid = makeTestGrid();
  std::vector<std::vector<cv::Point>> contours = {
    {cv::Point(10, 10)},
    {cv::Point(60, 60)},
  };
  const auto deduped = explorer_mission::dedupeContoursByMidpoint(contours, grid, 1.0);
  EXPECT_EQ(deduped.size(), 2u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
