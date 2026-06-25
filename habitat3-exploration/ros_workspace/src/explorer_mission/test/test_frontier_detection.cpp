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
  grid.data.assign(400, 100);  // occupied

  for (int y = 5; y < 15; ++y) {
    for (int x = 5; x < 15; ++x) {
      grid.data[y * 20 + x] = 0;  // free
    }
  }
  for (int y = 14; y < 20; ++y) {
    for (int x = 5; x < 15; ++x) {
      grid.data[y * 20 + x] = -1;  // unknown below free region
    }
  }
  return grid;
}

TEST(FrontierDetection, FindsContoursOnFreeUnknownBoundary)
{
  const auto grid = makeTestGrid();
  const auto contours = explorer_mission::findFrontierContours(grid, 5);
  EXPECT_FALSE(contours.empty());
}

TEST(FrontierDetection, FiltersByDistance)
{
  const auto grid = makeTestGrid();
  const auto contours = explorer_mission::findFrontierContours(grid, 5);
  const cv::Point2f robot(0.5f, 0.5f);
  const auto filtered = explorer_mission::filterFrontiers(
    contours, grid, robot, {}, 0.0, 10.0, 5);
  EXPECT_LE(filtered.size(), 5u);
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

// --- Incremental-reveal regression tests --------------------------------------
// These mirror the explored-map output the habitat engine now produces: an
// observed disc of FREE space surrounded by UNKNOWN (the rest of the navmesh
// the robot has not yet seen). This is the case that previously failed because
// the map contained no UNKNOWN cells at all.

// Build a square map that is FREE inside a centred disc of `radius` and
// UNKNOWN everywhere else (optionally walling the very border as OCCUPIED).
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
        grid.data[y * size + x] = 0;  // observed free
      }
    }
  }
  return grid;
}

TEST(FrontierDetection, RevealedDiscHasFrontier)
{
  // POSITIVE: an observed free disc inside unknown space has a free/unknown
  // boundary -> at least one frontier contour. This is the castle-scene case.
  const auto grid = makeRevealedDiscGrid(120, 30.0, /*fully_known=*/false);
  const auto contours = explorer_mission::findFrontierContours(grid, 20);
  EXPECT_FALSE(contours.empty()) << "revealed disc must produce a frontier";
}

TEST(FrontierDetection, FullyKnownMapHasNoFrontier_NegativeControl)
{
  // NEGATIVE CONTROL: with no UNKNOWN cells (the old full-navmesh behaviour),
  // frontier detection must find nothing — reproducing the "exploration
  // completed immediately" bug so a regression would fail this test.
  const auto grid = makeRevealedDiscGrid(120, 30.0, /*fully_known=*/true);
  const auto contours = explorer_mission::findFrontierContours(grid, 20);
  EXPECT_TRUE(contours.empty()) << "fully-known map must have no frontiers";
}

// A room whose floor is observed (FREE) with UNKNOWN beyond its top edge — the
// realistic shape of an explored area bounded by a sensor horizon / doorway.
// The frontier is the one-sided free/unknown boundary, so its centroid is
// offset from a robot standing deeper in the room.
static nav_msgs::msg::OccupancyGrid makeRevealedRoomGrid()
{
  const int size = 120;
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<unsigned int>(size);
  grid.info.height = static_cast<unsigned int>(size);
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.assign(static_cast<size_t>(size) * size, -1);  // unknown

  for (int y = 30; y < 90; ++y) {        // observed free band (rows 30..89)
    for (int x = 20; x < 100; ++x) {
      grid.data[y * size + x] = 0;
    }
  }
  return grid;
}

TEST(FrontierDetection, FilteredFrontierWithinRange)
{
  // Frontier lies along row ~30 (world y = 1.5 m); robot stands deep in the
  // room at row 85 (world y ~4.25 m), so the frontier is ~2.7 m away and must
  // survive the 1-5 m distance filter.
  const auto grid = makeRevealedRoomGrid();
  const auto contours = explorer_mission::findFrontierContours(grid, 20);
  ASSERT_FALSE(contours.empty());

  const cv::Point2f robot(60 * 0.05f, 85 * 0.05f);
  const auto filtered = explorer_mission::filterFrontiers(
    contours, grid, robot, {}, 1.0, 5.0, 5);
  EXPECT_FALSE(filtered.empty()) << "boundary frontier should pass the 1-5 m filter";
  EXPECT_LE(filtered.size(), 5u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
