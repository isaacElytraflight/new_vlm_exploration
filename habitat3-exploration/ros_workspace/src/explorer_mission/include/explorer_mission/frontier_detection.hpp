#pragma once

#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>

namespace explorer_mission
{

struct FrontierWithDistance
{
  double distance{0.0};
  cv::Point2f midpoint_world;
  std::vector<cv::Point> contour;
  uint8_t id{0};
};

/// Extract frontier contours from an occupancy grid (OpenCV contour logic from frontiers.cpp).
std::vector<std::vector<cv::Point>> findFrontierContours(
  const nav_msgs::msg::OccupancyGrid & grid,
  int min_length_pixels = 20);

cv::Point2f pixelToWorld(
  const cv::Point & pixel,
  const nav_msgs::msg::OccupancyGrid & grid);

cv::Point2f frontierMidpointWorld(
  const std::vector<cv::Point> & contour,
  const nav_msgs::msg::OccupancyGrid & grid);

double euclideanDist(const cv::Point2f & a, const cv::Point2f & b);

/// Filter contours by distance to robot and spatial blacklist; sort by distance, keep top N.
std::vector<FrontierWithDistance> filterFrontiers(
  const std::vector<std::vector<cv::Point>> & contours,
  const nav_msgs::msg::OccupancyGrid & grid,
  const cv::Point2f & robot_pos,
  const std::vector<cv::Point2f> & blacklist,
  double min_radius_m = 1.0,
  double max_radius_m = 5.0,
  size_t max_frontiers = 5);

}  // namespace explorer_mission
