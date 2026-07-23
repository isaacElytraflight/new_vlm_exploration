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

/// Extract frontier contours from an occupancy grid (free cells bordering unknown).
std::vector<std::vector<cv::Point>> findFrontierContours(
  const nav_msgs::msg::OccupancyGrid & grid,
  int min_length_pixels = 20);

/// Build a binary mask (255=allowed) with circular exclusions around centers.
cv::Mat buildExclusionMask(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<cv::Point2f> & exclusion_centers_world,
  double exclusion_radius_m);

/// Run frontier detection on grid cells where mask is non-zero.
std::vector<std::vector<cv::Point>> findFrontierContoursMasked(
  const nav_msgs::msg::OccupancyGrid & grid,
  const cv::Mat & allowed_mask,
  int min_length_pixels = 20);

/// Keep contours whose world centroid is within radius_m of robot_pos.
/// If ``radius_m <= 0``, keep all contours (no distance filter).
std::vector<std::vector<cv::Point>> filterContoursNearRobot(
  const std::vector<std::vector<cv::Point>> & contours,
  const nav_msgs::msg::OccupancyGrid & grid,
  const cv::Point2f & robot_pos,
  double radius_m);

cv::Point2f pixelToWorld(
  const cv::Point & pixel,
  const nav_msgs::msg::OccupancyGrid & grid);

cv::Point2f frontierMidpointWorld(
  const std::vector<cv::Point> & contour,
  const nav_msgs::msg::OccupancyGrid & grid);

double euclideanDist(const cv::Point2f & a, const cv::Point2f & b);

/// Drop later contours whose midpoint is within radius_m of an earlier kept midpoint.
std::vector<std::vector<cv::Point>> dedupeContoursByMidpoint(
  const std::vector<std::vector<cv::Point>> & contours,
  const nav_msgs::msg::OccupancyGrid & grid,
  double radius_m);

}  // namespace explorer_mission
