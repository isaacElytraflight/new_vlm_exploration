#include "explorer_mission/frontier_detection.hpp"

#include <cstring>
#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace explorer_mission
{

std::vector<std::vector<cv::Point>> findFrontierContours(
  const nav_msgs::msg::OccupancyGrid & grid,
  int min_length_pixels)
{
  std::vector<std::vector<cv::Point>> result;

  if (grid.data.empty()) {
    return result;
  }

  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  if (width <= 0 || height <= 0) {
    return result;
  }

  cv::Mat occupancy_grid(height, width, CV_8SC1);
  std::memcpy(
    occupancy_grid.data, grid.data.data(),
    static_cast<size_t>(width) * static_cast<size_t>(height) * sizeof(int8_t));

  cv::Mat is_free;
  cv::compare(occupancy_grid, cv::Scalar(0), is_free, cv::CMP_EQ);

  cv::Mat is_unknown;
  cv::compare(occupancy_grid, cv::Scalar(-1), is_unknown, cv::CMP_EQ);

  cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  cv::Mat has_unknown_neighbor;
  cv::dilate(is_unknown, has_unknown_neighbor, kernel);

  cv::Mat frontier_mask;
  cv::bitwise_and(is_free, has_unknown_neighbor, frontier_mask);
  frontier_mask.convertTo(frontier_mask, CV_8U, 255.0);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(frontier_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  for (const auto & contour : contours) {
    if (static_cast<int>(contour.size()) > min_length_pixels) {
      result.push_back(contour);
    }
  }

  return result;
}

cv::Point2f pixelToWorld(
  const cv::Point & pixel,
  const nav_msgs::msg::OccupancyGrid & grid)
{
  const double resolution = grid.info.resolution;
  const cv::Point2f origin(
    static_cast<float>(grid.info.origin.position.x),
    static_cast<float>(grid.info.origin.position.y));
  return cv::Point2f(
    static_cast<float>(pixel.x * resolution + origin.x),
    static_cast<float>(pixel.y * resolution + origin.y));
}

cv::Point2f frontierMidpointWorld(
  const std::vector<cv::Point> & contour,
  const nav_msgs::msg::OccupancyGrid & grid)
{
  if (contour.empty()) {
    return cv::Point2f(0.0f, 0.0f);
  }

  double x = 0.0;
  double y = 0.0;
  for (const auto & point : contour) {
    x += point.x;
    y += point.y;
  }
  const double avgx = x / static_cast<double>(contour.size());
  const double avgy = y / static_cast<double>(contour.size());
  return pixelToWorld(cv::Point2f(static_cast<float>(avgx), static_cast<float>(avgy)), grid);
}

double euclideanDist(const cv::Point2f & a, const cv::Point2f & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<FrontierWithDistance> filterFrontiers(
  const std::vector<std::vector<cv::Point>> & contours,
  const nav_msgs::msg::OccupancyGrid & grid,
  const cv::Point2f & robot_pos,
  const std::vector<cv::Point2f> & blacklist,
  double min_radius_m,
  double max_radius_m,
  size_t max_frontiers)
{
  std::vector<FrontierWithDistance> frontiers_with_dist;

  if (robot_pos.x == -1000.0f && robot_pos.y == -1000.0f) {
    return frontiers_with_dist;
  }

  uint8_t idx = 0;
  for (const auto & contour : contours) {
    const cv::Point2f midpoint_world = frontierMidpointWorld(contour, grid);
    const double distance_to_robot = euclideanDist(robot_pos, midpoint_world);

    if (distance_to_robot < min_radius_m || distance_to_robot > max_radius_m) {
      ++idx;
      continue;
    }

    bool is_in_blacklist = false;
    for (const auto & point : blacklist) {
      if (euclideanDist(point, midpoint_world) < 0.5) {
        is_in_blacklist = true;
        break;
      }
    }

    if (!is_in_blacklist) {
      frontiers_with_dist.push_back({distance_to_robot, midpoint_world, contour, idx});
    }
    ++idx;
  }

  std::sort(
    frontiers_with_dist.begin(), frontiers_with_dist.end(),
    [](const FrontierWithDistance & a, const FrontierWithDistance & b) {
      return a.distance < b.distance;
    });

  if (frontiers_with_dist.size() > max_frontiers) {
    frontiers_with_dist.resize(max_frontiers);
  }

  return frontiers_with_dist;
}

}  // namespace explorer_mission
