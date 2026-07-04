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
  cv::Mat allowed;
  allowed.create(grid.info.height, grid.info.width, CV_8U);
  allowed.setTo(255);
  return findFrontierContoursMasked(grid, allowed, min_length_pixels);
}

cv::Mat buildExclusionMask(
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<cv::Point2f> & exclusion_centers_world,
  double exclusion_radius_m)
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  cv::Mat mask(height, width, CV_8U, cv::Scalar(255));

  if (width <= 0 || height <= 0 || exclusion_radius_m <= 0.0) {
    return mask;
  }

  const double resolution = grid.info.resolution;
  const int radius_px = std::max(
    1, static_cast<int>(std::ceil(exclusion_radius_m / resolution)));
  const float origin_x = static_cast<float>(grid.info.origin.position.x);
  const float origin_y = static_cast<float>(grid.info.origin.position.y);

  for (const auto & center : exclusion_centers_world) {
    const int col = static_cast<int>(std::round((center.x - origin_x) / resolution));
    const int row = static_cast<int>(std::round((center.y - origin_y) / resolution));
    cv::circle(mask, cv::Point(col, row), radius_px, cv::Scalar(0), cv::FILLED);
  }
  return mask;
}

std::vector<std::vector<cv::Point>> findFrontierContoursMasked(
  const nav_msgs::msg::OccupancyGrid & grid,
  const cv::Mat & allowed_mask,
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

  if (!allowed_mask.empty() &&
    allowed_mask.rows == height && allowed_mask.cols == width)
  {
    cv::bitwise_and(frontier_mask, allowed_mask, frontier_mask);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(frontier_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  for (const auto & contour : contours) {
    if (static_cast<int>(contour.size()) > min_length_pixels) {
      result.push_back(contour);
    }
  }

  return result;
}

std::vector<std::vector<cv::Point>> filterContoursNearRobot(
  const std::vector<std::vector<cv::Point>> & contours,
  const nav_msgs::msg::OccupancyGrid & grid,
  const cv::Point2f & robot_pos,
  double radius_m)
{
  std::vector<std::vector<cv::Point>> result;
  if (radius_m <= 0.0) {
    return result;
  }

  for (const auto & contour : contours) {
    const cv::Point2f midpoint = frontierMidpointWorld(contour, grid);
    if (euclideanDist(robot_pos, midpoint) <= radius_m) {
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

}  // namespace explorer_mission
