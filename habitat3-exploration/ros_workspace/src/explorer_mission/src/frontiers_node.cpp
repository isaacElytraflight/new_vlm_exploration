#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <explorer_msgs/msg/frontier.hpp>
#include <explorer_msgs/msg/frontier_array.hpp>
#include <explorer_msgs/msg/frontier_blacklist.hpp>

#include "explorer_mission/frontier_detection.hpp"

class FrontiersNode : public rclcpp::Node
{
public:
  FrontiersNode()
  : Node("frontiers"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    grid_topic_ = declare_parameter<std::string>("grid_topic", "/grid_map");
    subscribe_costmap_updates_ = declare_parameter<bool>("subscribe_costmap_updates", true);

    grid_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic_, rclcpp::QoS(1),
      std::bind(&FrontiersNode::gridMapCb, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Subscribing to grid topic \"%s\"", grid_topic_.c_str());

    if (subscribe_costmap_updates_) {
      const std::string updates_topic = grid_topic_ + "_updates";
      costmap_updates_sub_ = create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        updates_topic, rclcpp::QoS(20),
        std::bind(&FrontiersNode::costmapUpdatesCb, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Also subscribing to \"%s\"", updates_topic.c_str());
    }

    frontier_blacklist_sub_ = create_subscription<explorer_msgs::msg::FrontierBlacklist>(
      "/frontier_blacklist", rclcpp::QoS(1),
      std::bind(&FrontiersNode::frontierBlacklistCb, this, std::placeholders::_1));

    frontiers_pub_ = create_publisher<explorer_msgs::msg::FrontierArray>("frontiers/frontiers", 10);
    filtered_frontiers_pub_ = create_publisher<explorer_msgs::msg::FrontierArray>(
      "frontiers/filtered_frontiers", 10);
  }

private:
  cv::Point2f getRobotPositionInMapFrame()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      return cv::Point2f(
        static_cast<float>(transform.transform.translation.x),
        static_cast<float>(transform.transform.translation.y));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Failed to lookup transform from %s to %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return cv::Point2f(-1000.0f, -1000.0f);
    }
  }

  void refreshFrontiersFromGrid()
  {
    all_frontiers_ = explorer_mission::findFrontierContours(grid_map_msg_);
    publishFrontiers();

    const cv::Point2f robot_pos = getRobotPositionInMapFrame();
    filtered_frontiers_ = explorer_mission::filterFrontiers(
      all_frontiers_, grid_map_msg_, robot_pos, frontier_blacklist_, 1.0, 5.0, 5);
    publishFilteredFrontiers();
  }

  void publishFilteredFrontiers()
  {
    explorer_msgs::msg::FrontierArray frontier_array;
    frontier_array.header.stamp = now();
    frontier_array.header.frame_id = map_frame_;
    frontier_array.total_count = static_cast<int32_t>(filtered_frontiers_.size());

    for (const auto & frontier : filtered_frontiers_) {
      explorer_msgs::msg::Frontier new_frontier;
      new_frontier.header = frontier_array.header;

      for (const cv::Point & pixel_point : frontier.contour) {
        geometry_msgs::msg::Point point;
        point.x = pixel_point.x;
        point.y = pixel_point.y;
        point.z = 0.0;
        new_frontier.points.push_back(point);
      }

      new_frontier.distance = frontier.distance;
      new_frontier.midpoint.x = frontier.midpoint_world.x;
      new_frontier.midpoint.y = frontier.midpoint_world.y;
      new_frontier.midpoint.z = 0.0;
      new_frontier.id = frontier.id;
      frontier_array.frontiers.push_back(new_frontier);
    }

    filtered_frontiers_pub_->publish(frontier_array);
  }

  void publishFrontiers()
  {
    explorer_msgs::msg::FrontierArray frontier_array;
    frontier_array.header.stamp = now();
    frontier_array.header.frame_id = map_frame_;
    frontier_array.total_count = static_cast<int32_t>(all_frontiers_.size());

    uint8_t idx = 0;
    for (const auto & contour : all_frontiers_) {
      explorer_msgs::msg::Frontier frontier;
      frontier.header = frontier_array.header;

      for (const cv::Point & pixel_point : contour) {
        geometry_msgs::msg::Point point;
        point.x = pixel_point.x;
        point.y = pixel_point.y;
        point.z = 0.0;
        frontier.points.push_back(point);
      }

      frontier.distance = 0.0;
      frontier.midpoint.x = 0.0;
      frontier.midpoint.y = 0.0;
      frontier.midpoint.z = 0.0;
      frontier.id = static_cast<uint8_t>(idx & 0xFF);
      frontier_array.frontiers.push_back(frontier);
      ++idx;
    }

    frontiers_pub_->publish(frontier_array);
  }

  void gridMapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_map_msg_ = *msg;
    refreshFrontiersFromGrid();
  }

  void costmapUpdatesCb(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
  {
    if (grid_map_msg_.data.empty()) {
      return;
    }

    const unsigned int map_w = grid_map_msg_.info.width;
    const unsigned int map_h = grid_map_msg_.info.height;
    if (map_w == 0 || map_h == 0) {
      return;
    }
    if (msg->width <= 0 || msg->height <= 0) {
      return;
    }

    const size_t expected = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (msg->data.size() != expected) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "costmap_updates: data size mismatch (got %zu, expected %zu)",
        msg->data.size(), expected);
      return;
    }
    if (msg->x < 0 || msg->y < 0) {
      return;
    }

    const unsigned int ux = static_cast<unsigned int>(msg->x);
    const unsigned int uy = static_cast<unsigned int>(msg->y);
    const unsigned int uw = static_cast<unsigned int>(msg->width);
    const unsigned int uh = static_cast<unsigned int>(msg->height);
    if (ux + uw > map_w || uy + uh > map_h) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "costmap_updates: patch out of map bounds");
      return;
    }

    for (unsigned int j = 0; j < uh; ++j) {
      for (unsigned int i = 0; i < uw; ++i) {
        const size_t map_idx = (uy + j) * map_w + (ux + i);
        const size_t patch_idx = j * uw + i;
        grid_map_msg_.data[map_idx] = msg->data[patch_idx];
      }
    }
    grid_map_msg_.header = msg->header;
    refreshFrontiersFromGrid();
  }

  void frontierBlacklistCb(const explorer_msgs::msg::FrontierBlacklist::SharedPtr msg)
  {
    frontier_blacklist_.clear();
    for (const auto & point : msg->points) {
      frontier_blacklist_.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
    }
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string grid_topic_;
  bool subscribe_costmap_updates_{true};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::OccupancyGrid grid_map_msg_;
  std::vector<std::vector<cv::Point>> all_frontiers_;
  std::vector<explorer_mission::FrontierWithDistance> filtered_frontiers_;
  std::vector<cv::Point2f> frontier_blacklist_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_sub_;
  rclcpp::Subscription<explorer_msgs::msg::FrontierBlacklist>::SharedPtr frontier_blacklist_sub_;
  rclcpp::Publisher<explorer_msgs::msg::FrontierArray>::SharedPtr frontiers_pub_;
  rclcpp::Publisher<explorer_msgs::msg::FrontierArray>::SharedPtr filtered_frontiers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontiersNode>());
  rclcpp::shutdown();
  return 0;
}
