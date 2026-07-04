#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace explorer_mission
{

class Nav2Navigator
{
public:
  explicit Nav2Navigator(
    rclcpp::Node * node,
    const std::string & action_name = "navigate_to_pose");

  bool waitForServer(std::chrono::seconds timeout) const;

  bool navigateToPose(
    double x, double y, double yaw_rad,
    const std::string & map_frame,
    double total_timeout_s = 120.0,
    double stuck_timeout_s = 60.0,
    double stuck_distance_m = 0.1,
    const std::function<bool()> & tick = {});

  void cancel();

  void noteProgress(double x, double y, double stuck_distance_m, double stuck_timeout_s);
  bool isStuck(double stuck_timeout_s) const;
  void resetProgress();

  std::string lastError() const {return last_error_;}

private:
  rclcpp::Node * node_{nullptr};
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr active_goal_handle_;
  std::string last_error_;
  bool have_progress_anchor_{false};
  double progress_x_{0.0};
  double progress_y_{0.0};
  rclcpp::Time last_progress_time_;
};

}  // namespace explorer_mission
