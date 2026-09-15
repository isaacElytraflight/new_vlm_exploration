#pragma once

#include <chrono>
#include <functional>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>

#include "explorer_mission/stuck_progress.hpp"

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
    double total_timeout_s = kNavTotalTimeoutS,
    double stuck_timeout_s = kNavStuckTimeoutS,
    double stuck_distance_m = kNavStuckDistanceM,
    const std::function<bool()> & tick = {},
    double goal_accept_radius_m = 1.0);

  void cancel();

  /// Theoretical reachability via ComputePathToPose (no controller execution).
  bool computePathExists(
    double x, double y, double yaw_rad,
    const std::string & map_frame,
    double timeout_s = 15.0);

  void noteProgress(double x, double y, double stuck_distance_m, double stuck_timeout_s);
  bool isStuck(double stuck_timeout_s) const;
  void resetProgress();
  bool withinGoalAcceptRadius(double goal_x, double goal_y, double radius_m) const;

  std::string lastError() const {return last_error_;}
  uint16_t lastErrorCode() const {return last_error_code_;}

private:
  rclcpp::Node * node_{nullptr};
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr plan_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr active_goal_handle_;
  std::string last_error_;
  uint16_t last_error_code_{0};
  StuckProgressTracker stuck_progress_;
  double current_x_{0.0};
  double current_y_{0.0};
  bool have_current_pose_{false};
};

}  // namespace explorer_mission
