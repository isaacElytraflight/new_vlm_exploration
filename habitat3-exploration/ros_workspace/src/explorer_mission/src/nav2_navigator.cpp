#include "explorer_mission/nav2_navigator.hpp"

#include <future>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace explorer_mission
{

Nav2Navigator::Nav2Navigator(rclcpp::Node * node, const std::string & action_name)
: node_(node),
  client_(rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, action_name))
{
}

bool Nav2Navigator::waitForServer(std::chrono::seconds timeout) const
{
  return client_->wait_for_action_server(timeout);
}

void Nav2Navigator::cancel()
{
  if (active_goal_handle_) {
    client_->async_cancel_goal(active_goal_handle_);
    active_goal_handle_.reset();
  }
}

bool Nav2Navigator::navigateToPose(
  double x, double y, double yaw_rad,
  const std::string & map_frame,
  double total_timeout_s,
  double stuck_timeout_s,
  double stuck_distance_m,
  const std::function<bool()> & tick)
{
  (void)stuck_distance_m;
  last_error_.clear();
  resetProgress();

  if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
    last_error_ = "navigate_to_pose action server not available";
    return false;
  }

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = map_frame;
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  goal.pose.pose.orientation = tf2::toMsg(q);

  auto send_future = client_->async_send_goal(goal);
  if (send_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
    last_error_ = "timed out sending NavigateToPose goal";
    return false;
  }
  active_goal_handle_ = send_future.get();
  if (!active_goal_handle_) {
    last_error_ = "NavigateToPose goal rejected";
    return false;
  }

  auto result_future = client_->async_get_result(active_goal_handle_);
  const auto start = node_->now();

  while (rclcpp::ok()) {
    if (tick && !tick()) {
      cancel();
      last_error_ = "navigation aborted by caller";
      active_goal_handle_.reset();
      return false;
    }

    if (isStuck(stuck_timeout_s)) {
      cancel();
      last_error_ = "NavigateToPose stuck timeout";
      active_goal_handle_.reset();
      return false;
    }

    if ((node_->now() - start).seconds() >= total_timeout_s) {
      cancel();
      last_error_ = "NavigateToPose total timeout";
      active_goal_handle_.reset();
      return false;
    }

    if (result_future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
      break;
    }
  }

  if (result_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    cancel();
    last_error_ = "NavigateToPose result not ready";
    active_goal_handle_.reset();
    return false;
  }

  const auto wrapped = result_future.get();
  active_goal_handle_.reset();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    last_error_ = "NavigateToPose failed with code " +
      std::to_string(static_cast<int>(wrapped.code));
    return false;
  }
  return true;
}

void Nav2Navigator::noteProgress(
  double x, double y, double stuck_distance_m, double /*stuck_timeout_s*/)
{
  if (!have_progress_anchor_) {
    progress_x_ = x;
    progress_y_ = y;
    have_progress_anchor_ = true;
    last_progress_time_ = node_->now();
    return;
  }
  const double dx = x - progress_x_;
  const double dy = y - progress_y_;
  if (std::hypot(dx, dy) >= stuck_distance_m) {
    progress_x_ = x;
    progress_y_ = y;
    last_progress_time_ = node_->now();
  }
}

bool Nav2Navigator::isStuck(double stuck_timeout_s) const
{
  if (!have_progress_anchor_) {
    return false;
  }
  return (node_->now() - last_progress_time_).seconds() > stuck_timeout_s;
}

void Nav2Navigator::resetProgress()
{
  have_progress_anchor_ = false;
}

}  // namespace explorer_mission
