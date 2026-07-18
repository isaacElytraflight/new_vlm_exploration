#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

#include <explorer_msgs/action/discrete_move.hpp>
#include <explorer_msgs/action/rotate360.hpp>
#include <explorer_msgs/msg/exploration_status.hpp>
#include <explorer_msgs/msg/frontier_openness_scores.hpp>
#include <explorer_msgs/msg/frontier_tree.hpp>
#include <explorer_msgs/msg/frontier_views.hpp>

#include "explorer_mission/discrete_navigator.hpp"
#include "explorer_mission/frontier_detection.hpp"
#include "explorer_mission/frontier_tree.hpp"
#include "explorer_mission/nav2_navigator.hpp"

using DiscreteMove = explorer_msgs::action::DiscreteMove;
using Rotate360 = explorer_msgs::action::Rotate360;

using cv::Point2f;

class ExploreNode : public rclcpp::Node
{
public:
  ExploreNode()
  : Node("explore"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    navigation_mode_ = declare_parameter<std::string>("navigation_mode", "nav2");
    frontier_detection_radius_ = declare_parameter<double>("frontier_detection_radius", 5.0);
    min_contour_pixels_ = declare_parameter<int>("min_contour_pixels", 15);
    vlm_scores_timeout_s_ = declare_parameter<double>("vlm_scores_timeout_s", 120.0);
    publish_debug_topics_ = declare_parameter<bool>("publish_debug_topics", false);

    if (navigation_mode_ == "nav2") {
      nav2_navigator_ = std::make_unique<explorer_mission::Nav2Navigator>(this);
    }

    discrete_move_client_ = rclcpp_action::create_client<DiscreteMove>(
      this, "/movement/discrete_move");
    rotate_client_ = rclcpp_action::create_client<Rotate360>(this, "rotate_360");

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/grid_map", rclcpp::QoS(1),
      std::bind(&ExploreNode::gridCb, this, std::placeholders::_1));

    vlm_scores_sub_ = create_subscription<explorer_msgs::msg::FrontierOpennessScores>(
      "exploration/vlm/scores", rclcpp::QoS(1),
      std::bind(&ExploreNode::vlmScoresCb, this, std::placeholders::_1));

    const auto latched = rclcpp::QoS(1).transient_local();
    tree_pub_ = create_publisher<explorer_msgs::msg::FrontierTree>(
      "exploration/frontier_tree", latched);
    status_pub_ = create_publisher<explorer_msgs::msg::ExplorationStatus>(
      "exploration/status", latched);
    vlm_views_pub_ = create_publisher<explorer_msgs::msg::FrontierViews>(
      "exploration/vlm/views", rclcpp::QoS(1));

    if (publish_debug_topics_) {
      debug_events_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "exploration/debug/events", rclcpp::QoS(1));
      debug_vlm_batch_pub_ = create_publisher<explorer_msgs::msg::FrontierViews>(
        "exploration/debug/last_vlm_batch", rclcpp::QoS(1));
    }

    RCLCPP_INFO(get_logger(), "Explore node initializing (frontier tree mode)...");
  }

  bool waitForDependencies()
  {
    RCLCPP_INFO(get_logger(), "Waiting for required action servers...");

    if (navigation_mode_ == "nav2" && nav2_navigator_) {
      if (!nav2_navigator_->waitForServer(std::chrono::seconds(90))) {
        RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
        return false;
      }
    }
    if (!discrete_move_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "discrete_move action server not available");
      return false;
    }
    if (!rotate_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "rotate_360 action server not available");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Waiting for map (/grid_map + map TF)...");
    const auto start_wait = now();
    bool bootstrap_sent = false;
    while (rclcpp::ok() &&
      (now() - start_wait).seconds() < 90.0)
    {
      updateRobotPoseFromTf();
      {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (have_grid_ && tf_received_) {
          break;
        }
      }
      if (!bootstrap_sent && (now() - start_wait).seconds() > 8.0) {
        bootstrap_sent = true;
        DiscreteMove::Goal goal;
        goal.direction = DiscreteMove::Goal::FORWARD;
        goal.steps = 2;
        auto future = discrete_move_client_->async_send_goal(goal);
        if (future.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
          auto handle = future.get();
          if (handle) {
            auto result_future = discrete_move_client_->async_get_result(handle);
            result_future.wait_for(std::chrono::seconds(60));
          }
        }
        RCLCPP_INFO(get_logger(), "Sent SLAM bootstrap forward move");
      }
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      if (!have_grid_ || !tf_received_) {
        RCLCPP_ERROR(
          get_logger(),
          "Timed out waiting for map (/grid_map=%s, map TF=%s)",
          have_grid_ ? "ok" : "missing",
          tf_received_ ? "ok" : "missing");
        return false;
      }
    }

    RCLCPP_INFO(get_logger(), "All dependencies are ready.");
    return true;
  }

  void startExploration()
  {
    RCLCPP_INFO(get_logger(), "Exploration started");
    updateRobotPoseFromTf();
    publishPhase("scanning", 0, 0, false, "initial scan");
    performScanIfNeeded();
    tree_.createRoot(current_pos_);
    tree_.setCurrentNodeId(tree_.rootId());
    publishTree();
    publishPhase("idle", tree_.currentNodeId(), 0, false, "root created");

    while (rclcpp::ok()) {
      updateRobotPoseFromTf();
      performScanIfNeeded();

      explorer_mission::TreeNode * current = tree_.find(tree_.currentNodeId());
      if (!current) {
        RCLCPP_ERROR(get_logger(), "Current tree node missing; stopping.");
        break;
      }

      if (current->children_ids.empty()) {
        if (!detectAndRateChildren()) {
          RCLCPP_WARN(get_logger(), "Frontier detection/VLM rating failed; retrying.");
          rclcpp::sleep_for(std::chrono::seconds(1));
          continue;
        }
        current = tree_.find(tree_.currentNodeId());
        if (!current) {
          break;
        }
      }

      if (current->children_ids.empty() || !tree_.hasUnexploredChildren(current->id)) {
        tree_.markFullyExplored(current->id);
        publishTree();
        if (!tree_.hasUnexploredNodesExcluding(current->id, tree_.rootId())) {
          publishPhase(
            "complete", current->id, 0, true,
            "exploration complete (in place)");
          break;
        }

        if (current->parent_id < 0) {
          publishPhase("complete", current->id, 0, true, "root exhausted");
          break;
        }

        const uint32_t parent_id = static_cast<uint32_t>(current->parent_id);
        explorer_mission::TreeNode * parent = tree_.find(parent_id);
        if (!parent) {
          RCLCPP_ERROR(get_logger(), "Parent node %u missing; stopping.", parent_id);
          break;
        }

        publishPhase(
          "backtracking", current->id, parent_id, false,
          "returning to parent");
        if (!navigateToPosition(parent->position, parent->position)) {
          RCLCPP_WARN(get_logger(), "Backtrack navigation failed toward parent %u.", parent_id);
        }
        tree_.setCurrentNodeId(parent_id);
        publishTree();
        continue;
      }

      const auto child_id = tree_.selectNextChild(current->id);
      if (!child_id.has_value()) {
        continue;
      }

      explorer_mission::TreeNode * child = tree_.find(*child_id);
      if (!child) {
        continue;
      }

      publishPhase(
        "navigating", current->id, *child_id, false,
        "selected child score=" + std::to_string(child->openness_score));

      const bool ok = navigateToPosition(child->position, child->position);
      if (!ok) {
        tree_.markFullyExplored(*child_id);
        publishTree();
        publishPhase(
          "backtracking", current->id, current->id, false,
          "nav failed; returning to parent scan node");
        if (!navigateToPosition(current->position, current->position)) {
          RCLCPP_WARN(get_logger(), "Return to parent after nav failure also failed.");
        }
        continue;
      }

      tree_.setCurrentNodeId(*child_id);
      publishTree();
      publishPhase("scanning", *child_id, 0, false, "arrived at child");
    }

    RCLCPP_INFO(get_logger(), "Exploration completed");
  }

private:
  void gridCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    latest_grid_ = *msg;
    have_grid_ = true;
  }

  void vlmScoresCb(const explorer_msgs::msg::FrontierOpennessScores::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(scores_mutex_);
    latest_scores_ = *msg;
    scores_received_ = true;
  }

  void publishTree()
  {
    const rclcpp::Time stamp = now();
    const int64_t total_ns = stamp.nanoseconds();
    const int32_t sec = static_cast<int32_t>(total_ns / 1000000000LL);
    const uint32_t nsec = static_cast<uint32_t>(total_ns % 1000000000LL);
    tree_pub_->publish(tree_.toMsg(map_frame_, sec, nsec));
  }

  void publishPhase(
    const std::string & phase,
    uint32_t current_id,
    uint32_t target_id,
    bool complete,
    const std::string & detail)
  {
    explorer_msgs::msg::ExplorationStatus status;
    const auto stamp = now();
    status.header.stamp = stamp;
    status.header.frame_id = map_frame_;
    status.phase = phase;
    status.current_node_id = current_id;
    status.target_node_id = target_id;
    status.exploration_complete = complete;
    status.detail = detail;
    status_pub_->publish(status);

    if (publish_debug_topics_ && debug_events_pub_) {
      diagnostic_msgs::msg::DiagnosticArray arr;
      arr.header = status.header;
      diagnostic_msgs::msg::DiagnosticStatus diag;
      diag.name = "exploration";
      diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag.message = phase + ": " + detail;
      arr.status.push_back(diag);
      debug_events_pub_->publish(arr);
    }
  }

  bool detectAndRateChildren()
  {
    nav_msgs::msg::OccupancyGrid grid;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      if (!have_grid_) {
        RCLCPP_WARN(get_logger(), "No /grid_map yet; skipping detection.");
        return false;
      }
      grid = latest_grid_;
    }

    publishPhase(
      "detecting", tree_.currentNodeId(), 0, false,
      "on-demand frontier detection");

    const auto exclusion_centers = tree_.allNodePositions();
    const cv::Mat mask = explorer_mission::buildExclusionMask(
      grid, exclusion_centers, frontier_detection_radius_);
    auto contours = explorer_mission::findFrontierContoursMasked(
      grid, mask, min_contour_pixels_);
    contours = explorer_mission::filterContoursNearRobot(
      contours, grid, current_pos_, frontier_detection_radius_);

    std::vector<uint32_t> new_child_ids;
    for (const auto & contour : contours) {
      const cv::Point2f midpoint = explorer_mission::frontierMidpointWorld(contour, grid);
      const uint32_t child_id = tree_.addChild(
        tree_.currentNodeId(), midpoint, explorer_mission::kOpennessNotRated, false);
      if (child_id != std::numeric_limits<uint32_t>::max()) {
        new_child_ids.push_back(child_id);
      }
    }
    publishTree();

    if (new_child_ids.empty()) {
      publishPhase(
        "selecting", tree_.currentNodeId(), 0, false,
        "no new frontiers detected");
      return true;
    }

    if (cached_images_.empty()) {
      RCLCPP_WARN(get_logger(), "No cached scan images; marking new children blocked.");
      for (uint32_t id : new_child_ids) {
        tree_.setOpennessScore(id, 0);
      }
      publishTree();
      return true;
    }

    auto views = buildFrontierViews(new_child_ids);
    if (views.frontier_ids.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to match images to frontiers.");
      for (uint32_t id : new_child_ids) {
        tree_.setOpennessScore(id, 0);
      }
      publishTree();
      return true;
    }

    publishPhase(
      "awaiting_vlm", tree_.currentNodeId(), 0, false,
      "rating " + std::to_string(views.frontier_ids.size()) + " frontiers");
    if (!waitForVlmScores(views.frontier_ids)) {
      RCLCPP_WARN(get_logger(), "VLM scores timeout; marking pending children blocked.");
      for (uint32_t id : views.frontier_ids) {
        tree_.setOpennessScore(id, 0);
      }
      publishTree();
      return true;
    }

    applyLatestScores();
    publishTree();
    publishPhase(
      "selecting", tree_.currentNodeId(), 0, false,
      "VLM scores applied");
    return true;
  }

  explorer_msgs::msg::FrontierViews buildFrontierViews(
    const std::vector<uint32_t> & child_ids)
  {
    explorer_msgs::msg::FrontierViews msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();

    if (cached_images_.empty() || cached_orientations_.empty()) {
      return msg;
    }

    updateRobotPoseFromTf();
    for (uint32_t child_id : child_ids) {
      const explorer_mission::TreeNode * node = tree_.find(child_id);
      if (!node) {
        continue;
      }
      const double dx = node->position.x - current_pos_.x;
      const double dy = node->position.y - current_pos_.y;
      double target_yaw_deg = std::atan2(dy, dx) * 180.0 / M_PI;
      while (target_yaw_deg < 0.0) {
        target_yaw_deg += 360.0;
      }
      while (target_yaw_deg >= 360.0) {
        target_yaw_deg -= 360.0;
      }

      size_t best_idx = 0;
      double best_diff = angularDifference(target_yaw_deg, cached_orientations_[0]);
      for (size_t i = 1; i < cached_orientations_.size(); ++i) {
        const double diff = angularDifference(target_yaw_deg, cached_orientations_[i]);
        if (diff < best_diff) {
          best_diff = diff;
          best_idx = i;
        }
      }

      msg.images.push_back(cached_images_[best_idx]);
      msg.frontier_ids.push_back(child_id);
    }
    return msg;
  }

  bool waitForVlmScores(const std::vector<uint32_t> & expected_ids)
  {
    std::set<uint32_t> expected(expected_ids.begin(), expected_ids.end());
    {
      std::lock_guard<std::mutex> lock(scores_mutex_);
      scores_received_ = false;
    }

    explorer_msgs::msg::FrontierViews views = buildFrontierViews(expected_ids);
    vlm_views_pub_->publish(views);
    if (publish_debug_topics_ && debug_vlm_batch_pub_) {
      debug_vlm_batch_pub_->publish(views);
    }

    const auto start = now();
    while (rclcpp::ok() && (now() - start).seconds() < vlm_scores_timeout_s_) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      std::lock_guard<std::mutex> lock(scores_mutex_);
      if (!scores_received_) {
        continue;
      }
      if (latest_scores_.frontier_ids.size() != latest_scores_.scores.size()) {
        continue;
      }
      std::set<uint32_t> received(
        latest_scores_.frontier_ids.begin(), latest_scores_.frontier_ids.end());
      if (received == expected) {
        return true;
      }
    }
    return false;
  }

  void applyLatestScores()
  {
    std::lock_guard<std::mutex> lock(scores_mutex_);
    for (size_t i = 0; i < latest_scores_.frontier_ids.size(); ++i) {
      if (i >= latest_scores_.scores.size()) {
        break;
      }
      tree_.setOpennessScore(
        latest_scores_.frontier_ids[i], latest_scores_.scores[i]);
    }
  }

  static double angularDifference(double angle1, double angle2)
  {
    double diff = angle1 - angle2;
    if (diff > 180.0) {
      diff -= 360.0;
    } else if (diff < -180.0) {
      diff += 360.0;
    }
    return std::abs(diff);
  }

  void updateRobotPoseFromTf()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      current_pos_ = Point2f(
        static_cast<float>(transform.transform.translation.x),
        static_cast<float>(transform.transform.translation.y));
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      m.getRPY(roll, pitch, yaw);
      current_yaw_deg_ = yaw * 180.0 / M_PI;
      tf_received_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF lookup failed: %s", ex.what());
    }
  }

  void performScanIfNeeded()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    bool should_scan = false;
    if (counter_ == 0) {
      should_scan = true;
    } else if (last_scan_position_.x != -1000.0f && last_scan_position_.y != -1000.0f) {
      const double dist = explorer_mission::euclideanDist(last_scan_position_, current_pos_);
      should_scan = dist > 0.5;
    }

    if (!should_scan) {
      return;
    }

    if (!rotate_client_->wait_for_action_server(std::chrono::seconds(5))) {
      return;
    }

    auto goal = Rotate360::Goal();
    auto future = rotate_client_->async_send_goal(goal);
    if (future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
      cached_images_.clear();
      cached_orientations_.clear();
      return;
    }

    const auto goal_handle = future.get();
    if (!goal_handle) {
      return;
    }

    auto result_future = rotate_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
      rotate_client_->async_cancel_goal(goal_handle);
      cached_images_.clear();
      cached_orientations_.clear();
      return;
    }

    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success) {
      cached_images_ = wrapped.result->cached_images;
      cached_orientations_ = wrapped.result->cached_orientations;
    } else {
      cached_images_.clear();
      cached_orientations_.clear();
    }
    last_scan_position_ = current_pos_;
    ++counter_;
  }

  bool navigateToPosition(const cv::Point2f & goal_pos, const cv::Point2f & look_at)
  {
    const double dx = look_at.x - current_pos_.x;
    const double dy = look_at.y - current_pos_.y;
    const double goal_yaw_deg = std::atan2(dy, dx) * 180.0 / M_PI;
    return navigateToGoal(goal_pos.x, goal_pos.y, goal_yaw_deg);
  }

  bool navigateToGoal(double goal_x, double goal_y, double goal_yaw_deg)
  {
    updateRobotPoseFromTf();
    if (navigation_mode_ == "nav2" && nav2_navigator_) {
      const double goal_yaw_rad = goal_yaw_deg * M_PI / 180.0;
      const bool ok = nav2_navigator_->navigateToPose(
        goal_x, goal_y, goal_yaw_rad, map_frame_, 120.0, 60.0, 0.1,
        [this]() {
          updateRobotPoseFromTf();
          nav2_navigator_->noteProgress(
            current_pos_.x, current_pos_.y, 0.1, 60.0);
          return true;
        });
      if (!ok) {
        RCLCPP_WARN(
          get_logger(), "Nav2 navigation failed: %s",
          nav2_navigator_->lastError().c_str());
      }
      return ok;
    }

    const auto plan = explorer_mission::planToPose(
      current_pos_.x, current_pos_.y, current_yaw_deg_,
      goal_x, goal_y, goal_yaw_deg);
    return executeDiscretePlan(plan);
  }

  bool executeDiscretePlan(const std::vector<explorer_mission::NavigationStep> & plan)
  {
    for (const auto & step : plan) {
      DiscreteMove::Goal goal;
      goal.direction = step.direction;
      goal.steps = step.steps;

      auto future = discrete_move_client_->async_send_goal(goal);
      if (future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
        return false;
      }
      const auto goal_handle = future.get();
      if (!goal_handle) {
        return false;
      }
      auto result_future = discrete_move_client_->async_get_result(goal_handle);
      if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
        discrete_move_client_->async_cancel_goal(goal_handle);
        return false;
      }
      const auto wrapped = result_future.get();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result->success) {
        return false;
      }
      updateRobotPoseFromTf();
    }
    return true;
  }

  int counter_{0};
  Point2f last_scan_position_{-1000.0f, -1000.0f};
  Point2f current_pos_{0.0f, 0.0f};
  double current_yaw_deg_{0.0};
  bool tf_received_{false};

  std::string map_frame_;
  std::string base_frame_;
  std::string navigation_mode_{"nav2"};
  double frontier_detection_radius_{5.0};
  int min_contour_pixels_{15};
  double vlm_scores_timeout_s_{120.0};
  bool publish_debug_topics_{false};

  std::unique_ptr<explorer_mission::Nav2Navigator> nav2_navigator_;
  explorer_mission::FrontierTree tree_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Client<DiscreteMove>::SharedPtr discrete_move_client_;
  rclcpp_action::Client<Rotate360>::SharedPtr rotate_client_;

  std::mutex grid_mutex_;
  nav_msgs::msg::OccupancyGrid latest_grid_;
  bool have_grid_{false};

  std::mutex scores_mutex_;
  explorer_msgs::msg::FrontierOpennessScores latest_scores_;
  bool scores_received_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<explorer_msgs::msg::FrontierOpennessScores>::SharedPtr vlm_scores_sub_;

  rclcpp::Publisher<explorer_msgs::msg::FrontierTree>::SharedPtr tree_pub_;
  rclcpp::Publisher<explorer_msgs::msg::ExplorationStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<explorer_msgs::msg::FrontierViews>::SharedPtr vlm_views_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr debug_events_pub_;
  rclcpp::Publisher<explorer_msgs::msg::FrontierViews>::SharedPtr debug_vlm_batch_pub_;

  std::vector<sensor_msgs::msg::CompressedImage> cached_images_;
  std::vector<double> cached_orientations_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  int rc = 0;
  if (!node->waitForDependencies()) {
    RCLCPP_ERROR(rclcpp::get_logger("explore"), "Failed to initialize dependencies.");
    rc = 1;
  } else {
    node->startExploration();
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return rc;
}
