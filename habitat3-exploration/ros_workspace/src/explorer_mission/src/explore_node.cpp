#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int8.hpp>

#include <explorer_msgs/action/discrete_move.hpp>
#include <explorer_msgs/action/perceive_and_capture.hpp>
#include <explorer_msgs/action/rotate360.hpp>
#include <explorer_msgs/msg/frontier.hpp>
#include <explorer_msgs/msg/frontier_array.hpp>
#include <explorer_msgs/msg/frontier_blacklist.hpp>
#include <explorer_msgs/msg/frontier_views.hpp>
#include <explorer_msgs/msg/vertex.hpp>
#include <explorer_msgs/srv/add_edge.hpp>
#include <explorer_msgs/srv/run_dijkstra.hpp>

#include "explorer_mission/discrete_navigator.hpp"
#include "explorer_mission/frontier_detection.hpp"

using DiscreteMove = explorer_msgs::action::DiscreteMove;
using Rotate360 = explorer_msgs::action::Rotate360;
using PerceiveAndCapture = explorer_msgs::action::PerceiveAndCapture;

using cv::Point2f;

struct PairIntHash
{
  std::size_t operator()(const std::pair<int, int> & p) const
  {
    const std::size_t h1 = std::hash<int>{}(p.first);
    const std::size_t h2 = std::hash<int>{}(p.second);
    return h1 ^ (h2 * 2654435761u);
  }
};

using BlacklistKey = std::pair<int, int>;

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
    perceive_timeout_s_ = declare_parameter<double>("perceive_timeout_s", 240.0);
    vlm_choice_timeout_s_ = declare_parameter<double>("vlm_choice_timeout_s", 240.0);

    graph_add_edge_client_ = create_client<explorer_msgs::srv::AddEdge>(
      "/graph_node/graph/add_edge");
    graph_run_dijkstra_client_ = create_client<explorer_msgs::srv::RunDijkstra>(
      "/graph_node/graph/run_dijkstra");

    discrete_move_client_ = rclcpp_action::create_client<DiscreteMove>(
      this, "/movement/discrete_move");
    rotate_client_ = rclcpp_action::create_client<Rotate360>(this, "rotate_360");
    perceive_client_ = rclcpp_action::create_client<PerceiveAndCapture>(
      this, "perceive_and_capture");

    filtered_frontiers_sub_ = create_subscription<explorer_msgs::msg::FrontierArray>(
      "frontiers/filtered_frontiers", rclcpp::QoS(1),
      std::bind(&ExploreNode::filteredFrontiersCb, this, std::placeholders::_1));
    raw_frontiers_sub_ = create_subscription<explorer_msgs::msg::FrontierArray>(
      "frontiers/frontiers", rclcpp::QoS(1),
      std::bind(&ExploreNode::rawFrontiersCb, this, std::placeholders::_1));
    chosen_frontier_sub_ = create_subscription<std_msgs::msg::Int8>(
      "chosen_frontier", rclcpp::QoS(1),
      std::bind(&ExploreNode::chosenFrontierCb, this, std::placeholders::_1));

    frontier_blacklist_pub_ = create_publisher<explorer_msgs::msg::FrontierBlacklist>(
      "frontier_blacklist", rclcpp::QoS(1).transient_local());
    frontier_views_pub_ = create_publisher<explorer_msgs::msg::FrontierViews>(
      "frontiers/frontier_views", rclcpp::QoS(1).transient_local());

    latest_chosen_frontier_index_ = -1;
    chosen_frontier_received_ = false;

    RCLCPP_INFO(get_logger(), "Explore node initializing...");
  }

  bool waitForDependencies()
  {
    RCLCPP_INFO(get_logger(), "Waiting for required services and action servers...");

    if (!graph_add_edge_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "graph add_edge service not available");
      return false;
    }
    if (!graph_run_dijkstra_client_->wait_for_service(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "graph run_dijkstra service not available");
      return false;
    }

    if (!discrete_move_client_->wait_for_action_server(std::chrono::seconds(90))) {
      RCLCPP_ERROR(get_logger(), "DiscreteMove action server not available");
      return false;
    }
    if (!rotate_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "rotate_360 action server not available");
      return false;
    }
    if (!perceive_client_->wait_for_action_server(std::chrono::seconds(30))) {
      RCLCPP_ERROR(get_logger(), "perceive_and_capture action server not available");
      return false;
    }

    const auto start_wait = now();
    while (rclcpp::ok() && !tf_received_ &&
      (now() - start_wait).seconds() < 10.0)
    {
      updateRobotPoseFromTf();
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(get_logger(), "All dependencies are ready.");
    return true;
  }

  void startExploration()
  {
    RCLCPP_INFO(get_logger(), "Exploration started");

    while (rclcpp::ok()) {
      updateRobotPoseFromTf();
      performScanIfNeeded();

      const auto starting_vertex = determineStartingVertexAndCreateEdges();

      std::vector<explorer_msgs::msg::Frontier> closest_frontiers;
      if (!getFrontiersToExplore(closest_frontiers, starting_vertex)) {
        if (traversal_vertex_.id == 0) {
          logExplorationStopReason();
          break;
        }
        continue;
      }

      const auto snapshot_closest_frontiers = closest_frontiers;

      if (snapshot_closest_frontiers.size() > 1) {
        RCLCPP_INFO(
          get_logger(), "Decision point found with %zu options.",
          snapshot_closest_frontiers.size());
        decision_point_stack_.push_back(starting_vertex);
      }

      if (cached_images_.empty()) {
        RCLCPP_WARN(get_logger(), "No cached images; blacklisting frontiers.");
        blacklistFrontiers(snapshot_closest_frontiers);
        continue;
      }

      auto frontier_views_msg = matchCachedImagesToFrontiers(snapshot_closest_frontiers);
      if (frontier_views_msg.images.empty()) {
        RCLCPP_WARN(get_logger(), "Failed to match cached images; blacklisting.");
        blacklistFrontiers(snapshot_closest_frontiers);
        continue;
      }

      int8_t chosen_idx = 0;
      if (snapshot_closest_frontiers.size() > 1) {
        frontier_views_pub_->publish(frontier_views_msg);
        chosen_frontier_received_ = false;
        if (!selectFrontierIndex(snapshot_closest_frontiers, chosen_idx)) {
          continue;
        }
      } else {
        RCLCPP_INFO(get_logger(), "Single frontier available; auto-selecting index 0.");
      }

      if (static_cast<size_t>(chosen_idx) >= snapshot_closest_frontiers.size()) {
        RCLCPP_ERROR(
          get_logger(), "Chosen frontier index %d out of range (count=%zu).",
          chosen_idx, snapshot_closest_frontiers.size());
        blacklistFrontiers(snapshot_closest_frontiers);
        continue;
      }

      const explorer_msgs::msg::Frontier & chosen_frontier =
        snapshot_closest_frontiers[static_cast<size_t>(chosen_idx)];

      navigateToFrontier(chosen_frontier);

      prev_vertex_ = starting_vertex;
      traversal_vertex_.id = 0;
      traversal_vertex_.x = 0.0;
      traversal_vertex_.y = 0.0;
      ++counter_;
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(get_logger(), "Exploration completed");
  }

private:
  void filteredFrontiersCb(const explorer_msgs::msg::FrontierArray::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      latest_filtered_frontiers_ = *msg;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received filtered frontiers (count=%zu)", msg->frontiers.size());
  }

  void rawFrontiersCb(const explorer_msgs::msg::FrontierArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(frontiers_mutex_);
    latest_raw_frontiers_ = *msg;
  }

  void chosenFrontierCb(const std_msgs::msg::Int8::SharedPtr msg)
  {
    latest_chosen_frontier_index_ = msg->data;
    chosen_frontier_received_ = true;
    RCLCPP_INFO(
      get_logger(), "Received chosen frontier index: %d",
      latest_chosen_frontier_index_.load());
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

  explorer_msgs::msg::FrontierViews matchCachedImagesToFrontiers(
    const std::vector<explorer_msgs::msg::Frontier> & frontiers)
  {
    explorer_msgs::msg::FrontierViews msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();

    if (cached_images_.empty() || cached_orientations_.empty()) {
      return msg;
    }
    if (cached_images_.size() != cached_orientations_.size()) {
      return msg;
    }

    updateRobotPoseFromTf();

    for (size_t display_id = 0; display_id < frontiers.size(); ++display_id) {
      const auto & frontier = frontiers[display_id];
      const double dx = frontier.midpoint.x - current_pos_.x;
      const double dy = frontier.midpoint.y - current_pos_.y;
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
      msg.frontiers.push_back(static_cast<uint8_t>(display_id));
    }
    return msg;
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
  }

  explorer_msgs::msg::Vertex determineStartingVertexAndCreateEdges()
  {
    explorer_msgs::msg::Vertex starting_vertex;
    if (traversal_vertex_.id > 0) {
      starting_vertex = traversal_vertex_;
    } else {
      starting_vertex.id = 0;
      starting_vertex.x = current_pos_.x;
      starting_vertex.y = current_pos_.y;
    }

    if (prev_vertex_.id > 0 && traversal_vertex_.id == 0) {
      auto req = std::make_shared<explorer_msgs::srv::AddEdge::Request>();
      req->a = prev_vertex_;
      req->b = starting_vertex;
      req->weight_override = 0.0;
      req->snap_tolerance_m = 0.1;
      auto future = graph_add_edge_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
        starting_vertex = future.get()->b_out;
      }
    } else if (prev_vertex_.id == 0 && traversal_vertex_.id == 0) {
      auto req = std::make_shared<explorer_msgs::srv::AddEdge::Request>();
      req->a = starting_vertex;
      req->b = starting_vertex;
      req->weight_override = 0.0;
      req->snap_tolerance_m = 0.1;
      auto future = graph_add_edge_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
        starting_vertex = future.get()->a_out;
      }
    }
    return starting_vertex;
  }

  bool getFrontiersToExplore(
    std::vector<explorer_msgs::msg::Frontier> & frontiers_out,
    const explorer_msgs::msg::Vertex & starting_vertex)
  {
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      frontiers_out = latest_filtered_frontiers_.frontiers;
    }
    if (!frontiers_out.empty()) {
      return true;
    }

    if (decision_point_stack_.empty()) {
      frontiers_out = getFrontiersInRange(0.5, 60.0);
      if (frontiers_out.empty()) {
        frontiers_out = getAnyUnblacklistedRawFrontiers();
      }
      if (frontiers_out.empty()) {
        return false;
      }
      renumberFrontierIds(frontiers_out);
      RCLCPP_INFO(
        get_logger(), "Using %zu fallback raw frontiers (filtered set empty).",
        frontiers_out.size());
      return true;
    }

    RCLCPP_INFO(
      get_logger(), "No filtered frontiers; attempting dead-end backtrack (%zu decision points).",
      decision_point_stack_.size());
    traversal_vertex_ = handleDeadEnd(starting_vertex);
    if (traversal_vertex_.id == 0) {
      return false;
    }
    prev_vertex_ = starting_vertex;
    ++counter_;
    return false;
  }

  bool selectFrontierIndex(
    const std::vector<explorer_msgs::msg::Frontier> & frontiers,
    int8_t & chosen_idx_out)
  {
    if (frontiers.empty()) {
      return false;
    }
    if (frontiers.size() > 1) {
      const auto start = now();
      while (!chosen_frontier_received_ && rclcpp::ok() &&
        (now() - start).seconds() < vlm_choice_timeout_s_)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
      if (!chosen_frontier_received_) {
        blacklistFrontiers(frontiers);
        return false;
      }
      const int chosen_index = latest_chosen_frontier_index_.load();
      if (chosen_index < 0 ||
        static_cast<size_t>(chosen_index) >= frontiers.size())
      {
        RCLCPP_WARN(
          get_logger(),
          "VLM chose invalid frontier index %d (valid 0..%zu); blacklisting candidates.",
          chosen_index, frontiers.size() - 1);
        blacklistFrontiers(frontiers);
        return false;
      }
      chosen_idx_out = static_cast<int8_t>(chosen_index);
    } else {
      chosen_idx_out = 0;
    }
    return true;
  }

  void navigateToFrontier(const explorer_msgs::msg::Frontier & chosen_frontier)
  {
    const bool ok = executeNavigation(chosen_frontier);
    blacklistFrontier(chosen_frontier);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Navigation failed; frontier blacklisted.");
    }
  }

  static void renumberFrontierIds(std::vector<explorer_msgs::msg::Frontier> & frontiers)
  {
    for (size_t i = 0; i < frontiers.size(); ++i) {
      frontiers[i].id = static_cast<uint8_t>(i & 0xFF);
    }
  }

  void logExplorationStopReason()
  {
    size_t raw_count = 0;
    size_t filtered_count = 0;
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      raw_count = latest_raw_frontiers_.frontiers.size();
      filtered_count = latest_filtered_frontiers_.frontiers.size();
    }
    RCLCPP_WARN(
      get_logger(),
      "Stopping exploration: raw_frontiers=%zu filtered_frontiers=%zu "
      "blacklist_entries=%zu decision_points=%zu",
      raw_count, filtered_count, frontier_blacklist_.size(), decision_point_stack_.size());
  }

  std::vector<explorer_msgs::msg::Frontier> getAnyUnblacklistedRawFrontiers()
  {
    std::vector<explorer_msgs::msg::Frontier> result;
    std::vector<explorer_msgs::msg::Frontier> raw;
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      raw = latest_raw_frontiers_.frontiers;
    }
    for (const auto & frontier : raw) {
      const Point2f frontier_pos(
        static_cast<float>(frontier.midpoint.x),
        static_cast<float>(frontier.midpoint.y));
      if (frontier_pos.x == 0.0f && frontier_pos.y == 0.0f) {
        continue;
      }
      if (!isBlacklisted(frontier_pos)) {
        result.push_back(frontier);
      }
    }
    std::sort(
      result.begin(), result.end(),
      [this](const explorer_msgs::msg::Frontier & a, const explorer_msgs::msg::Frontier & b) {
        const double da = explorer_mission::euclideanDist(
          current_pos_, Point2f(static_cast<float>(a.midpoint.x), static_cast<float>(a.midpoint.y)));
        const double db = explorer_mission::euclideanDist(
          current_pos_, Point2f(static_cast<float>(b.midpoint.x), static_cast<float>(b.midpoint.y)));
        return da < db;
      });
    if (result.size() > 8) {
      result.resize(8);
    }
    return result;
  }

  std::vector<explorer_msgs::msg::Frontier> getFrontiersInRange(
    double min_radius_m, double max_radius_m)
  {
    std::vector<explorer_msgs::msg::Frontier> result;
    std::vector<explorer_msgs::msg::Frontier> raw;
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      raw = latest_raw_frontiers_.frontiers;
    }
    for (const auto & frontier : raw) {
      const Point2f frontier_pos(
        static_cast<float>(frontier.midpoint.x),
        static_cast<float>(frontier.midpoint.y));
      const double dist = explorer_mission::euclideanDist(current_pos_, frontier_pos);
      if (dist >= min_radius_m && dist <= max_radius_m && !isBlacklisted(frontier_pos)) {
        result.push_back(frontier);
      }
    }
    std::sort(
      result.begin(), result.end(),
      [this](const explorer_msgs::msg::Frontier & a, const explorer_msgs::msg::Frontier & b) {
        const double da = explorer_mission::euclideanDist(
          current_pos_, Point2f(static_cast<float>(a.midpoint.x), static_cast<float>(a.midpoint.y)));
        const double db = explorer_mission::euclideanDist(
          current_pos_, Point2f(static_cast<float>(b.midpoint.x), static_cast<float>(b.midpoint.y)));
        return da < db;
      });
    if (result.size() > 8) {
      result.resize(8);
    }
    return result;
  }

  std::vector<explorer_msgs::msg::Frontier> getFrontiersNearPoint(
    double x, double y, double min_radius_m, double max_radius_m)
  {
    std::vector<explorer_msgs::msg::Frontier> result;
    const Point2f query(static_cast<float>(x), static_cast<float>(y));
    std::vector<explorer_msgs::msg::Frontier> raw;
    {
      std::lock_guard<std::mutex> lock(frontiers_mutex_);
      raw = latest_raw_frontiers_.frontiers;
    }
    for (const auto & frontier : raw) {
      const Point2f pos(
        static_cast<float>(frontier.midpoint.x),
        static_cast<float>(frontier.midpoint.y));
      const double dist = explorer_mission::euclideanDist(query, pos);
      if (dist >= min_radius_m && dist <= max_radius_m && !isBlacklisted(pos)) {
        result.push_back(frontier);
      }
    }
    if (result.size() > 8) {
      result.resize(8);
    }
    return result;
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

  double getFrontierOrientation(
    const explorer_msgs::msg::Frontier & frontier,
    const Point2f & midpoint) const
  {
    const double dx = midpoint.x - current_pos_.x;
    const double dy = midpoint.y - current_pos_.y;
    return std::atan2(dy, dx);
  }

  bool executeNavigation(const explorer_msgs::msg::Frontier & frontier)
  {
    const Point2f midpoint(
      static_cast<float>(frontier.midpoint.x),
      static_cast<float>(frontier.midpoint.y));
    const double dist = frontier.distance;
    const double safety_offset = 0.3;
    const double safe_distance = dist - safety_offset;
    if (safe_distance <= 0.0) {
      return false;
    }

    const double dx_vec = midpoint.x - current_pos_.x;
    const double dy_vec = midpoint.y - current_pos_.y;
    const double vec_len = std::sqrt(dx_vec * dx_vec + dy_vec * dy_vec);
    const double dx_norm = dx_vec / vec_len;
    const double dy_norm = dy_vec / vec_len;

    const double goal_x = current_pos_.x + dx_norm * safe_distance;
    const double goal_y = current_pos_.y + dy_norm * safe_distance;
    const double goal_yaw_rad = getFrontierOrientation(frontier, midpoint);
    const double goal_yaw_deg = goal_yaw_rad * 180.0 / M_PI;

    updateRobotPoseFromTf();
    const auto plan = explorer_mission::planToPose(
      current_pos_.x, current_pos_.y, current_yaw_deg_,
      goal_x, goal_y, goal_yaw_deg);

    const auto start = now();
    const double total_timeout_s = 120.0;
    Point2f last_check_pos = current_pos_;
    auto last_check_time = start;

    for (const auto & step : plan) {
      if ((now() - start).seconds() >= total_timeout_s) {
        return false;
      }
      if ((now() - last_check_time).seconds() > 60.0) {
        const double moved = explorer_mission::euclideanDist(current_pos_, last_check_pos);
        if (moved < 0.1) {
          return false;
        }
        last_check_time = now();
        last_check_pos = current_pos_;
      }

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

  explorer_msgs::msg::Vertex handleDeadEnd(const explorer_msgs::msg::Vertex & starting_vertex)
  {
    explorer_msgs::msg::Vertex empty;
    empty.id = 0;
    if (decision_point_stack_.empty()) {
      return empty;
    }

    std::sort(
      decision_point_stack_.begin(), decision_point_stack_.end(),
      [this](const explorer_msgs::msg::Vertex & a, const explorer_msgs::msg::Vertex & b) {
        const double da = explorer_mission::euclideanDist(current_pos_, Point2f(a.x, a.y));
        const double db = explorer_mission::euclideanDist(current_pos_, Point2f(b.x, b.y));
        return da < db;
      });

    while (!decision_point_stack_.empty()) {
      const auto traversal_vertex = decision_point_stack_.front();
      decision_point_stack_.erase(decision_point_stack_.begin());

      const auto frontiers = getFrontiersNearPoint(
        traversal_vertex.x, traversal_vertex.y, 0.5, 15.0);
      if (frontiers.empty()) {
        continue;
      }

      auto req = std::make_shared<explorer_msgs::srv::RunDijkstra::Request>();
      req->start = starting_vertex;
      req->goal = traversal_vertex;
      req->snap_tolerance_m = 0.5;
      auto future = graph_run_dijkstra_client_->async_send_request(req);
      if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
        continue;
      }
      const auto res = future.get();
      if (!res->success || res->path.poses.empty()) {
        continue;
      }

      const auto & final_pose = res->path.poses.back().pose;
      updateRobotPoseFromTf();
      const double goal_yaw = 0.0;
      const auto plan = explorer_mission::planToPose(
        current_pos_.x, current_pos_.y, current_yaw_deg_,
        final_pose.position.x, final_pose.position.y, goal_yaw);
      if (executeDiscretePlan(plan)) {
        RCLCPP_INFO(
          get_logger(), "Dead-end backtrack succeeded toward decision point id=%d.",
          traversal_vertex.id);
        return traversal_vertex;
      }
    }
    RCLCPP_WARN(get_logger(), "Dead-end backtrack exhausted all decision points.");
    return empty;
  }

  static Point2f roundMidpoint(Point2f midpoint)
  {
    const double rounded_x = std::round(midpoint.x / 0.15) * 0.15;
    const double rounded_y = std::round(midpoint.y / 0.15) * 0.15;
    return Point2f(
      static_cast<float>(std::round(rounded_x * 10.0) / 10.0),
      static_cast<float>(std::round(rounded_y * 10.0) / 10.0));
  }

  BlacklistKey toGridKey(float x, float y) const
  {
    return {
      static_cast<int>(std::round(x * 10.0f)),
      static_cast<int>(std::round(y * 10.0f))};
  }

  bool isBlacklisted(Point2f pos) const
  {
    const int kx = static_cast<int>(std::round(pos.x * 10.0f));
    const int ky = static_cast<int>(std::round(pos.y * 10.0f));
    for (int dx = -5; dx <= 5; ++dx) {
      for (int dy = -5; dy <= 5; ++dy) {
        if (frontier_blacklist_.count({kx + dx, ky + dy})) {
          return true;
        }
      }
    }
    return false;
  }

  void blacklistFrontier(const explorer_msgs::msg::Frontier & frontier)
  {
    const Point2f rounded = roundMidpoint(
      Point2f(static_cast<float>(frontier.midpoint.x), static_cast<float>(frontier.midpoint.y)));
    frontier_blacklist_.insert(toGridKey(rounded.x, rounded.y));
    publishBlacklist();
  }

  void blacklistFrontiers(const std::vector<explorer_msgs::msg::Frontier> & frontiers)
  {
    for (const auto & frontier : frontiers) {
      blacklistFrontier(frontier);
    }
  }

  void publishBlacklist()
  {
    explorer_msgs::msg::FrontierBlacklist msg;
    for (const auto & key : frontier_blacklist_) {
      geometry_msgs::msg::Point p;
      p.x = key.first / 10.0;
      p.y = key.second / 10.0;
      p.z = 0.0;
      msg.points.push_back(p);
    }
    frontier_blacklist_pub_->publish(msg);
  }

  int counter_{0};
  Point2f last_scan_position_{-1000.0f, -1000.0f};
  Point2f current_pos_{0.0f, 0.0f};
  double current_yaw_deg_{0.0};
  bool tf_received_{false};

  std::string map_frame_;
  std::string base_frame_;
  double perceive_timeout_s_{240.0};
  double vlm_choice_timeout_s_{240.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  explorer_msgs::msg::Vertex prev_vertex_;
  explorer_msgs::msg::Vertex traversal_vertex_;

  rclcpp::Client<explorer_msgs::srv::AddEdge>::SharedPtr graph_add_edge_client_;
  rclcpp::Client<explorer_msgs::srv::RunDijkstra>::SharedPtr graph_run_dijkstra_client_;
  rclcpp_action::Client<DiscreteMove>::SharedPtr discrete_move_client_;
  rclcpp_action::Client<Rotate360>::SharedPtr rotate_client_;
  rclcpp_action::Client<PerceiveAndCapture>::SharedPtr perceive_client_;

  std::vector<explorer_msgs::msg::Vertex> decision_point_stack_;
  std::unordered_set<BlacklistKey, PairIntHash> frontier_blacklist_;
  // Written by subscription callbacks (executor thread), read by the main loop.
  std::mutex frontiers_mutex_;
  explorer_msgs::msg::FrontierArray latest_filtered_frontiers_;
  explorer_msgs::msg::FrontierArray latest_raw_frontiers_;

  rclcpp::Subscription<explorer_msgs::msg::FrontierArray>::SharedPtr filtered_frontiers_sub_;
  rclcpp::Subscription<explorer_msgs::msg::FrontierArray>::SharedPtr raw_frontiers_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr chosen_frontier_sub_;
  rclcpp::Publisher<explorer_msgs::msg::FrontierBlacklist>::SharedPtr frontier_blacklist_pub_;
  rclcpp::Publisher<explorer_msgs::msg::FrontierViews>::SharedPtr frontier_views_pub_;

  std::atomic<int> latest_chosen_frontier_index_{-1};
  std::atomic<bool> chosen_frontier_received_{false};
  std::vector<sensor_msgs::msg::CompressedImage> cached_images_;
  std::vector<double> cached_orientations_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreNode>();

  // Spin the node on a background executor so that action-client goal/result
  // futures and service futures actually make progress while the main loop
  // blocks on future.wait_for(...). Without this, every wait_for blocks for its
  // full timeout (e.g. 120 s for the rotate scan) before returning.
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
