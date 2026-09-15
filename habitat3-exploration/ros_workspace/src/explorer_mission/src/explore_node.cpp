#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
#include <explorer_msgs/msg/brain_decision_event.hpp>
#include <explorer_msgs/msg/brain_graph_edges.hpp>
#include <explorer_msgs/msg/exploration_status.hpp>
#include <explorer_msgs/msg/frontier_openness_scores.hpp>
#include <explorer_msgs/msg/frontier_tree.hpp>
#include <explorer_msgs/msg/frontier_views.hpp>
#include <explorer_msgs/msg/vlm_choice_event.hpp>

#include "explorer_mission/discrete_navigator.hpp"
#include "explorer_mission/exploration_brain.hpp"
#include "explorer_mission/frontier_detection.hpp"
#include "explorer_mission/nav_fail_policy.hpp"
#include "explorer_mission/nav2_navigator.hpp"
#include "explorer_mission/return_home_guard.hpp"
#include "explorer_mission/stuck_progress.hpp"
#include "explorer_mission/wall_unstick.hpp"

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
    frontier_detection_radius_ = declare_parameter<double>("frontier_detection_radius", 50.0);
    frontier_exclusion_radius_ = declare_parameter<double>("frontier_exclusion_radius", 1.0);
    min_contour_pixels_ = declare_parameter<int>("min_contour_pixels", 15);
    vlm_scores_timeout_s_ = declare_parameter<double>("vlm_scores_timeout_s", 300.0);
    publish_debug_topics_ = declare_parameter<bool>("publish_debug_topics", false);
    brain_id_ = declare_parameter<std::string>("brain_id", "vlm_tree_dfs");
    dfs_prefer_highest_openness_ = declare_parameter<bool>("dfs_prefer_highest_openness", true);
    goal_accept_radius_m_ = declare_parameter<double>("goal_accept_radius_m", 1.0);
    parent_to_nearest_node_ = declare_parameter<bool>("parent_to_nearest_node", true);
    early_nav_min_score_ = declare_parameter<int>("early_nav_min_score", 3);
    return_home_max_attempts_ = declare_parameter<int>("return_home_max_attempts", 20);
    unstick_min_clearance_m_ = declare_parameter<double>("unstick_min_clearance_m", 0.25);
    unstick_max_steps_ = declare_parameter<int>("unstick_max_steps", 8);
    unstick_max_attempts_ = declare_parameter<int>(
      "unstick_max_attempts", explorer_mission::kDefaultMaxUnstickAttempts);
    nav2_inflation_radius_m_ = declare_parameter<double>("nav2_inflation_radius_m", 0.22);
    mapper_inflation_m_ = declare_parameter<double>("mapper_inflation_m", mapper_inflation_m_);

    {
      explorer_mission::ExplorationBrainConfig cfg;
      cfg.dfs_prefer_highest_openness = dfs_prefer_highest_openness_;
      cfg.parent_to_nearest_node = parent_to_nearest_node_;
      brain_ = explorer_mission::createExplorationBrain(brain_id_, cfg);
    }

    param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        bool refresh_config = false;
        for (const auto & p : params) {
          if (p.get_name() == "dfs_prefer_highest_openness") {
            dfs_prefer_highest_openness_ = p.as_bool();
            refresh_config = true;
            RCLCPP_INFO(
              get_logger(), "dfs_prefer_highest_openness set to %s",
              dfs_prefer_highest_openness_ ? "true (highest first)" : "false (lowest first)");
          } else if (p.get_name() == "parent_to_nearest_node") {
            parent_to_nearest_node_ = p.as_bool();
            refresh_config = true;
            RCLCPP_INFO(
              get_logger(), "parent_to_nearest_node set to %s",
              parent_to_nearest_node_ ? "true" : "false");
          }
        }
        if (refresh_config && brain_) {
          explorer_mission::ExplorationBrainConfig cfg;
          cfg.dfs_prefer_highest_openness = dfs_prefer_highest_openness_;
          cfg.parent_to_nearest_node = parent_to_nearest_node_;
          brain_->setConfig(cfg);
        }
        return result;
      });

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
    brain_decision_pub_ = create_publisher<explorer_msgs::msg::BrainDecisionEvent>(
      "exploration/brain/decision", latched);
    brain_graph_edges_pub_ = create_publisher<explorer_msgs::msg::BrainGraphEdges>(
      "exploration/brain/graph_edges", latched);
    vlm_choice_pub_ = create_publisher<explorer_msgs::msg::VlmChoiceEvent>(
      "exploration/vlm/choice", latched);
    vlm_views_pub_ = create_publisher<explorer_msgs::msg::FrontierViews>(
      "exploration/vlm/views", rclcpp::QoS(1));

    if (publish_debug_topics_) {
      debug_events_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "exploration/debug/events", rclcpp::QoS(1));
      debug_vlm_batch_pub_ = create_publisher<explorer_msgs::msg::FrontierViews>(
        "exploration/debug/last_vlm_batch", rclcpp::QoS(1));
    }

    RCLCPP_INFO(
      get_logger(), "Explore node initializing (brain_id=%s)...", brain_id_.c_str());
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
    RCLCPP_INFO(get_logger(), "Exploration started (brain=%s)", brain_->id().c_str());
    updateRobotPoseFromTf();
    brain_->onEpisodeStart(current_pos_);
    scanned_detect_keys_.clear();
    scan_poses_.clear();
    greedy_visited_poses_.clear();
    greedy_detect_key_ = 0;

    // Initial 360° at episode start (root / spawn is still unvisited).
    publishPhase("scanning", 0, 0, false, "initial scan");
    performScanIfNeeded(/*force=*/true);
    // Mark spawn / root visited so return-home / backtrack does not re-scan.
    if (brain_->usesFrontierTree() && !brain_->isVisited(statusNodeId())) {
      brain_->onArrived(statusNodeId(), current_pos_);
    }
    publishTree();
    publishPhase("idle", statusNodeId(), 0, false, "episode start");

    while (rclcpp::ok()) {
      updateRobotPoseFromTf();

      const uint32_t detect_key = detectKey();
      if (scanned_detect_keys_.count(detect_key) == 0) {
        if (!detectAndOfferToBrain()) {
          RCLCPP_WARN(get_logger(), "Frontier detection/VLM rating failed; retrying.");
          rclcpp::sleep_for(std::chrono::seconds(1));
          continue;
        }
        scanned_detect_keys_.insert(detect_key);
        scan_poses_[detect_key] = current_pos_;
      }

      explorer_mission::BrainDecision decision =
        brain_->selectNextGoal(explorer_mission::BrainContext{current_pos_});
      if (decision.action == explorer_mission::BrainAction::kWait) {
        // Detect path should have blocked on VLM; soft-fail leftover unrated.
        RCLCPP_WARN(get_logger(), "Brain waiting on scores; soft-failing unrated");
        brain_->softFailUnrated(1);
        publishTree();
        decision = brain_->selectNextGoal(explorer_mission::BrainContext{current_pos_});
      }

      publishBrainDecision(decision);
      publishPendingBrainTelemetry();

      if (decision.action == explorer_mission::BrainAction::kComplete) {
        publishPhase(
          "complete", statusNodeId(), 0, true, decision.detail,
          explorer_mission::terminationReasonCStr(
            explorer_mission::TerminationReason::kSuccess));
        break;
      }

      if (decision.action != explorer_mission::BrainAction::kNavigateTo) {
        RCLCPP_WARN(get_logger(), "Unexpected brain action; retrying.");
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      const uint32_t from_id = detectKey();

      // Theoretical DFS / parent hops: adopt node without physical return.
      if (decision.theoretical) {
        publishPhase(
          "backtracking", from_id, decision.goal_id, false,
          decision.detail.empty() ? "theoretical backtrack" : decision.detail);
        brain_->onArrived(decision.goal_id, decision.goal);
        if (!brain_->usesFrontierTree()) {
          greedy_detect_key_ = decision.goal_id;
        }
        publishTree();
        continue;
      }

      publishPhase(
        "navigating", from_id, decision.goal_id, false, decision.detail);

      const auto nav = navigateNewGoalWithRecovery(
        decision.goal, decision.goal, from_id);
      if (nav.terminate_stuck) {
        publishPhase(
          "complete", statusNodeId(), decision.goal_id, true,
          "stuck: cannot return to previous node",
          explorer_mission::terminationReasonCStr(
            explorer_mission::TerminationReason::kStuck));
        break;
      }
      if (!nav.ok) {
        brain_->onNavFailed(decision.goal_id, nav.mark_frontier_dead);
        publishTree();
        continue;
      }

      const bool already_visited = brain_->isVisited(decision.goal_id);
      brain_->onArrived(decision.goal_id, decision.goal);
      if (!brain_->usesFrontierTree()) {
        greedy_visited_poses_.push_back(decision.goal);
        greedy_detect_key_ = decision.goal_id;
      }
      publishTree();
      // 360° only on first visit; visited-but-alive (e.g. backtrack) skips rescan.
      if (explorer_mission::shouldPerformFrontierScan(already_visited)) {
        publishPhase("scanning", decision.goal_id, 0, false, "arrived at unvisited frontier");
        performScanIfNeeded(/*force=*/true);
      } else {
        publishPhase("idle", decision.goal_id, 0, false, "skip scan (visited alive)");
      }
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
    std::unordered_map<uint32_t, uint8_t> batch;
    for (size_t i = 0; i < msg->frontier_ids.size() && i < msg->scores.size(); ++i) {
      const uint32_t id = msg->frontier_ids[i];
      accumulated_scores_[id] = msg->scores[i];
      batch[id] = msg->scores[i];
      if (i < msg->reasonings.size()) {
        accumulated_reasonings_[id] = msg->reasonings[i];
      }
    }
    if (brain_ && !batch.empty()) {
      brain_->onVlmScores(batch);
    }
    publishTree();
  }

  uint32_t detectKey() const
  {
    if (brain_ && brain_->usesFrontierTree() && brain_->frontierTree()) {
      return brain_->frontierTree()->currentNodeId();
    }
    return greedy_detect_key_;
  }

  uint32_t statusNodeId() const
  {
    return detectKey();
  }

  cv::Point2f scanPoseFor(uint32_t scan_id) const
  {
    const auto it = scan_poses_.find(scan_id);
    if (it != scan_poses_.end()) {
      return it->second;
    }
    return current_pos_;
  }

  void publishTree()
  {
    if (!tree_pub_) {
      return;
    }
    const rclcpp::Time stamp = now();
    const int64_t total_ns = stamp.nanoseconds();
    const int32_t sec = static_cast<int32_t>(total_ns / 1000000000LL);
    const uint32_t nsec = static_cast<uint32_t>(total_ns % 1000000000LL);

    const explorer_mission::FrontierTree * tree =
      brain_ ? brain_->frontierTree() : nullptr;
    if (tree) {
      tree_pub_->publish(tree->toMsg(map_frame_, sec, nsec));
      return;
    }
    if (!brain_) {
      return;
    }
    // Non-tree brains: publish a flat synthetic tree so maprender green dots work.
    const auto viz = brain_->vizNodes();
    explorer_msgs::msg::FrontierTree msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nsec;
    msg.current_node_id = brain_->vizCurrentNodeId();
    msg.nodes.reserve(viz.size());
    for (const auto & node : viz) {
      explorer_msgs::msg::FrontierTreeNode out;
      out.id = node.id;
      out.position.x = node.position.x;
      out.position.y = node.position.y;
      out.position.z = 0.0;
      out.parent_id = -1;
      out.openness_score = node.openness_score;
      out.fully_explored = node.dead;
      out.visited = node.visited;
      msg.nodes.push_back(out);
    }
    tree_pub_->publish(msg);
  }

  void publishPendingGraphEdges()
  {
    if (!brain_graph_edges_pub_ || !brain_) {
      return;
    }
    const auto edges = brain_->takePendingGraphEdges();
    if (edges.empty()) {
      return;
    }
    explorer_msgs::msg::BrainGraphEdges msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.brain_id = brain_->id();
    msg.from_ids.reserve(edges.size());
    msg.to_ids.reserve(edges.size());
    msg.costs.reserve(edges.size());
    for (const auto & e : edges) {
      msg.from_ids.push_back(e.from_id);
      msg.to_ids.push_back(e.to_id);
      msg.costs.push_back(e.cost);
    }
    brain_graph_edges_pub_->publish(msg);
  }

  void publishPendingVlmChoice()
  {
    if (!vlm_choice_pub_ || !brain_) {
      return;
    }
    const auto rec = brain_->takePendingVlmChoice();
    if (!rec.has_value()) {
      return;
    }
    explorer_msgs::msg::VlmChoiceEvent msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.brain_id = brain_->id();
    msg.prompt = rec->prompt;
    msg.response = rec->response;
    msg.selected_frontier_id = rec->selected_frontier_id;
    msg.candidate_ids = rec->candidate_ids;
    vlm_choice_pub_->publish(msg);
  }

  void publishPendingBrainTelemetry()
  {
    publishPendingGraphEdges();
    publishPendingVlmChoice();
  }

  void publishBrainDecision(const explorer_mission::BrainDecision & decision)
  {
    if (!brain_decision_pub_ || !brain_) {
      return;
    }
    explorer_msgs::msg::BrainDecisionEvent msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.brain_id = brain_->id();
    switch (decision.action) {
      case explorer_mission::BrainAction::kNavigateTo:
        msg.action = "navigate";
        break;
      case explorer_mission::BrainAction::kWait:
        msg.action = "wait";
        break;
      case explorer_mission::BrainAction::kComplete:
      default:
        msg.action = "complete";
        break;
    }
    msg.goal_id = decision.goal_id;
    msg.goal_x = decision.goal.x;
    msg.goal_y = decision.goal.y;
    msg.detail = decision.detail;
    msg.visited_ids = brain_->visitedIds();
    msg.live_ids = brain_->liveFrontierIds();
    brain_decision_pub_->publish(msg);
  }

  void publishPhase(
    const std::string & phase,
    uint32_t current_id,
    uint32_t target_id,
    bool complete,
    const std::string & detail,
    const std::string & termination_reason = "")
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
    status.termination_reason = termination_reason;
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

  bool detectAndOfferToBrain()
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

    publishPhase("detecting", statusNodeId(), 0, false, "on-demand frontier detection");

    std::vector<cv::Point2f> exclusion_centers;
    if (brain_->usesFrontierTree() && brain_->frontierTree()) {
      exclusion_centers = brain_->frontierTree()->allNodePositions();
    } else {
      exclusion_centers = greedy_visited_poses_;
    }
    const cv::Mat mask = explorer_mission::buildExclusionMask(
      grid, exclusion_centers, frontier_exclusion_radius_);
    auto contours = explorer_mission::findFrontierContoursMasked(
      grid, mask, min_contour_pixels_);
    contours = explorer_mission::filterContoursNearRobot(
      contours, grid, current_pos_, frontier_detection_radius_);
    const size_t before_dedupe = contours.size();
    contours = explorer_mission::dedupeContoursByMidpoint(
      contours, grid, frontier_exclusion_radius_);
    RCLCPP_INFO(
      get_logger(),
      "Frontier detect: kept=%zu (deduped %zu→%zu) keep_r=%.1fm excl_r=%.1fm",
      contours.size(), before_dedupe, contours.size(),
      frontier_detection_radius_, frontier_exclusion_radius_);

    std::vector<explorer_mission::FrontierCandidate> candidates;
    candidates.reserve(contours.size());
    uint32_t greedy_id_base = greedy_detect_key_ + 1;
    for (size_t i = 0; i < contours.size(); ++i) {
      const cv::Point2f midpoint =
        explorer_mission::frontierMidpointWorld(contours[i], grid);
      // Pull goal off the free↔unknown edge so Nav2/footprint can plan & arrive.
      constexpr double kFrontierGoalInsetM = 0.35;
      const cv::Point2f goal = explorer_mission::insetFrontierGoalWorld(
        grid, midpoint, kFrontierGoalInsetM);
      explorer_mission::FrontierCandidate c;
      // Tree brain ignores id; greedy needs stable ids within this detect batch.
      c.id = brain_->usesFrontierTree() ? 0u : (greedy_id_base + static_cast<uint32_t>(i));
      c.position = goal;
      candidates.push_back(c);
    }

    const std::vector<uint32_t> new_ids = brain_->onFrontiersDetected(candidates);
    publishTree();
    publishPendingGraphEdges();

    if (new_ids.empty()) {
      publishPhase("selecting", statusNodeId(), 0, false, "no new frontiers detected");
      return true;
    }

    if (!brain_->wantsVlmScores()) {
      publishPhase("selecting", statusNodeId(), 0, false, "brain skips VLM");
      return true;
    }

    if (cached_images_.empty()) {
      RCLCPP_WARN(get_logger(), "No cached scan images; soft-failing new frontiers to score=1");
      brain_->softFailUnrated(1);
      publishTree();
      return true;
    }

    auto views = buildFrontierViews(new_ids);
    if (views.frontier_ids.empty()) {
      RCLCPP_WARN(get_logger(), "Failed to match images to frontiers; soft-fail score=1");
      brain_->softFailUnrated(1);
      publishTree();
      return true;
    }

    publishPhase(
      "awaiting_vlm", statusNodeId(), 0, false,
      "rating " + std::to_string(views.frontier_ids.size()) + " frontiers");
    if (!waitForVlmScores(views.frontier_ids)) {
      RCLCPP_WARN(
        get_logger(),
        "VLM scores timeout; soft-failing still-unrated children to score=1");
      applyLatestScoresToBrain();
      brain_->softFailUnrated(1);
      publishTree();
      return true;
    }

    applyLatestScoresToBrain();
    publishTree();
    publishPhase("selecting", statusNodeId(), 0, false, "VLM scores applied");
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

    const explorer_mission::FrontierTree * tree = brain_->frontierTree();
    if (!tree) {
      return msg;
    }

    updateRobotPoseFromTf();
    for (uint32_t child_id : child_ids) {
      const explorer_mission::TreeNode * node = tree->find(child_id);
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
      for (uint32_t id : expected_ids) {
        accumulated_scores_.erase(id);
        accumulated_reasonings_.erase(id);
      }
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
      if (!scores_received_ && accumulated_scores_.empty()) {
        continue;
      }
      size_t rated = 0;
      bool early = false;
      for (uint32_t id : expected) {
        const auto it = accumulated_scores_.find(id);
        if (it == accumulated_scores_.end()) {
          continue;
        }
        ++rated;
        if (static_cast<int>(it->second) >= early_nav_min_score_) {
          early = true;
        }
      }
      if (early) {
        RCLCPP_INFO(
          get_logger(),
          "Early nav: at least one frontier scored >= %d (%zu/%zu rated)",
          early_nav_min_score_, rated, expected.size());
        applyLatestScoresToBrainLocked();
        return true;
      }
      if (rated == expected.size() && !expected.empty()) {
        applyLatestScoresToBrainLocked();
        return true;
      }
    }
    return false;
  }

  void applyLatestScoresToBrain()
  {
    std::lock_guard<std::mutex> lock(scores_mutex_);
    applyLatestScoresToBrainLocked();
  }

  void applyLatestScoresToBrainLocked()
  {
    if (!brain_ || accumulated_scores_.empty()) {
      return;
    }
    std::unordered_map<uint32_t, uint8_t> batch;
    for (const auto & entry : accumulated_scores_) {
      batch[entry.first] = entry.second;
    }
    brain_->onVlmScores(batch);
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

  void performScanIfNeeded(bool force = false)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    bool should_scan = force;
    if (!should_scan) {
      // Legacy distance heuristic removed: callers pass force on unvisited arrivals.
      return;
    }

    if (!rotate_client_->wait_for_action_server(std::chrono::seconds(5))) {
      return;
    }

    const char * scan_why = (counter_ == 0)
      ? "rotate_360 first scan"
      : "rotate_360 (unvisited frontier)";
    publishPhase("scanning", statusNodeId(), 0, false, scan_why);

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
    publishPhase("scanning", statusNodeId(), 0, false, "rotate_360 done");
  }

  bool returnToScanPoseWithRetry(uint32_t scan_id, const cv::Point2f & scan_pose)
  {
    updateRobotPoseFromTf();
    const double dist_m = std::hypot(
      static_cast<double>(current_pos_.x - scan_pose.x),
      static_cast<double>(current_pos_.y - scan_pose.y));
    if (goal_accept_radius_m_ > 0.0 && dist_m <= goal_accept_radius_m_) {
      RCLCPP_INFO(
        get_logger(),
        "Already within %.2f m of scan node %u (dist=%.3f); skip return-home nav",
        goal_accept_radius_m_, scan_id, dist_m);
      return_home_guard_.onReturnHomeSucceeded();
      return true;
    }

    publishPhase(
      "backtracking", statusNodeId(), scan_id, false,
      "nav/plan failed — return to scan node before next frontier");

    for (int attempt = 1; attempt <= return_home_max_attempts_; ++attempt) {
      if (navigateWithUnstickRecovery(scan_pose, scan_pose).ok) {
        return_home_guard_.onReturnHomeSucceeded();
        return true;
      }

      updateRobotPoseFromTf();
      const double d = std::hypot(
        static_cast<double>(current_pos_.x - scan_pose.x),
        static_cast<double>(current_pos_.y - scan_pose.y));
      if (goal_accept_radius_m_ > 0.0 && d <= goal_accept_radius_m_) {
        return_home_guard_.onReturnHomeSucceeded();
        return true;
      }

      RCLCPP_WARN(
        get_logger(),
        "Return to scan node %u failed (attempt %d/%d); retrying.",
        scan_id, attempt, return_home_max_attempts_);
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_ERROR(
      get_logger(),
      "Return to scan node %u abandoned after %d attempts; clearing return-home guard",
      scan_id, return_home_max_attempts_);
    return_home_guard_.onReturnHomeAbandoned();
    return false;
  }

  struct NavAttemptResult
  {
    bool ok{false};
    bool mark_frontier_dead{true};
    bool terminate_stuck{false};
  };

  bool nearPose(const cv::Point2f & pose) const
  {
    if (goal_accept_radius_m_ <= 0.0) {
      return false;
    }
    return std::hypot(
      static_cast<double>(current_pos_.x - pose.x),
      static_cast<double>(current_pos_.y - pose.y)) <= goal_accept_radius_m_;
  }

  bool theoreticalPlanTo(const cv::Point2f & pose)
  {
    updateRobotPoseFromTf();
    if (nearPose(pose)) {
      return true;
    }
    if (navigation_mode_ == "nav2" && nav2_navigator_) {
      return nav2_navigator_->computePathExists(
        pose.x, pose.y, 0.0, map_frame_, 15.0);
    }
    // Discrete mode: straight-line plan always "exists" geometrically.
    return true;
  }

  /// NEW-frontier nav fail policy: inaccessible vs stuck recovery.
  NavAttemptResult navigateNewGoalWithRecovery(
    const cv::Point2f & goal_pos,
    const cv::Point2f & look_at,
    uint32_t prior_id)
  {
    const rclcpp::Time nav_start = now();
    if (navigateToPosition(goal_pos, look_at)) {
      return NavAttemptResult{true, false, false};
    }
    const double nav_dt_s = (now() - nav_start).seconds();
    const bool nav_substantive = explorer_mission::isSubstantiveNavAttempt(nav_dt_s);

    const uint16_t nav_error_code =
      (nav2_navigator_ != nullptr) ? nav2_navigator_->lastErrorCode() : 0;
    const std::string nav_error =
      (nav2_navigator_ != nullptr) ? nav2_navigator_->lastError() : std::string{};

    const bool prior_known = scan_poses_.count(prior_id) != 0;
    const cv::Point2f prior_pose = scanPoseFor(prior_id);
    const bool prior_plan_ok = prior_known && theoreticalPlanTo(prior_pose);

    // Explicit NEW-goal ComputePath check (captures error_code for logging /
    // definitive-unreachable gating). Do not treat TF/timeout/unknown as inaccessible.
    bool new_goal_plan_ok = false;
    uint16_t new_plan_code = 0;
    std::string new_plan_err;
    updateRobotPoseFromTf();
    if (nearPose(goal_pos)) {
      new_goal_plan_ok = true;
    } else if (navigation_mode_ == "nav2" && nav2_navigator_) {
      new_goal_plan_ok = nav2_navigator_->computePathExists(
        goal_pos.x, goal_pos.y, 0.0, map_frame_, 15.0);
      new_plan_code = nav2_navigator_->lastErrorCode();
      new_plan_err = nav2_navigator_->lastError();
    } else {
      new_goal_plan_ok = true;
    }
    const bool new_unreachable_definitive =
      !new_goal_plan_ok &&
      explorer_mission::isDefinitiveGoalUnreachable(new_plan_code);

    const double start_clearance = currentClearanceM();
    const bool start_ok = explorer_mission::isStartClearanceOk(
      start_clearance, unstick_min_clearance_m_);

    RCLCPP_WARN(
      get_logger(),
      "NEW frontier nav fail: nav_dt=%.3fs substantive=%d nav_err='%s' nav_code=%u "
      "new_plan_ok=%d new_plan_code=%u new_plan_err='%s' definitive=%d "
      "prior_plan_ok=%d prior=%u start_clearance=%.3f start_ok=%d",
      nav_dt_s, static_cast<int>(nav_substantive),
      nav_error.c_str(), static_cast<unsigned>(nav_error_code),
      static_cast<int>(new_goal_plan_ok), static_cast<unsigned>(new_plan_code),
      new_plan_err.c_str(), static_cast<int>(new_unreachable_definitive),
      static_cast<int>(prior_plan_ok), prior_id,
      start_clearance, static_cast<int>(start_ok));

    const auto fail_class = explorer_mission::classifyNewGoalNavFailure(
      prior_known, prior_plan_ok, new_goal_plan_ok, new_unreachable_definitive,
      start_ok);

    if (fail_class == explorer_mission::NewGoalFailClass::kInaccessible) {
      RCLCPP_WARN(
        get_logger(),
        "NEW frontier definitively unplannable (code=%u) and prior %u reachable "
        "(nav_dt=%.2fs start_ok) — mark inaccessible (no thrash)",
        static_cast<unsigned>(new_plan_code), prior_id, nav_dt_s);
      return NavAttemptResult{false, true, false};
    }

    // Stuck: wedged start, prior unreachable, NEW still plannable, or inconclusive.
    RCLCPP_WARN(
      get_logger(),
      "NEW frontier nav fail not definitive inaccessible — treating as stuck "
      "(will thrash then deflate-return)");

    publishPhase(
      "unsticking", statusNodeId(), prior_id, false,
      "stuck recovery: wall thrash");
    for (int i = 0; i < unstick_max_attempts_; ++i) {
      runWallUnstick(i);
    }

    publishPhase(
      "backtracking", statusNodeId(), prior_id, false,
      "stuck recovery: deflate + return to previous");
    applyCostmapInflation(0.0, 0.0);
    bool back_ok = navigateToPosition(prior_pose, prior_pose);
    if (!back_ok) {
      for (int i = 0; i < unstick_max_attempts_ && !back_ok; ++i) {
        runWallUnstick(i);
        back_ok = navigateToPosition(prior_pose, prior_pose);
      }
    }
    applyCostmapInflation(nav2_inflation_radius_m_, mapper_inflation_m_);
    updateRobotPoseFromTf();
    if (!back_ok && nearPose(prior_pose)) {
      back_ok = true;
    }
    if (!back_ok) {
      RCLCPP_ERROR(
        get_logger(),
        "Stuck recovery failed: cannot return to prior node %u", prior_id);
      return NavAttemptResult{false, false, true};
    }

    RCLCPP_INFO(
      get_logger(),
      "Returned to prior %u; retrying NEW frontier with normal inflation",
      prior_id);
    if (navigateToPosition(goal_pos, look_at)) {
      return NavAttemptResult{true, false, false};
    }
    updateRobotPoseFromTf();
    const bool start_ok_after = explorer_mission::isStartClearanceOk(
      currentClearanceM(), unstick_min_clearance_m_);
    if (!explorer_mission::shouldMarkDeadAfterStuckRecovery(start_ok_after)) {
      RCLCPP_ERROR(
        get_logger(),
        "Still wedged after stuck recovery (clearance low) — ending episode stuck "
        "instead of blacklisting remaining frontiers");
      return NavAttemptResult{false, false, true};
    }
    RCLCPP_WARN(
      get_logger(),
      "NEW frontier still unreachable after unstuck (start clear) — marking dead");
    return NavAttemptResult{false, true, false};
  }

  double currentClearanceM()
  {
    nav_msgs::msg::OccupancyGrid grid;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      if (!have_grid_) {
        return std::numeric_limits<double>::infinity();
      }
      grid = latest_grid_;
    }
    const auto c = explorer_mission::clearanceToOccupiedM(
      grid, current_pos_.x, current_pos_.y);
    return c.value_or(std::numeric_limits<double>::infinity());
  }

  uint16_t lastNavErrorCode() const
  {
    if (navigation_mode_ == "nav2" && nav2_navigator_) {
      return nav2_navigator_->lastErrorCode();
    }
    return explorer_mission::kNavErrorNone;
  }

  bool sendOneDiscreteMove(int direction, int steps)
  {
    DiscreteMove::Goal goal;
    goal.direction = static_cast<uint8_t>(direction);
    goal.steps = static_cast<uint32_t>(steps);
    auto future = discrete_move_client_->async_send_goal(goal);
    if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      return false;
    }
    const auto goal_handle = future.get();
    if (!goal_handle) {
      return false;
    }
    auto result_future = discrete_move_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
      discrete_move_client_->async_cancel_goal(goal_handle);
      return false;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result->success) {
      return false;
    }
    updateRobotPoseFromTf();
    return true;
  }

  void runWallUnstick(int escalation_level)
  {
    // Alternate BACKWARD / FORWARD with increasing step count so we can escape
    // whether the occlusion is in front of or behind the robot (no Nav2).
    const auto motion = explorer_mission::unstickThrashMotion(escalation_level);
    const char * dir_name =
      motion.direction == explorer_mission::kUnstickBackward ? "BACKWARD" : "FORWARD";
    RCLCPP_WARN(
      get_logger(),
      "Wall unstick attempt %d/%d: DiscreteMove %s x%d (thrash, no Nav2)",
      escalation_level + 1, unstick_max_attempts_, dir_name, motion.steps);
    if (!sendOneDiscreteMove(motion.direction, motion.steps)) {
      RCLCPP_WARN(
        get_logger(), "Wall unstick: DiscreteMove %s x%d failed",
        dir_name, motion.steps);
    }
  }

  bool setRemoteDoubleParam(
    const std::string & remote_node,
    const std::string & param_name,
    double value)
  {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, remote_node);
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(
        get_logger(), "Param service unavailable for %s (skip %s=%.3f)",
        remote_node.c_str(), param_name.c_str(), value);
      return false;
    }
    auto future = client->set_parameters({rclcpp::Parameter(param_name, value)});
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN(
        get_logger(), "Timed out setting %s on %s",
        param_name.c_str(), remote_node.c_str());
      return false;
    }
    const auto results = future.get();
    for (const auto & r : results) {
      if (!r.successful) {
        RCLCPP_WARN(
          get_logger(), "Failed setting %s on %s: %s",
          param_name.c_str(), remote_node.c_str(), r.reason.c_str());
        return false;
      }
    }
    return true;
  }

  void applyCostmapInflation(double nav2_radius_m, double mapper_inflation_m)
  {
    setRemoteDoubleParam(
      "/global_costmap/global_costmap",
      "inflation_layer.inflation_radius", nav2_radius_m);
    setRemoteDoubleParam(
      "/local_costmap/local_costmap",
      "inflation_layer.inflation_radius", nav2_radius_m);
    // One of these mappers is active depending on launch; both are best-effort.
    setRemoteDoubleParam("known_pose_pc_mapper", "obstacle_inflation_m", mapper_inflation_m);
    setRemoteDoubleParam("known_pose_mapper", "obstacle_inflation_m", mapper_inflation_m);
    // Allow a costmap / grid republish before the next NavigateToPose.
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  NavAttemptResult navigateWithUnstickRecovery(
    const cv::Point2f & goal_pos, const cv::Point2f & look_at)
  {
    if (navigateToPosition(goal_pos, look_at)) {
      return NavAttemptResult{true, false, false};
    }

    int unstick_attempts = 0;
    bool zero_inflation_done = false;
    while (true) {
      const uint16_t code = lastNavErrorCode();
      const double clearance = currentClearanceM();
      auto decision = explorer_mission::decideNavFailureRecovery(
        code, clearance, unstick_min_clearance_m_, unstick_attempts, unstick_max_attempts_,
        zero_inflation_done);
      RCLCPP_WARN(
        get_logger(),
        "Nav failed (error_code=%u clearance=%.3f attempts=%d/%d): "
        "unstick=%d zero_infl=%d mark=%d — %s",
        static_cast<unsigned>(code), clearance,
        unstick_attempts, unstick_max_attempts_,
        static_cast<int>(decision.attempt_unstick),
        static_cast<int>(decision.attempt_zero_inflation),
        static_cast<int>(decision.mark_frontier_dead),
        navigation_mode_ == "nav2" && nav2_navigator_ ?
        nav2_navigator_->lastError().c_str() : "n/a");

      if (decision.attempt_unstick) {
        runWallUnstick(unstick_attempts);
        ++unstick_attempts;
        if (navigateToPosition(goal_pos, look_at)) {
          return NavAttemptResult{true, false, false};
        }
        continue;
      }

      if (decision.attempt_zero_inflation) {
        RCLCPP_WARN(
          get_logger(),
          "Last-ditch recovery: NavigateToPose with zero inflation "
          "(nav2 radius=0, mapper inflate=0)");
        applyCostmapInflation(0.0, 0.0);
        const bool ok = navigateToPosition(goal_pos, look_at);
        applyCostmapInflation(nav2_inflation_radius_m_, mapper_inflation_m_);
        zero_inflation_done = true;
        if (ok) {
          return NavAttemptResult{true, false, false};
        }
        continue;
      }

      return NavAttemptResult{false, decision.mark_frontier_dead, false};
    }
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
    // Short-circuit before calling Nav2: goals at/near current pose (esp. return-home
    // to the scan node we never left) otherwise send NavigateToPose to self and fail.
    if (goal_accept_radius_m_ > 0.0) {
      const double dist_m = std::hypot(
        static_cast<double>(current_pos_.x) - goal_x,
        static_cast<double>(current_pos_.y) - goal_y);
      if (dist_m <= goal_accept_radius_m_) {
        return true;
      }
    }
    if (navigation_mode_ == "nav2" && nav2_navigator_) {
      const double goal_yaw_rad = goal_yaw_deg * M_PI / 180.0;
      const bool ok = nav2_navigator_->navigateToPose(
        goal_x, goal_y, goal_yaw_rad, map_frame_,
        explorer_mission::kNavTotalTimeoutS,
        explorer_mission::kNavStuckTimeoutS,
        explorer_mission::kNavStuckDistanceM,
        [this]() {
          updateRobotPoseFromTf();
          nav2_navigator_->noteProgress(
            current_pos_.x, current_pos_.y,
            explorer_mission::kNavStuckDistanceM,
            explorer_mission::kNavStuckTimeoutS);
          return true;
        },
        goal_accept_radius_m_);
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
  double frontier_detection_radius_{50.0};
  double frontier_exclusion_radius_{1.0};
  int min_contour_pixels_{15};
  double vlm_scores_timeout_s_{120.0};
  bool publish_debug_topics_{false};
  std::string brain_id_{"vlm_tree_dfs"};
  bool dfs_prefer_highest_openness_{true};
  double goal_accept_radius_m_{1.0};
  bool parent_to_nearest_node_{true};
  int early_nav_min_score_{3};
  int return_home_max_attempts_{20};
  double unstick_min_clearance_m_{0.25};
  int unstick_max_steps_{8};
  int unstick_max_attempts_{explorer_mission::kDefaultMaxUnstickAttempts};
  /// Nominal inflation restored after last-ditch zero-inflation retry.
  double nav2_inflation_radius_m_{0.22};
  double mapper_inflation_m_{0.05};

  explorer_mission::ReturnHomeGuard return_home_guard_;

  std::unique_ptr<explorer_mission::Nav2Navigator> nav2_navigator_;
  std::unique_ptr<explorer_mission::ExplorationBrain> brain_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::unordered_set<uint32_t> scanned_detect_keys_;
  std::map<uint32_t, cv::Point2f> scan_poses_;
  std::vector<cv::Point2f> greedy_visited_poses_;
  uint32_t greedy_detect_key_{0};
  std::map<uint32_t, uint8_t> accumulated_scores_;
  std::map<uint32_t, std::string> accumulated_reasonings_;

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
  rclcpp::Publisher<explorer_msgs::msg::BrainDecisionEvent>::SharedPtr brain_decision_pub_;
  rclcpp::Publisher<explorer_msgs::msg::BrainGraphEdges>::SharedPtr brain_graph_edges_pub_;
  rclcpp::Publisher<explorer_msgs::msg::VlmChoiceEvent>::SharedPtr vlm_choice_pub_;
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
