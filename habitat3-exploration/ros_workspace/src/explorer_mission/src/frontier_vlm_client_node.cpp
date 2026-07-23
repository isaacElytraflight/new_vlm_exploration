#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <explorer_msgs/action/rate_frontier_openness.hpp>
#include <explorer_msgs/msg/frontier_openness_scores.hpp>
#include <explorer_msgs/msg/frontier_views.hpp>

using RateFrontierOpenness = explorer_msgs::action::RateFrontierOpenness;

class FrontierVlmClientNode : public rclcpp::Node
{
public:
  FrontierVlmClientNode()
  : Node("frontier_vlm_client")
  {
    timeout_s_ = declare_parameter<double>("result_timeout_s", 300.0);

    scores_pub_ = create_publisher<explorer_msgs::msg::FrontierOpennessScores>(
      "exploration/vlm/scores", rclcpp::QoS(10));

    vlm_client_ = rclcpp_action::create_client<RateFrontierOpenness>(this, "vlm/rate_frontiers");

    views_sub_ = create_subscription<explorer_msgs::msg::FrontierViews>(
      "exploration/vlm/views", rclcpp::QoS(10),
      std::bind(&FrontierVlmClientNode::viewsCb, this, std::placeholders::_1));

    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FrontierVlmClientNode::checkTimeout, this));

    RCLCPP_INFO(get_logger(), "Waiting for VLM rate_frontiers action server...");
    if (!waitForVlmServer()) {
      RCLCPP_ERROR(
        get_logger(),
        "VLM rate_frontiers action server unavailable — "
        "ensure vlm_node is running and Ollama is reachable.");
    } else {
      RCLCPP_INFO(get_logger(), "Connected to VLM rate_frontiers action server.");
    }
  }

  bool waitForVlmServer()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (vlm_client_->wait_for_action_server(std::chrono::seconds(0))) {
        vlm_ready_ = true;
        return true;
      }
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    return false;
  }

  void viewsCb(const explorer_msgs::msg::FrontierViews::SharedPtr fv)
  {
    if (!vlm_ready_ && !waitForVlmServer()) {
      RCLCPP_WARN(get_logger(), "Dropping FrontierViews batch; VLM server not ready.");
      return;
    }
    if (fv->frontier_ids.empty()) {
      return;
    }
    if (fv->images.size() != fv->frontier_ids.size()) {
      RCLCPP_WARN(get_logger(), "FrontierViews image/id size mismatch");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (inflight_) {
      queue_.push_back(fv);
      RCLCPP_INFO(
        get_logger(),
        "Queued FrontierViews batch (%zu frontiers); queue depth=%zu",
        fv->frontier_ids.size(), queue_.size());
      return;
    }
    startGoalLocked(fv);
  }

  void startGoalLocked(const explorer_msgs::msg::FrontierViews::SharedPtr fv)
  {
    auto goal = RateFrontierOpenness::Goal();
    goal.images = fv->images;
    goal.frontier_ids = fv->frontier_ids;

    inflight_ = true;
    ++goal_gen_;
    const uint64_t gen = goal_gen_;
    inflight_ids_.clear();
    for (uint32_t id : fv->frontier_ids) {
      inflight_ids_.insert(id);
    }
    goal_start_ = now();
    active_fv_ = fv;

    auto send_options = rclcpp_action::Client<RateFrontierOpenness>::SendGoalOptions();
    send_options.result_callback =
      [this, gen](const rclcpp_action::ClientGoalHandle<RateFrontierOpenness>::WrappedResult & result) {
        onResult(result, gen);
      };

    vlm_client_->async_send_goal(goal, send_options);
    RCLCPP_INFO(
      get_logger(), "Sent VLM rating goal for %zu frontiers",
      fv->frontier_ids.size());
  }

  void onResult(
    const rclcpp_action::ClientGoalHandle<RateFrontierOpenness>::WrappedResult & result,
    uint64_t gen)
  {
    explorer_msgs::msg::FrontierViews::SharedPtr fv;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (gen != goal_gen_) {
        // Timed out / superseded; ignore stale result.
        return;
      }
      fv = active_fv_;
      active_fv_.reset();
      inflight_ = false;
      inflight_ids_.clear();
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && fv) {
      explorer_msgs::msg::FrontierOpennessScores msg;
      msg.header = fv->header;
      msg.frontier_ids = result.result->frontier_ids;
      msg.scores = result.result->scores;
      msg.reasonings = result.result->reasonings;
      scores_pub_->publish(msg);
      RCLCPP_INFO(
        get_logger(), "Published openness scores for %zu frontiers",
        msg.frontier_ids.size());
    } else {
      RCLCPP_WARN(
        get_logger(), "VLM rating finished with code %d",
        static_cast<int>(result.code));
    }

    pumpQueue();
  }

  void pumpQueue()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (inflight_ || queue_.empty()) {
      return;
    }
    auto next = queue_.front();
    queue_.pop_front();
    startGoalLocked(next);
  }

  void checkTimeout()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!inflight_) {
      return;
    }
    if ((now() - goal_start_).seconds() < timeout_s_) {
      return;
    }
    RCLCPP_WARN(get_logger(), "VLM rating timed out after %.1f s", timeout_s_);
    ++goal_gen_;  // invalidate in-flight result callback
    inflight_ = false;
    active_fv_.reset();
    inflight_ids_.clear();
    lock.unlock();
    vlm_client_->async_cancel_all_goals();
    pumpQueue();
  }

  rclcpp::Time now() const {return get_clock()->now();}

  double timeout_s_{300.0};
  bool inflight_{false};
  uint64_t goal_gen_{0};
  rclcpp::Time goal_start_;
  std::set<uint32_t> inflight_ids_;
  std::mutex mutex_;
  std::deque<explorer_msgs::msg::FrontierViews::SharedPtr> queue_;
  explorer_msgs::msg::FrontierViews::SharedPtr active_fv_;

  rclcpp::Publisher<explorer_msgs::msg::FrontierOpennessScores>::SharedPtr scores_pub_;
  rclcpp_action::Client<RateFrontierOpenness>::SharedPtr vlm_client_;
  rclcpp::Subscription<explorer_msgs::msg::FrontierViews>::SharedPtr views_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  bool vlm_ready_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierVlmClientNode>());
  rclcpp::shutdown();
  return 0;
}
