#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
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
    timeout_s_ = declare_parameter<double>("result_timeout_s", 120.0);

    scores_pub_ = create_publisher<explorer_msgs::msg::FrontierOpennessScores>(
      "exploration/vlm/scores", rclcpp::QoS(1));

    vlm_client_ = rclcpp_action::create_client<RateFrontierOpenness>(this, "vlm/rate_frontiers");

    views_sub_ = create_subscription<explorer_msgs::msg::FrontierViews>(
      "exploration/vlm/views", rclcpp::QoS(1),
      std::bind(&FrontierVlmClientNode::viewsCb, this, std::placeholders::_1));

    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FrontierVlmClientNode::checkTimeout, this));

    RCLCPP_INFO(get_logger(), "Waiting for VLM rate_frontiers action server...");
    if (!vlm_client_->wait_for_action_server(std::chrono::seconds(15))) {
      throw std::runtime_error("VLM rate_frontiers action server unavailable");
    }
    RCLCPP_INFO(get_logger(), "Connected to VLM rate_frontiers action server.");
  }

private:
  void viewsCb(const explorer_msgs::msg::FrontierViews::SharedPtr fv)
  {
    if (fv->frontier_ids.empty()) {
      RCLCPP_DEBUG(get_logger(), "Empty FrontierViews batch; skipping.");
      return;
    }
    if (fv->images.size() != fv->frontier_ids.size()) {
      RCLCPP_WARN(get_logger(), "FrontierViews image/id size mismatch");
      return;
    }
    if (inflight_) {
      RCLCPP_WARN(get_logger(), "VLM rating already in flight; dropping batch.");
      return;
    }
    tryProcessViews(fv);
  }

  void tryProcessViews(const explorer_msgs::msg::FrontierViews::SharedPtr fv)
  {
    auto goal = RateFrontierOpenness::Goal();
    goal.images = fv->images;
    goal.frontier_ids = fv->frontier_ids;

    inflight_ = true;
    inflight_ids_.clear();
    for (uint32_t id : fv->frontier_ids) {
      inflight_ids_.insert(id);
    }
    goal_start_ = now();

    auto send_options = rclcpp_action::Client<RateFrontierOpenness>::SendGoalOptions();
    send_options.result_callback = [this, fv](
      const rclcpp_action::ClientGoalHandle<RateFrontierOpenness>::WrappedResult & result) {
        inflight_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "VLM rating failed with code %d", static_cast<int>(result.code));
          return;
        }
        explorer_msgs::msg::FrontierOpennessScores msg;
        msg.header = fv->header;
        msg.frontier_ids = result.result->frontier_ids;
        msg.scores = result.result->scores;
        scores_pub_->publish(msg);
        RCLCPP_INFO(
          get_logger(), "Published openness scores for %zu frontiers",
          msg.frontier_ids.size());
      };

    vlm_client_->async_send_goal(goal, send_options);
    RCLCPP_INFO(
      get_logger(), "Sent VLM rating goal for %zu frontiers",
      fv->frontier_ids.size());
  }

  void checkTimeout()
  {
    if (!inflight_) {
      return;
    }
    if ((now() - goal_start_).seconds() >= timeout_s_) {
      RCLCPP_WARN(get_logger(), "VLM rating timed out after %.1f s", timeout_s_);
      inflight_ = false;
      vlm_client_->async_cancel_all_goals();
    }
  }

  rclcpp::Time now() const {return get_clock()->now();}

  double timeout_s_{120.0};
  bool inflight_{false};
  rclcpp::Time goal_start_;
  std::set<uint32_t> inflight_ids_;

  rclcpp::Publisher<explorer_msgs::msg::FrontierOpennessScores>::SharedPtr scores_pub_;
  rclcpp_action::Client<RateFrontierOpenness>::SharedPtr vlm_client_;
  rclcpp::Subscription<explorer_msgs::msg::FrontierViews>::SharedPtr views_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierVlmClientNode>());
  rclcpp::shutdown();
  return 0;
}
