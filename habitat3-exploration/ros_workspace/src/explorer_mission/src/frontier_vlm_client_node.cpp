#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int8.hpp>

#include <explorer_msgs/action/frontier_views_process.hpp>
#include <explorer_msgs/msg/frontier_views.hpp>

using FrontierViewsProcess = explorer_msgs::action::FrontierViewsProcess;

class FrontierVlmClientNode : public rclcpp::Node
{
public:
  FrontierVlmClientNode()
  : Node("frontier_vlm_client")
  {
    require_frontiers_ = declare_parameter<bool>("require_frontiers", true);
    dedupe_ = declare_parameter<bool>("dedupe_batches", true);
    timeout_s_ = declare_parameter<double>("result_timeout_s", 30.0);

    chosen_pub_ = create_publisher<std_msgs::msg::Int8>("chosen_frontier", rclcpp::QoS(1).transient_local());

    vlm_client_ = rclcpp_action::create_client<FrontierViewsProcess>(this, "vlm/query");

    map_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "/map_renderer/map_img", rclcpp::QoS(1),
      std::bind(&FrontierVlmClientNode::mapCb, this, std::placeholders::_1));

    views_sub_ = create_subscription<explorer_msgs::msg::FrontierViews>(
      "/frontiers/frontier_views", rclcpp::QoS(1),
      std::bind(&FrontierVlmClientNode::viewsCb, this, std::placeholders::_1));

    // Wall timer to enforce a timeout on an in-flight VLM goal without blocking
    // (callbacks are serviced by the single executor in main()).
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FrontierVlmClientNode::checkTimeout, this));

    RCLCPP_INFO(get_logger(), "Waiting for VLM action server...");
    if (!vlm_client_->wait_for_action_server(std::chrono::seconds(15))) {
      throw std::runtime_error("VLM action server unavailable");
    }
    RCLCPP_INFO(get_logger(), "Connected to VLM action server.");
  }

private:
  void mapCb(const sensor_msgs::msg::CompressedImage::SharedPtr map_msg)
  {
    explorer_msgs::msg::FrontierViews::SharedPtr pending;
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      latest_map_ = map_msg;
      pending = pending_views_;
    }
    if (pending) {
      tryProcessViews(pending);
    }
  }

  // Event-driven: validate the batch, then fire the VLM goal asynchronously and
  // return immediately. The goal response / result are serviced by the executor
  // in main() and handled in doneCb. NEVER spin here (the node is already in an
  // executor — nesting a spin_some would abort with "already added to executor").
  void viewsCb(const explorer_msgs::msg::FrontierViews::SharedPtr fv)
  {
    RCLCPP_INFO(get_logger(), "Received frontier views");
    tryProcessViews(fv);
  }

  void tryProcessViews(const explorer_msgs::msg::FrontierViews::SharedPtr fv)
  {
    if (require_frontiers_ && fv->frontiers.empty()) {
      RCLCPP_DEBUG(get_logger(), "FrontierViews received but no frontiers; skipping.");
      return;
    }
    if (fv->frontiers.size() <= 1) {
      RCLCPP_DEBUG(
        get_logger(), "Single-frontier batch; explore_node auto-selects without VLM.");
      return;
    }
    if (fv->images.size() != fv->frontiers.size()) {
      RCLCPP_WARN(get_logger(), "FrontierViews image/frontier size mismatch");
      return;
    }

    sensor_msgs::msg::CompressedImage::SharedPtr map;
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      map = latest_map_;
    }
    if (!map) {
      {
        std::lock_guard<std::mutex> lk(map_mtx_);
        pending_views_ = fv;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No map image yet; queued batch (will retry when map arrives).");
      return;
    }

    if (inflight_) {
      RCLCPP_DEBUG(get_logger(), "Previous VLM goal still in flight; skipping.");
      return;
    }

    if (dedupe_) {
      const std::string sig = signature(*fv, *map);
      if (!last_sig_.empty() && sig == last_sig_) {
        RCLCPP_DEBUG(get_logger(), "Duplicate FrontierViews batch; skipping.");
        return;
      }
      last_sig_ = sig;
    }

    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      pending_views_.reset();
    }

    FrontierViewsProcess::Goal goal;
    goal.images = fv->images;
    goal.map = *map;
    goal.frontiers = fv->frontiers;

    inflight_ = true;
    goal_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_s_));
    RCLCPP_INFO(get_logger(), "Querying VLM");

    auto send_options = rclcpp_action::Client<FrontierViewsProcess>::SendGoalOptions();
    send_options.result_callback =
      std::bind(&FrontierVlmClientNode::doneCb, this, std::placeholders::_1);
    send_options.feedback_callback =
      std::bind(&FrontierVlmClientNode::fbCb, this, std::placeholders::_1, std::placeholders::_2);

    vlm_client_->async_send_goal(goal, send_options);
  }

  void checkTimeout()
  {
    if (inflight_ && std::chrono::steady_clock::now() > goal_deadline_) {
      RCLCPP_WARN(get_logger(), "VLM goal timed out after %.1fs; canceling.", timeout_s_);
      vlm_client_->async_cancel_all_goals();
      inflight_ = false;
    }
  }

  void fbCb(
    rclcpp_action::ClientGoalHandle<FrontierViewsProcess>::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const FrontierViewsProcess::Feedback> fb)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "VLM status: %d", fb->status);
  }

  void doneCb(const rclcpp_action::ClientGoalHandle<FrontierViewsProcess>::WrappedResult & result)
  {
    inflight_ = false;
    RCLCPP_INFO(
      get_logger(), "VLM finished: code=%d",
      static_cast<int>(result.code));

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
      RCLCPP_WARN(get_logger(), "No successful VLM result.");
      return;
    }

    std_msgs::msg::Int8 out;
    out.data = result.result->frontier;
    RCLCPP_INFO(get_logger(), "Publishing chosen frontier index %d", out.data);
    chosen_pub_->publish(out);
  }

  static std::string signature(
    const explorer_msgs::msg::FrontierViews & fv,
    const sensor_msgs::msg::CompressedImage & map)
  {
    std::ostringstream oss;
    oss << fv.header.stamp.sec << "." << fv.header.stamp.nanosec
        << "|" << fv.images.size() << "|" << fv.frontiers.size()
        << "|" << map.header.stamp.sec << "." << map.header.stamp.nanosec
        << "|" << map.data.size() << "|" << map.format;
    return oss.str();
  }

  bool require_frontiers_{true};
  bool dedupe_{true};
  double timeout_s_{30.0};
  bool inflight_{false};
  std::chrono::steady_clock::time_point goal_deadline_;
  std::string last_sig_;

  std::mutex map_mtx_;
  sensor_msgs::msg::CompressedImage::SharedPtr latest_map_;
  explorer_msgs::msg::FrontierViews::SharedPtr pending_views_;

  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr chosen_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr map_sub_;
  rclcpp::Subscription<explorer_msgs::msg::FrontierViews>::SharedPtr views_sub_;
  rclcpp_action::Client<FrontierViewsProcess>::SharedPtr vlm_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FrontierVlmClientNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("frontier_vlm_client"), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
