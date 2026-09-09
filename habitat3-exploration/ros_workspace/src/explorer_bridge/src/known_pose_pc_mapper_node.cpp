#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "explorer_bridge/occupancy_map.hpp"
#include "explorer_bridge/pc_occupancy.hpp"
#include "explorer_bridge/pose_gate.hpp"

namespace explorer_bridge
{
namespace
{

int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

double yawFromOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class KnownPosePcMapperNode : public rclcpp::Node
{
public:
  KnownPosePcMapperNode()
  : Node("known_pose_pc_mapper")
  {
    declare_parameter("depth_topic", "/depth_data");
    declare_parameter("camera_info_topic", "/depth/camera_info");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("grid_topic", "/grid_map");
    declare_parameter("map_frame", "map");
    declare_parameter("resolution", 0.05);
    declare_parameter("initial_size_m", 20.0);
    declare_parameter("publish_hz", 5.0);
    declare_parameter("odom_cache_size", 2048);
    declare_parameter("max_stamp_skew_sec", 0.0);
    declare_parameter("pending_depth_limit", 128);
    declare_parameter("obstacle_inflation_m", 0.05);
    declare_parameter("range_min", 0.1);
    declare_parameter("range_max", 10.0);
    declare_parameter("sensor_far", 50.0);
    declare_parameter("sat_eps", 0.5);
    declare_parameter("camera_z", 0.1);
    declare_parameter("wall_height_min_m", kDefaultWallHeightMin_m);
    declare_parameter("wall_height_max_m", kDefaultWallHeightMax_m);
    declare_parameter("subsample", 8);
    declare_parameter("min_pose_change_m", 0.25);
    declare_parameter("min_yaw_change_rad", 0.17);

    const auto depth_topic = get_parameter("depth_topic").as_string();
    const auto info_topic = get_parameter("camera_info_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto grid_topic = get_parameter("grid_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    const double resolution = get_parameter("resolution").as_double();
    const double initial_size = get_parameter("initial_size_m").as_double();
    const double publish_hz = std::max(0.2, get_parameter("publish_hz").as_double());
    cache_size_ = std::max(8, static_cast<int>(get_parameter("odom_cache_size").as_int()));
    const double skew_sec = std::max(0.0, get_parameter("max_stamp_skew_sec").as_double());
    max_skew_ns_ = static_cast<int64_t>(skew_sec * 1e9);
    pending_limit_ = std::max(1, static_cast<int>(get_parameter("pending_depth_limit").as_int()));
    const double inflation_m = std::max(0.0, get_parameter("obstacle_inflation_m").as_double());
    resolution_ = resolution;
    inflate_cells_ = inflationRadiusCells(resolution_, inflation_m);

    params_.range_min = get_parameter("range_min").as_double();
    params_.range_max = get_parameter("range_max").as_double();
    params_.sensor_far = get_parameter("sensor_far").as_double();
    params_.sat_eps = get_parameter("sat_eps").as_double();
    params_.camera_z = get_parameter("camera_z").as_double();
    params_.wall_height_min = get_parameter("wall_height_min_m").as_double();
    params_.wall_height_max = get_parameter("wall_height_max_m").as_double();
    params_.subsample = std::max(1, static_cast<int>(get_parameter("subsample").as_int()));
    min_pose_change_m_ = get_parameter("min_pose_change_m").as_double();
    min_yaw_change_rad_ = get_parameter("min_yaw_change_rad").as_double();

    grid_ = std::make_unique<OccupancyMap>(resolution, initial_size);

    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.transient_local();
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic, map_qos);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(msg);});
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {onInfo(msg);});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {onDepth(msg);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz),
      [this]() {publishGrid();});

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "obstacle_inflation_m") {
            const double m = std::max(0.0, p.as_double());
            inflate_cells_ = inflationRadiusCells(resolution_, m);
            RCLCPP_WARN(
              get_logger(),
              "obstacle_inflation_m set to %.3f (%d cells)", m, inflate_cells_);
          }
        }
        return result;
      });

    RCLCPP_INFO(
      get_logger(),
      "Known-pose PC mapper (C++): %s + %s -> %s (subsample=%d, pose_gate xy=%.2f yaw=%.2f)",
      depth_topic.c_str(), odom_topic.c_str(), grid_topic.c_str(),
      params_.subsample, min_pose_change_m_, min_yaw_change_rad_);
  }

private:
  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    params_.K.fx = msg->k[0];
    params_.K.fy = msg->k[4];
    params_.K.cx = msg->k[2];
    params_.K.cy = msg->k[5];
    have_info_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const int64_t stamp_ns = stampToNs(msg->header.stamp);
    Pose2D pose{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      yawFromOdom(*msg)};
    odom_by_stamp_[stamp_ns] = pose;
    while (static_cast<int>(odom_by_stamp_.size()) > cache_size_) {
      odom_by_stamp_.erase(odom_by_stamp_.begin());
    }
    drainPending();
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!have_info_ || msg->encoding != "32FC1") {
      return;
    }
    if (!tryIntegrate(msg)) {
      pending_depth_.push_back(msg);
      while (static_cast<int>(pending_depth_.size()) > pending_limit_) {
        pending_depth_.pop_front();
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No /odom with exact depth stamp (cache=%zu; deferred)",
        odom_by_stamp_.size());
    }
  }

  void drainPending()
  {
    if (pending_depth_.empty()) {
      return;
    }
    std::deque<sensor_msgs::msg::Image::SharedPtr> remaining;
    while (!pending_depth_.empty()) {
      auto img = pending_depth_.front();
      pending_depth_.pop_front();
      const int64_t stamp_ns = stampToNs(img->header.stamp);
      if (odom_by_stamp_.count(stamp_ns)) {
        tryIntegrate(img);
        continue;
      }
      if (!odom_by_stamp_.empty()) {
        const int64_t oldest = odom_by_stamp_.begin()->first;
        if (stamp_ns < oldest) {
          continue;
        }
      }
      remaining.push_back(img);
    }
    pending_depth_ = std::move(remaining);
  }

  std::optional<Pose2D> lookupPose(int64_t stamp_ns) const
  {
    const auto it = odom_by_stamp_.find(stamp_ns);
    if (it != odom_by_stamp_.end()) {
      return it->second;
    }
    if (max_skew_ns_ <= 0) {
      return std::nullopt;
    }
    std::optional<Pose2D> best;
    int64_t best_skew = std::numeric_limits<int64_t>::max();
    for (const auto & [ts, pose] : odom_by_stamp_) {
      const int64_t skew = std::llabs(ts - stamp_ns);
      if (skew < best_skew) {
        best_skew = skew;
        best = pose;
      }
    }
    if (!best || best_skew > max_skew_ns_) {
      return std::nullopt;
    }
    return best;
  }

  bool tryIntegrate(const sensor_msgs::msg::Image::SharedPtr & msg)
  {
    const int64_t stamp_ns = stampToNs(msg->header.stamp);
    auto pose = lookupPose(stamp_ns);
    if (!pose.has_value()) {
      return false;
    }

    if (!shouldIntegratePose(*pose, last_integrate_pose_, min_pose_change_m_, min_yaw_change_rad_)) {
      return true;
    }

    if (msg->step < msg->width * sizeof(float) ||
      msg->data.size() < static_cast<size_t>(msg->height) * msg->step)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Malformed depth image");
      return true;
    }

    std::vector<float> depth(static_cast<size_t>(msg->height) * static_cast<size_t>(msg->width));
    for (uint32_t r = 0; r < msg->height; ++r) {
      const auto * row = reinterpret_cast<const float *>(
        msg->data.data() + static_cast<size_t>(r) * msg->step);
      std::memcpy(
        depth.data() + static_cast<size_t>(r) * msg->width,
        row,
        static_cast<size_t>(msg->width) * sizeof(float));
    }

    params_.robot_x = pose->x;
    params_.robot_y = pose->y;
    params_.yaw = pose->yaw;

    integrateDepthFrame(
      *grid_, depth.data(),
      static_cast<int>(msg->height), static_cast<int>(msg->width),
      params_);

    last_integrate_pose_ = *pose;
    return true;
  }

  void publishGrid()
  {
    auto published = inflateOccupied(*grid_, inflate_cells_);

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(grid_->resolution());
    msg.info.width = static_cast<uint32_t>(grid_->width());
    msg.info.height = static_cast<uint32_t>(grid_->height());
    msg.info.origin.position.x = grid_->originX();
    msg.info.origin.position.y = grid_->originY();
    msg.info.origin.orientation.w = 1.0;
    msg.data.assign(published.begin(), published.end());
    pub_->publish(msg);
  }

  std::string map_frame_;
  int cache_size_{2048};
  int64_t max_skew_ns_{0};
  int pending_limit_{128};
  int inflate_cells_{0};
  double resolution_{0.05};
  double min_pose_change_m_{0.25};
  double min_yaw_change_rad_{0.17};
  bool have_info_{false};
  IntegrateDepthParams params_{};
  std::unique_ptr<OccupancyMap> grid_;
  std::optional<Pose2D> last_integrate_pose_;
  std::map<int64_t, Pose2D> odom_by_stamp_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> pending_depth_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace explorer_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<explorer_bridge::KnownPosePcMapperNode>());
  rclcpp::shutdown();
  return 0;
}
