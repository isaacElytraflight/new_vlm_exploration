#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <explorer_msgs/action/discrete_move.hpp>
#include <explorer_msgs/action/perceive_and_capture.hpp>
#include <explorer_msgs/action/rotate360.hpp>

using DiscreteMove = explorer_msgs::action::DiscreteMove;
using Rotate360 = explorer_msgs::action::Rotate360;
using PerceiveAndCapture = explorer_msgs::action::PerceiveAndCapture;

class ActionsNode : public rclcpp::Node
{
public:
  ActionsNode()
  : Node("actions"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_data");
    rotate_steps_ = declare_parameter<int>("rotate_steps", 36);

    discrete_move_client_ = rclcpp_action::create_client<DiscreteMove>(
      this, "/movement/discrete_move");

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ActionsNode::imageCb, this, std::placeholders::_1));

    rotate_server_ = rclcpp_action::create_server<Rotate360>(
      this, "rotate_360",
      std::bind(&ActionsNode::rotateGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionsNode::rotateCancel, this, std::placeholders::_1),
      std::bind(&ActionsNode::rotateAccepted, this, std::placeholders::_1));

    perceive_server_ = rclcpp_action::create_server<PerceiveAndCapture>(
      this, "perceive_and_capture",
      std::bind(&ActionsNode::perceiveGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionsNode::perceiveCancel, this, std::placeholders::_1),
      std::bind(&ActionsNode::perceiveAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Actions node ready (rotate_360 + perceive_and_capture stub)");
  }

private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_ = msg;
    image_received_ = true;
  }

  bool getRobotYawDeg(double & yaw_deg)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
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
      yaw_deg = yaw * 180.0 / M_PI;
      while (yaw_deg < 0.0) {
        yaw_deg += 360.0;
      }
      while (yaw_deg >= 360.0) {
        yaw_deg -= 360.0;
      }
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  static double angularDifference(double a, double b)
  {
    double diff = a - b;
    if (diff > 180.0) {
      diff -= 360.0;
    } else if (diff < -180.0) {
      diff += 360.0;
    }
    return std::abs(diff);
  }

  sensor_msgs::msg::CompressedImage compressImage(const sensor_msgs::msg::Image & img)
  {
    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = img.header;
    compressed.format = "jpeg";

    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
      cv::Mat bgr;
      cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 95};
      std::vector<uchar> data;
      cv::imencode(".jpg", bgr, data, params);
      compressed.data = data;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      compressed.format = "";
    }
    return compressed;
  }

  bool sendDiscreteMove(uint8_t direction, uint32_t steps)
  {
    if (!discrete_move_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "DiscreteMove action server unavailable");
      return false;
    }

    DiscreteMove::Goal goal;
    goal.direction = direction;
    goal.steps = steps;

    auto future = discrete_move_client_->async_send_goal(goal);
    if (future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Timed out sending DiscreteMove goal");
      return false;
    }

    const auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "DiscreteMove goal rejected");
      return false;
    }

    auto result_future = discrete_move_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "DiscreteMove timed out");
      return false;
    }

    const auto wrapped = result_future.get();
    return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success;
  }

  rclcpp_action::GoalResponse rotateGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const Rotate360::Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse rotateCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotate360>> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void rotateAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotate360>> goal_handle)
  {
    std::thread{[this, goal_handle]() {executeRotate(goal_handle);}}.detach();
  }

  void executeRotate(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Rotate360>> goal_handle)
  {
    auto feedback = std::make_shared<Rotate360::Feedback>();
    auto result = std::make_shared<Rotate360::Result>();

    feedback->status = "Performing 360-degree rotation via DiscreteMove";
    goal_handle->publish_feedback(feedback);

    double starting_yaw = 0.0;
    if (!getRobotYawDeg(starting_yaw)) {
      result->success = false;
      result->message = "Failed to get starting yaw";
      goal_handle->abort(result);
      return;
    }

    result->cached_images.clear();
    result->cached_orientations.clear();

    double last_capture_yaw = starting_yaw;
    const uint32_t steps = static_cast<uint32_t>(rotate_steps_);

    for (uint32_t i = 0; i < steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Action was preempted";
        goal_handle->canceled(result);
        return;
      }

      double yaw = 0.0;
      if (getRobotYawDeg(yaw)) {
        sensor_msgs::msg::Image::SharedPtr image;
        {
          std::lock_guard<std::mutex> lock(image_mutex_);
          if (image_received_ && latest_image_) {
            image = latest_image_;
          }
        }
        const double diff = angularDifference(yaw, last_capture_yaw);
        if (image && (result->cached_images.empty() || diff >= 6.0)) {
          result->cached_images.push_back(compressImage(*image));
          result->cached_orientations.push_back(yaw);
          last_capture_yaw = yaw;
          RCLCPP_INFO(
            get_logger(), "Captured image %zu at yaw %.1f",
            result->cached_images.size(), yaw);
        }
      }

      if (!sendDiscreteMove(DiscreteMove::Goal::TURN_LEFT, 1)) {
        result->success = false;
        result->message = "DiscreteMove turn_left failed at step " + std::to_string(i);
        goal_handle->abort(result);
        return;
      }
    }

    result->success = true;
    result->message = "360-degree scan completed. Captured " +
      std::to_string(result->cached_images.size()) + " images.";
    goal_handle->succeed(result);
  }

  rclcpp_action::GoalResponse perceiveGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const PerceiveAndCapture::Goal> /*goal*/)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse perceiveCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PerceiveAndCapture>> /*goal_handle*/)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void perceiveAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PerceiveAndCapture>> goal_handle)
  {
    auto result = std::make_shared<PerceiveAndCapture::Result>();
    result->success = false;
    result->message = "perceive_and_capture is a stub in explorer_mission";
    goal_handle->abort(result);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string image_topic_;
  int rotate_steps_{36};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex image_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  bool image_received_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp_action::Client<DiscreteMove>::SharedPtr discrete_move_client_;
  rclcpp_action::Server<Rotate360>::SharedPtr rotate_server_;
  rclcpp_action::Server<PerceiveAndCapture>::SharedPtr perceive_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionsNode>());
  rclcpp::shutdown();
  return 0;
}
