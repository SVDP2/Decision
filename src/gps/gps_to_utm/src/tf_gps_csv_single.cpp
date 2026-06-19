#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

class TfGpsCsvSingleNode : public rclcpp::Node
{
public:
  TfGpsCsvSingleNode()
  : Node("tf_gps_csv_single_node")
  {
    csv_file_path_ = this->declare_parameter<std::string>("csv_file_path", "");
    vehicle_ref_topic_ =
      this->declare_parameter<std::string>("vehicle_ref_topic", "/vehicle_ref_utm");
    heading_topic_ = this->declare_parameter<std::string>("heading_topic", "/vehicle_heading_rad");
    heading_valid_topic_ =
      this->declare_parameter<std::string>("heading_valid_topic", "/vehicle_heading_valid");
    path_topic_ = this->declare_parameter<std::string>("path_topic", "/csv_path");
    origin_topic_ =
      this->declare_parameter<std::string>("origin_topic", "/leader/map_origin_utm");
    csv_frame_id_ = this->declare_parameter<std::string>("csv_frame_id", "csv");
    vehicle_frame_id_ =
      this->declare_parameter<std::string>("vehicle_frame_id", "vehicle_ref");
    path_publish_rate_hz_ = this->declare_parameter<double>("path_publish_rate_hz", 10.0);
    publish_tf_only_when_heading_valid_ =
      this->declare_parameter<bool>("publish_tf_only_when_heading_valid", true);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, path_qos);
    origin_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(origin_topic_, path_qos);

    vehicle_ref_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      vehicle_ref_topic_, 10,
      std::bind(&TfGpsCsvSingleNode::vehicle_ref_callback, this, std::placeholders::_1));
    heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      heading_topic_, 10,
      std::bind(&TfGpsCsvSingleNode::heading_callback, this, std::placeholders::_1));
    heading_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      heading_valid_topic_, 10,
      std::bind(&TfGpsCsvSingleNode::heading_valid_callback, this, std::placeholders::_1));

    load_csv_path();

    const auto period = std::chrono::duration<double>(1.0 / std::max(path_publish_rate_hz_, 1e-3));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TfGpsCsvSingleNode::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "tf_gps_csv_single_node started: csv_frame=%s vehicle_frame=%s path_points=%zu",
      csv_frame_id_.c_str(), vehicle_frame_id_.c_str(), path_msg_.poses.size());
  }

private:
  void load_csv_path()
  {
    if (csv_file_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "csv_file_path is empty.");
      return;
    }

    std::ifstream file(csv_file_path_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open csv_file_path: %s", csv_file_path_.c_str());
      return;
    }

    std::string line;
    path_msg_.header.frame_id = csv_frame_id_;
    path_msg_.poses.clear();

    size_t line_index = 0;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token_x;
      std::string token_y;
      if (!std::getline(ss, token_x, ',') || !std::getline(ss, token_y, ',')) {
        ++line_index;
        continue;
      }

      try {
        const double x = std::stod(token_x);
        const double y = std::stod(token_y);

        if (!origin_x_.has_value()) {
          origin_x_ = x;
          origin_y_ = y;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = csv_frame_id_;
        pose.pose.position.x = x - origin_x_.value();
        pose.pose.position.y = y - origin_y_.value();
        pose.pose.orientation.w = 1.0;
        path_msg_.poses.push_back(pose);
      } catch (const std::exception &) {
        if (line_index > 0) {
          RCLCPP_DEBUG(
            this->get_logger(), "Skipping non-numeric csv row %zu: %s", line_index, line.c_str());
        }
      }

      ++line_index;
    }

    if (!origin_x_.has_value() || path_msg_.poses.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load a valid csv path.");
      return;
    }
  }

  void vehicle_ref_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    last_vehicle_ref_utm_ = *msg;
  }

  void heading_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    last_heading_rad_ = msg->data;
  }

  void heading_valid_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    heading_valid_ = msg->data;
  }

  void timer_callback()
  {
    if (!path_msg_.poses.empty()) {
      path_msg_.header.stamp = this->get_clock()->now();
      for (auto & pose : path_msg_.poses) {
        pose.header = path_msg_.header;
      }
      path_pub_->publish(path_msg_);
    }
    publish_origin();

    if (!origin_x_.has_value() || !last_vehicle_ref_utm_.has_value()) {
      return;
    }

    if (publish_tf_only_when_heading_valid_ && !heading_valid_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = csv_frame_id_;
    transform.child_frame_id = vehicle_frame_id_;
    transform.transform.translation.x = last_vehicle_ref_utm_->point.x - origin_x_.value();
    transform.transform.translation.y = last_vehicle_ref_utm_->point.y - origin_y_.value();
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, last_heading_rad_.value_or(0.0));
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_origin()
  {
    if (!origin_x_.has_value() || !origin_y_.has_value()) {
      return;
    }
    geometry_msgs::msg::PointStamped origin_msg;
    origin_msg.header.stamp = this->get_clock()->now();
    origin_msg.header.frame_id = "utm";
    origin_msg.point.x = origin_x_.value();
    origin_msg.point.y = origin_y_.value();
    origin_msg.point.z = 0.0;
    origin_pub_->publish(origin_msg);
  }

  std::string csv_file_path_;
  std::string vehicle_ref_topic_;
  std::string heading_topic_;
  std::string heading_valid_topic_;
  std::string path_topic_;
  std::string origin_topic_;
  std::string csv_frame_id_;
  std::string vehicle_frame_id_;
  double path_publish_rate_hz_;
  bool publish_tf_only_when_heading_valid_;
  bool heading_valid_{false};

  std::optional<double> origin_x_;
  std::optional<double> origin_y_;
  std::optional<double> last_heading_rad_;
  std::optional<geometry_msgs::msg::PointStamped> last_vehicle_ref_utm_;
  nav_msgs::msg::Path path_msg_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr vehicle_ref_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_valid_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr origin_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfGpsCsvSingleNode>());
  rclcpp::shutdown();
  return 0;
}
