#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{
double wrap_to_pi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double blend_angle(double previous, double current, double alpha)
{
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  const double x =
    (1.0 - clamped_alpha) * std::cos(previous) + clamped_alpha * std::cos(current);
  const double y =
    (1.0 - clamped_alpha) * std::sin(previous) + clamped_alpha * std::sin(current);
  return std::atan2(y, x);
}

struct CsvPoint
{
  double x;
  double y;
};
}  // namespace

class SingleF9pHeadingNode : public rclcpp::Node
{
public:
  SingleF9pHeadingNode()
  : Node("single_f9p_heading_node"),
    heading_valid_(false),
    using_velocity_heading_(false)
  {
    utm_topic_ = this->declare_parameter<std::string>("utm_topic", "/f9p_utm");
    velocity_topic_ = this->declare_parameter<std::string>("velocity_topic", "/f9p/fix_velocity");
    heading_topic_ = this->declare_parameter<std::string>("heading_topic", "/vehicle_heading_rad");
    heading_valid_topic_ =
      this->declare_parameter<std::string>("heading_valid_topic", "/vehicle_heading_valid");
    speed_topic_ = this->declare_parameter<std::string>("speed_topic", "/vehicle_speed");
    vehicle_ref_topic_ =
      this->declare_parameter<std::string>("vehicle_ref_topic", "/vehicle_ref_utm");
    csv_file_path_ = this->declare_parameter<std::string>("csv_file_path", "");

    use_velocity_heading_ = this->declare_parameter<bool>("use_velocity_heading", true);
    use_path_tangent_fallback_ =
      this->declare_parameter<bool>("use_path_tangent_fallback", true);
    v_heading_enable_ = this->declare_parameter<double>("v_heading_enable", 1.0);
    v_heading_disable_ = this->declare_parameter<double>("v_heading_disable", 0.5);
    velocity_timeout_sec_ = this->declare_parameter<double>("velocity_timeout_sec", 0.5);
    d_min_for_position_heading_ =
      this->declare_parameter<double>("d_min_for_position_heading", 0.4);
    heading_filter_alpha_ = this->declare_parameter<double>("heading_filter_alpha", 0.3);
    antenna_offset_x_ = this->declare_parameter<double>("antenna_offset_x", 0.0);
    antenna_offset_y_ = this->declare_parameter<double>("antenna_offset_y", 0.0);
    path_tangent_step_ = this->declare_parameter<int>("path_tangent_step", 5);

    load_csv_path();

    utm_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      utm_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SingleF9pHeadingNode::utm_callback, this, std::placeholders::_1));
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      velocity_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SingleF9pHeadingNode::velocity_callback, this, std::placeholders::_1));

    heading_pub_ = this->create_publisher<std_msgs::msg::Float64>(heading_topic_, 10);
    heading_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(heading_valid_topic_, 10);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(speed_topic_, 10);
    vehicle_ref_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(vehicle_ref_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "single_f9p_heading_node started: utm=%s velocity=%s csv_points=%zu",
      utm_topic_.c_str(), velocity_topic_.c_str(), csv_points_.size());
  }

private:
  void load_csv_path()
  {
    csv_points_.clear();

    if (csv_file_path_.empty()) {
      RCLCPP_WARN(this->get_logger(), "csv_file_path is empty, path tangent fallback disabled.");
      return;
    }

    std::ifstream file(csv_file_path_);
    if (!file.is_open()) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to open csv_file_path: %s", csv_file_path_.c_str());
      return;
    }

    std::string line;
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
        csv_points_.push_back(CsvPoint{std::stod(token_x), std::stod(token_y)});
      } catch (const std::exception &) {
        if (line_index > 0) {
          RCLCPP_DEBUG(
            this->get_logger(), "Skipping non-numeric csv row %zu: %s", line_index, line.c_str());
        }
      }
      ++line_index;
    }

    if (csv_points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid points loaded from csv_file_path.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Loaded %zu csv points for path tangent fallback.", csv_points_.size());
  }

  void velocity_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    last_velocity_msg_ = *msg;
  }

  void utm_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const rclcpp::Time current_time =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
      ? this->get_clock()->now()
      : rclcpp::Time(msg->header.stamp);

    double current_speed = 0.0;
    std::optional<double> proposed_heading;

    if (const auto velocity_heading = heading_from_velocity(current_time, current_speed)) {
      proposed_heading = velocity_heading;
    }

    if (!proposed_heading) {
      if (const auto position_heading = heading_from_position(*msg, current_time, current_speed)) {
        proposed_heading = position_heading;
      }
    }

    if (!proposed_heading && !last_heading_.has_value() && use_path_tangent_fallback_) {
      if (const auto tangent_heading = heading_from_path_tangent(*msg)) {
        proposed_heading = tangent_heading;
        current_speed = 0.0;
      }
    }

    if (proposed_heading.has_value()) {
      last_heading_ = last_heading_.has_value()
        ? blend_angle(last_heading_.value(), proposed_heading.value(), heading_filter_alpha_)
        : wrap_to_pi(proposed_heading.value());
      heading_valid_ = true;
    } else if (!last_heading_.has_value()) {
      heading_valid_ = false;
    }

    publish_state(*msg, current_speed, current_time);
    last_position_msg_ = *msg;
    last_position_time_ = current_time;
  }

  std::optional<double> heading_from_velocity(const rclcpp::Time & current_time, double & speed_out)
  {
    if (!use_velocity_heading_ || !last_velocity_msg_.has_value()) {
      return std::nullopt;
    }

    const auto & velocity_msg = last_velocity_msg_.value();
    const rclcpp::Time velocity_time =
      velocity_msg.header.stamp.sec == 0 && velocity_msg.header.stamp.nanosec == 0
      ? current_time
      : rclcpp::Time(velocity_msg.header.stamp);

    if ((current_time - velocity_time).seconds() > velocity_timeout_sec_) {
      return std::nullopt;
    }

    const double vx = velocity_msg.twist.twist.linear.x;
    const double vy = velocity_msg.twist.twist.linear.y;
    const double speed = std::hypot(vx, vy);
    speed_out = speed;

    if (speed >= v_heading_enable_) {
      using_velocity_heading_ = true;
    } else if (speed <= v_heading_disable_) {
      using_velocity_heading_ = false;
    }

    if (!using_velocity_heading_ || speed < 1e-3) {
      return std::nullopt;
    }

    return wrap_to_pi(std::atan2(vy, vx));
  }

  std::optional<double> heading_from_position(
    const geometry_msgs::msg::PointStamped & current_msg,
    const rclcpp::Time & current_time,
    double & speed_out)
  {
    if (!last_position_msg_.has_value()) {
      return std::nullopt;
    }

    const double dx = current_msg.point.x - last_position_msg_->point.x;
    const double dy = current_msg.point.y - last_position_msg_->point.y;
    const double distance = std::hypot(dx, dy);
    const double dt = std::max((current_time - last_position_time_).seconds(), 1e-3);

    speed_out = std::max(speed_out, distance / dt);
    if (distance < d_min_for_position_heading_) {
      return std::nullopt;
    }

    return wrap_to_pi(std::atan2(dy, dx));
  }

  std::optional<double> heading_from_path_tangent(const geometry_msgs::msg::PointStamped & current_msg)
  {
    if (csv_points_.size() < 2) {
      return std::nullopt;
    }

    size_t nearest_idx = 0;
    double best_distance_sq = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < csv_points_.size(); ++i) {
      const double dx = csv_points_[i].x - current_msg.point.x;
      const double dy = csv_points_[i].y - current_msg.point.y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        nearest_idx = i;
      }
    }

    const size_t step = static_cast<size_t>(std::max(path_tangent_step_, 1));
    size_t target_idx = std::min(nearest_idx + step, csv_points_.size() - 1);
    if (target_idx == nearest_idx && nearest_idx > 0) {
      target_idx = nearest_idx - 1;
    }
    if (target_idx == nearest_idx) {
      return std::nullopt;
    }

    const double dx = csv_points_[target_idx].x - csv_points_[nearest_idx].x;
    const double dy = csv_points_[target_idx].y - csv_points_[nearest_idx].y;
    if (std::hypot(dx, dy) < 1e-6) {
      return std::nullopt;
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "Using path tangent fallback to initialize heading.");
    return wrap_to_pi(std::atan2(dy, dx));
  }

  void publish_state(
    const geometry_msgs::msg::PointStamped & current_msg, double current_speed,
    const rclcpp::Time & current_time)
  {
    auto speed_msg = std_msgs::msg::Float64();
    speed_msg.data = current_speed;
    speed_pub_->publish(speed_msg);

    auto heading_valid_msg = std_msgs::msg::Bool();
    heading_valid_msg.data = heading_valid_;
    heading_valid_pub_->publish(heading_valid_msg);

    if (!heading_valid_ || !last_heading_.has_value()) {
      return;
    }

    auto heading_msg = std_msgs::msg::Float64();
    heading_msg.data = last_heading_.value();
    heading_pub_->publish(heading_msg);

    geometry_msgs::msg::PointStamped vehicle_ref_msg;
    vehicle_ref_msg.header.stamp = current_time;
    vehicle_ref_msg.header.frame_id = current_msg.header.frame_id.empty()
      ? "utm"
      : current_msg.header.frame_id;

    const double heading = last_heading_.value();
    const double offset_x =
      antenna_offset_x_ * std::cos(heading) - antenna_offset_y_ * std::sin(heading);
    const double offset_y =
      antenna_offset_x_ * std::sin(heading) + antenna_offset_y_ * std::cos(heading);

    vehicle_ref_msg.point.x = current_msg.point.x - offset_x;
    vehicle_ref_msg.point.y = current_msg.point.y - offset_y;
    vehicle_ref_msg.point.z = current_msg.point.z;
    vehicle_ref_pub_->publish(vehicle_ref_msg);
  }

  std::string utm_topic_;
  std::string velocity_topic_;
  std::string heading_topic_;
  std::string heading_valid_topic_;
  std::string speed_topic_;
  std::string vehicle_ref_topic_;
  std::string csv_file_path_;

  bool use_velocity_heading_;
  bool use_path_tangent_fallback_;
  bool heading_valid_;
  bool using_velocity_heading_;
  double v_heading_enable_;
  double v_heading_disable_;
  double velocity_timeout_sec_;
  double d_min_for_position_heading_;
  double heading_filter_alpha_;
  double antenna_offset_x_;
  double antenna_offset_y_;
  int path_tangent_step_;

  std::vector<CsvPoint> csv_points_;

  std::optional<geometry_msgs::msg::PointStamped> last_position_msg_;
  rclcpp::Time last_position_time_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> last_velocity_msg_;
  std::optional<double> last_heading_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr utm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heading_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr vehicle_ref_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleF9pHeadingNode>());
  rclcpp::shutdown();
  return 0;
}
