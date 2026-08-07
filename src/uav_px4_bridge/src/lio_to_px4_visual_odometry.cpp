#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

class LioToPx4VisualOdometryNode : public rclcpp::Node
{
public:
  LioToPx4VisualOdometryNode() : Node("lio_to_px4_visual_odometry")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lio/odom");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/fmu/in/vehicle_visual_odometry");
    publish_orientation_ = declare_parameter<bool>("publish_orientation", false);
    publish_velocity_ = declare_parameter<bool>("publish_velocity", true);
    position_variance_ = declare_parameter<double>("position_variance", 0.04);
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.04);
    velocity_variance_ = declare_parameter<double>("velocity_variance", 0.09);
    quality_ = declare_parameter<int>("quality", 100);

    px4_ev_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic_, 10);
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        publishPx4VisualOdometry(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "LIO -> PX4 visual odometry: %s -> %s, position/velocity ENU->NED, orientation=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      publish_orientation_ ? "enabled" : "disabled");
  }

private:
  static float nanf()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  void publishPx4VisualOdometry(const nav_msgs::msg::Odometry & odom)
  {
    px4_msgs::msg::VehicleOdometry px4_msg{};

    const uint64_t now_us = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
    px4_msg.timestamp = now_us;
    px4_msg.timestamp_sample = now_us;

    px4_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    px4_msg.velocity_frame = publish_velocity_
      ? px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED
      : px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

    // ROS/LIO local frame is treated as ENU. PX4 EKF2 expects EV position in NED.
    px4_msg.position[0] = static_cast<float>(odom.pose.pose.position.y);
    px4_msg.position[1] = static_cast<float>(odom.pose.pose.position.x);
    px4_msg.position[2] = static_cast<float>(-odom.pose.pose.position.z);

    if (publish_orientation_) {
      tf2::Quaternion q_enu(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);

      tf2::Quaternion q_ned_to_enu;
      q_ned_to_enu.setRPY(M_PI, 0.0, M_PI / 2.0);
      tf2::Quaternion q_ned = q_ned_to_enu.inverse() * q_enu;
      q_ned.normalize();

      px4_msg.q[0] = static_cast<float>(q_ned.w());
      px4_msg.q[1] = static_cast<float>(q_ned.x());
      px4_msg.q[2] = static_cast<float>(q_ned.y());
      px4_msg.q[3] = static_cast<float>(q_ned.z());
    } else {
      px4_msg.q[0] = nanf();
      px4_msg.q[1] = nanf();
      px4_msg.q[2] = nanf();
      px4_msg.q[3] = nanf();
    }

    if (publish_velocity_) {
      px4_msg.velocity[0] = static_cast<float>(odom.twist.twist.linear.y);
      px4_msg.velocity[1] = static_cast<float>(odom.twist.twist.linear.x);
      px4_msg.velocity[2] = static_cast<float>(-odom.twist.twist.linear.z);
    } else {
      px4_msg.velocity[0] = nanf();
      px4_msg.velocity[1] = nanf();
      px4_msg.velocity[2] = nanf();
    }

    px4_msg.angular_velocity[0] = nanf();
    px4_msg.angular_velocity[1] = nanf();
    px4_msg.angular_velocity[2] = nanf();

    for (int i = 0; i < 3; ++i) {
      px4_msg.position_variance[i] = static_cast<float>(position_variance_);
      px4_msg.orientation_variance[i] = publish_orientation_
        ? static_cast<float>(orientation_variance_)
        : nanf();
      px4_msg.velocity_variance[i] = publish_velocity_
        ? static_cast<float>(velocity_variance_)
        : nanf();
    }

    px4_msg.reset_counter = 0;
    px4_msg.quality = static_cast<int8_t>(std::clamp(quality_, 0, 100));

    px4_ev_pub_->publish(px4_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  bool publish_orientation_{false};
  bool publish_velocity_{true};
  double position_variance_{0.04};
  double orientation_variance_{0.04};
  double velocity_variance_{0.09};
  int quality_{100};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_ev_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioToPx4VisualOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
