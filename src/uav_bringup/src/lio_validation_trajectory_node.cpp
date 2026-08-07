#include <algorithm>
#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{
constexpr double kGravity = 9.80665;

double smoothStep(double x)
{
  x = std::clamp(x, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Quaternion yawToQuat(double yaw)
{
  const double half = 0.5 * yaw;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

double wrapPi(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void worldToBodyYawOnly(double yaw, double wx, double wy, double wz, double & bx, double & by, double & bz)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  bx = c * wx + s * wy;
  by = -s * wx + c * wy;
  bz = wz;
}
}  // namespace

class LioValidationTrajectoryNode : public rclcpp::Node
{
public:
  LioValidationTrajectoryNode() : Node("lio_validation_trajectory_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lio_validation/odom_truth");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/lio/imu");
    trajectory_mode_ = declare_parameter<std::string>("trajectory_mode", "line");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    hover_time_s_ = declare_parameter<double>("hover_time_s", 3.0);
    altitude_m_ = declare_parameter<double>("altitude_m", 1.8);
    line_speed_mps_ = declare_parameter<double>("line_speed_mps", 0.15);
    radius_m_ = declare_parameter<double>("radius_m", 1.5);
    period_s_ = declare_parameter<double>("period_s", 20.0);
    yaw_follow_velocity_ = declare_parameter<bool>("yaw_follow_velocity", true);
    max_yaw_rate_radps_ = declare_parameter<double>("max_yaw_rate_radps", 0.25);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 50);

    start_time_ = now();
    const double safe_rate = std::max(publish_rate_hz_, 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_rate)),
      std::bind(&LioValidationTrajectoryNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "[LIO Validation Trajectory] odom=%s imu=%s mode=%s rate=%.1fHz hover=%.1fs line_speed=%.2fm/s radius=%.2fm period=%.1fs altitude=%.2fm",
      odom_topic_.c_str(), imu_topic_.c_str(), trajectory_mode_.c_str(), publish_rate_hz_,
      hover_time_s_, line_speed_mps_, radius_m_, period_s_, altitude_m_);
  }

private:
  void onTimer()
  {
    const rclcpp::Time stamp = now();
    const double t = std::max(0.0, (stamp - start_time_).seconds());
    const double tau = std::max(0.0, t - hover_time_s_);
    double x = 0.0;
    double y = 0.0;
    const double z = altitude_m_;
    double vx = 0.0;
    double vy = 0.0;
    const double vz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    const double az = 0.0;

    if (trajectory_mode_ == "figure8") {
      const double omega = 2.0 * M_PI / std::max(period_s_, 1.0);
      const double ramp = smoothStep(tau / 3.0);
      x = ramp * radius_m_ * std::sin(omega * tau);
      y = ramp * 0.5 * radius_m_ * std::sin(2.0 * omega * tau);
      vx = ramp * radius_m_ * omega * std::cos(omega * tau);
      vy = ramp * radius_m_ * omega * std::cos(2.0 * omega * tau);
      ax = ramp * -radius_m_ * omega * omega * std::sin(omega * tau);
      ay = ramp * -2.0 * radius_m_ * omega * omega * std::sin(2.0 * omega * tau);
    } else {
      const double accel_time = 2.0;
      const double accel = line_speed_mps_ / accel_time;
      if (tau < accel_time) {
        x = 0.5 * accel * tau * tau;
        vx = accel * tau;
        ax = accel;
      } else {
        x = 0.5 * line_speed_mps_ * accel_time + line_speed_mps_ * (tau - accel_time);
        vx = line_speed_mps_;
        ax = 0.0;
      }
      y = 0.0;
      vy = 0.0;
      ay = 0.0;
    }

    const double dt = has_prev_stamp_ ? std::max(1e-4, (stamp - prev_stamp_).seconds()) :
      (1.0 / std::max(publish_rate_hz_, 1.0));
    prev_stamp_ = stamp;
    has_prev_stamp_ = true;

    double yaw_rate = 0.0;
    if (yaw_follow_velocity_ && std::hypot(vx, vy) > 1e-3) {
      const double desired_yaw = std::atan2(vy, vx);
      const double yaw_error = wrapPi(desired_yaw - yaw_state_rad_);
      yaw_rate = std::clamp(
        yaw_error / dt,
        -std::abs(max_yaw_rate_radps_),
        std::abs(max_yaw_rate_radps_));
      yaw_state_rad_ = wrapPi(yaw_state_rad_ + yaw_rate * dt);
    }

    const Quaternion q = yawToQuat(yaw_state_rad_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation.w = q.w;
    odom.pose.pose.orientation.x = q.x;
    odom.pose.pose.orientation.y = q.y;
    odom.pose.pose.orientation.z = q.z;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = vz;
    odom.twist.twist.angular.z = yaw_rate;
    odom_pub_->publish(odom);

    double fb_x = 0.0;
    double fb_y = 0.0;
    double fb_z = 0.0;
    worldToBodyYawOnly(yaw_state_rad_, ax, ay, az + kGravity, fb_x, fb_y, fb_z);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_id_;
    imu.orientation.w = q.w;
    imu.orientation.x = q.x;
    imu.orientation.y = q.y;
    imu.orientation.z = q.z;
    imu.angular_velocity.z = yaw_rate;
    imu.linear_acceleration.x = fb_x;
    imu.linear_acceleration.y = fb_y;
    imu.linear_acceleration.z = fb_z;
    imu_pub_->publish(imu);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[LIO Validation Trajectory] p=(%.2f, %.2f, %.2f) v=(%.2f, %.2f, %.2f)",
      x, y, z, vx, vy, vz);
  }

  std::string odom_topic_;
  std::string imu_topic_;
  std::string trajectory_mode_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string imu_frame_id_;
  double publish_rate_hz_{100.0};
  double hover_time_s_{3.0};
  double altitude_m_{1.8};
  double line_speed_mps_{0.15};
  double radius_m_{1.5};
  double period_s_{20.0};
  bool yaw_follow_velocity_{true};
  double max_yaw_rate_radps_{0.25};
  double yaw_state_rad_{0.0};
  bool has_prev_stamp_{false};
  rclcpp::Time prev_stamp_;

  rclcpp::Time start_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioValidationTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
