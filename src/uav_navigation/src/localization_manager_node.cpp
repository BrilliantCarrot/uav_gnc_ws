#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LocalizationManagerNode : public rclcpp::Node
{
public:
  LocalizationManagerNode() : Node("localization_manager_node")
  {
    lio_odom_topic_ = declare_parameter<std::string>("lio_odom_topic", "/lio/odom");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/nav/odom");
    expected_frame_id_ = declare_parameter<std::string>("expected_frame_id", "world");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "world");
    output_child_frame_id_ = declare_parameter<std::string>("output_child_frame_id", "base_link");

    max_source_age_sec_ = declare_parameter<double>("max_source_age_sec", 0.25);
    max_pose_jump_m_ = declare_parameter<double>("max_pose_jump_m", 3.0);
    max_yaw_jump_rad_ = declare_parameter<double>("max_yaw_jump_rad", 1.2);
    max_position_covariance_ = declare_parameter<double>("max_position_covariance", 4.0);
    require_expected_frame_ = declare_parameter<bool>("require_expected_frame", false);
    publish_stale_odom_ = declare_parameter<bool>("publish_stale_odom", false);

    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_odom_topic_, 20,
      std::bind(&LocalizationManagerNode::lioCallback, this, std::placeholders::_1));
    nav_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LocalizationManagerNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "[Localization Manager] lio=%s output=%s frame=%s max_age=%.2fs max_jump=%.2fm",
      lio_odom_topic_.c_str(), output_odom_topic_.c_str(), output_frame_id_.c_str(),
      max_source_age_sec_, max_pose_jump_m_);
  }

private:
  struct PoseLite
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
  };

  static double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static bool finiteOdom(const nav_msgs::msg::Odometry &msg)
  {
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const auto &v = msg.twist.twist.linear;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
           std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }

  static double maxPositionCovariance(const nav_msgs::msg::Odometry &msg)
  {
    return std::max({msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14]});
  }

  PoseLite poseLite(const nav_msgs::msg::Odometry &msg) const
  {
    PoseLite p;
    p.x = msg.pose.pose.position.x;
    p.y = msg.pose.pose.position.y;
    p.z = msg.pose.pose.position.z;
    p.yaw = yawFromQuat(msg.pose.pose.orientation);
    return p;
  }

  bool passesChecks(const nav_msgs::msg::Odometry &msg)
  {
    if (!finiteOdom(msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[Localization Manager] rejected LIO odom: NaN/Inf");
      return false;
    }

    if (require_expected_frame_ && msg.header.frame_id != expected_frame_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[Localization Manager] rejected LIO odom: frame_id=%s expected=%s",
        msg.header.frame_id.c_str(), expected_frame_id_.c_str());
      return false;
    }

    const double max_cov = maxPositionCovariance(msg);
    if (max_position_covariance_ > 0.0 && std::isfinite(max_cov) && max_cov > max_position_covariance_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[Localization Manager] rejected LIO odom: position covariance %.3f > %.3f",
        max_cov, max_position_covariance_);
      return false;
    }

    const PoseLite cur = poseLite(msg);
    if (last_valid_received_ && max_pose_jump_m_ > 0.0) {
      const double dx = cur.x - last_valid_pose_.x;
      const double dy = cur.y - last_valid_pose_.y;
      const double dz = cur.z - last_valid_pose_.z;
      const double jump = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double yaw_jump = std::abs(normalizeAngle(cur.yaw - last_valid_pose_.yaw));
      if (jump > max_pose_jump_m_ || yaw_jump > max_yaw_jump_rad_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "[Localization Manager] rejected LIO odom: jump=%.3fm yaw_jump=%.3frad",
          jump, yaw_jump);
        return false;
      }
    }

    return true;
  }

  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!passesChecks(*msg)) {
      return;
    }

    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = output_frame_id_;
    out.child_frame_id = output_child_frame_id_;
    latest_valid_odom_ = out;
    last_valid_pose_ = poseLite(out);
    last_valid_stamp_ = msg->header.stamp;
    last_receive_time_ = now();
    last_valid_received_ = true;

    nav_pub_->publish(out);
  }

  void onTimer()
  {
    if (!last_valid_received_) return;

    const double age = (now() - last_receive_time_).seconds();
    if (age > max_source_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[Localization Manager] LIO timeout: age=%.3fs > %.3fs",
        age, max_source_age_sec_);
      if (publish_stale_odom_) {
        nav_msgs::msg::Odometry stale = latest_valid_odom_;
        stale.header.stamp = now();
        nav_pub_->publish(stale);
      }
    }
  }

  std::string lio_odom_topic_;
  std::string output_odom_topic_;
  std::string expected_frame_id_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  double max_source_age_sec_{0.25};
  double max_pose_jump_m_{3.0};
  double max_yaw_jump_rad_{1.2};
  double max_position_covariance_{4.0};
  bool require_expected_frame_{false};
  bool publish_stale_odom_{false};

  nav_msgs::msg::Odometry latest_valid_odom_;
  PoseLite last_valid_pose_;
  rclcpp::Time last_valid_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_receive_time_{0, 0, RCL_ROS_TIME};
  bool last_valid_received_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationManagerNode>());
  rclcpp::shutdown();
  return 0;
}
