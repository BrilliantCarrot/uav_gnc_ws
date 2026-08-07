#include <algorithm>
#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>

class SimOdomToGazeboPoseNode : public rclcpp::Node
{
public:
  SimOdomToGazeboPoseNode() : Node("sim_odom_to_gazebo_pose_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
    world_name_ = declare_parameter<std::string>("world_name", "gps_denied_outdoor_lio_vio");
    model_name_ = declare_parameter<std::string>("model_name", "f450_lio_vio_sensor_rig");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    z_offset_m_ = declare_parameter<double>("z_offset_m", 0.0);
    request_timeout_ms_ = declare_parameter<int>("request_timeout_ms", 50);
    wait_for_first_odom_ = declare_parameter<bool>("wait_for_first_odom", true);

    service_name_ = "/world/" + world_name_ + "/set_pose";

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SimOdomToGazeboPoseNode::odomCallback, this, std::placeholders::_1));

    const double safe_rate = std::max(publish_rate_hz_, 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_rate)),
      std::bind(&SimOdomToGazeboPoseNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "[Sim->Gazebo Pose Sync] odom=%s service=%s model=%s rate=%.1fHz z_offset=%.2fm",
      odom_topic_.c_str(), service_name_.c_str(), model_name_.c_str(), publish_rate_hz_, z_offset_m_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  static bool finite(const geometry_msgs::msg::Point & p)
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  static bool finite(const geometry_msgs::msg::Quaternion & q)
  {
    return std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w);
  }

  void onTimer()
  {
    if (!has_odom_) {
      if (wait_for_first_odom_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for first odometry message on %s", odom_topic_.c_str());
      }
      return;
    }

    const auto & pose_in = latest_odom_.pose.pose;
    if (!finite(pose_in.position) || !finite(pose_in.orientation)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Skipping non-finite odometry pose.");
      return;
    }

    ignition::msgs::Pose pose_msg;
    pose_msg.set_name(model_name_);
    pose_msg.mutable_position()->set_x(pose_in.position.x);
    pose_msg.mutable_position()->set_y(pose_in.position.y);
    pose_msg.mutable_position()->set_z(pose_in.position.z + z_offset_m_);
    pose_msg.mutable_orientation()->set_x(pose_in.orientation.x);
    pose_msg.mutable_orientation()->set_y(pose_in.orientation.y);
    pose_msg.mutable_orientation()->set_z(pose_in.orientation.z);
    pose_msg.mutable_orientation()->set_w(pose_in.orientation.w);

    ignition::msgs::Boolean reply;
    bool result = false;
    const bool executed = gz_node_.Request(
      service_name_, pose_msg, static_cast<unsigned int>(request_timeout_ms_), reply, result);

    if (!executed || !result || !reply.data()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Gazebo set_pose failed: service=%s executed=%s result=%s reply=%s",
        service_name_.c_str(),
        executed ? "true" : "false",
        result ? "true" : "false",
        reply.data() ? "true" : "false");
      return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "[Sim->Gazebo Pose Sync] synced p=(%.2f, %.2f, %.2f)",
      pose_in.position.x, pose_in.position.y, pose_in.position.z + z_offset_m_);
  }

  std::string odom_topic_;
  std::string world_name_;
  std::string model_name_;
  std::string service_name_;
  double publish_rate_hz_{30.0};
  double z_offset_m_{0.0};
  int request_timeout_ms_{50};
  bool wait_for_first_odom_{true};
  bool has_odom_{false};

  nav_msgs::msg::Odometry latest_odom_;
  ignition::transport::Node gz_node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimOdomToGazeboPoseNode>());
  rclcpp::shutdown();
  return 0;
}
