#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

// ======================================================================
// PX4OdomConverterNode
// /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, NED)
//   → /nav/odom (nav_msgs/Odometry, ENU)
//
// 좌표계 변환 (NED → ENU):
//   x_enu =  y_ned
//   y_enu =  x_ned
//   z_enu = -z_ned
// ======================================================================
class PX4OdomConverterNode : public rclcpp::Node
{
public:
  PX4OdomConverterNode() : Node("px4_odom_converter")
  {
    px4_odom_topic_ = this->declare_parameter<std::string>(
      "px4_odom_topic", "/fmu/out/vehicle_odometry");
    nav_odom_topic_ = this->declare_parameter<std::string>(
      "nav_odom_topic", "/nav/odom");
    output_frame_id_ = this->declare_parameter<std::string>(
      "output_frame_id", "world");
    output_child_frame_id_ = this->declare_parameter<std::string>(
      "output_child_frame_id", "base_link");
    use_px4_timestamp_ = this->declare_parameter<bool>(
      "use_px4_timestamp", false);

    // PX4 토픽은 BEST_EFFORT QoS 필수
    px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4_odom_topic_,
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        convertAndPublish(msg);
      });

    nav_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      nav_odom_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "PX4 Odom Converter started: %s -> %s (NED -> ENU, frame=%s child=%s)",
      px4_odom_topic_.c_str(), nav_odom_topic_.c_str(),
      output_frame_id_.c_str(), output_child_frame_id_.c_str());
  }

private:
  void convertAndPublish(const px4_msgs::msg::VehicleOdometry::SharedPtr px4_odom)
  {
    nav_msgs::msg::Odometry odom{};

    if (use_px4_timestamp_ && px4_odom->timestamp > 0) {
      odom.header.stamp = rclcpp::Time(
        static_cast<int64_t>(px4_odom->timestamp) * 1000, RCL_ROS_TIME);
    } else {
      odom.header.stamp = this->get_clock()->now();
    }
    odom.header.frame_id = output_frame_id_;
    odom.child_frame_id  = output_child_frame_id_;

    // NED → ENU 위치 변환
    odom.pose.pose.position.x =  px4_odom->position[1];  // ENU x = NED y
    odom.pose.pose.position.y =  px4_odom->position[0];  // ENU y = NED x
    odom.pose.pose.position.z = -px4_odom->position[2];  // ENU z = -NED z

    // NED → ENU quaternion 변환
    // PX4 q 배열: [w, x, y, z]
    tf2::Quaternion q_ned(
      px4_odom->q[1],  // x
      px4_odom->q[2],  // y
      px4_odom->q[3],  // z
      px4_odom->q[0]   // w
    );
    // NED → ENU 회전: x축 180도 회전
    tf2::Quaternion q_ned_to_enu;
    q_ned_to_enu.setRPY(M_PI, 0.0, M_PI / 2.0);
    tf2::Quaternion q_enu = q_ned_to_enu * q_ned;
    q_enu.normalize();

    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();
    odom.pose.pose.orientation.w = q_enu.w();

    // NED → ENU 속도 변환
    odom.twist.twist.linear.x =  px4_odom->velocity[1];
    odom.twist.twist.linear.y =  px4_odom->velocity[0];
    odom.twist.twist.linear.z = -px4_odom->velocity[2];

    odom.twist.twist.angular.x =  px4_odom->angular_velocity[1];
    odom.twist.twist.angular.y =  px4_odom->angular_velocity[0];
    odom.twist.twist.angular.z = -px4_odom->angular_velocity[2];

    nav_odom_pub_->publish(odom);
  }

  std::string px4_odom_topic_;
  std::string nav_odom_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;
  bool use_px4_timestamp_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            nav_odom_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4OdomConverterNode>());
  rclcpp::shutdown();
  return 0;
}
