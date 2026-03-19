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
    // PX4 토픽은 BEST_EFFORT QoS 필수
    px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        convertAndPublish(msg);
      });

    nav_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/nav/odom", 10);

    RCLCPP_INFO(this->get_logger(), "PX4 Odom Converter started (NED → ENU)");
  }

private:
void convertAndPublish(const px4_msgs::msg::VehicleOdometry::SharedPtr px4_odom)
{
    nav_msgs::msg::Odometry odom{};

    odom.header.stamp    = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // ===== NED → ENU 위치 변환 =====
    odom.pose.pose.position.x =  px4_odom->position[1];  // ENU x = NED y
    odom.pose.pose.position.y =  px4_odom->position[0];  // ENU y = NED x
    odom.pose.pose.position.z = -px4_odom->position[2];  // ENU z = -NED z

    // ===== NED → ENU 쿼터니언 변환 =====
    // PX4 q 배열: [w, x, y, z]
    tf2::Quaternion q_ned(
        px4_odom->q[1],  // x
        px4_odom->q[2],  // y
        px4_odom->q[3],  // z
        px4_odom->q[0]   // w
    );

    // NED → ENU 프레임 회전 행렬 적용
    tf2::Quaternion q_ned_to_enu;
    q_ned_to_enu.setRPY(M_PI, 0.0, M_PI / 2.0);
    tf2::Quaternion q_enu = q_ned_to_enu * q_ned;
    q_enu.normalize();

    // ===== 더블 커버 해결: w가 음수면 전체 부호 반전 =====
    // q와 -q는 수학적으로 같은 회전이지만, atan2 기반 yaw 계산 등에서
    // w < 0이면 yaw가 ±π 뒤집혀 나오는 문제가 생김
    // 항상 w >= 0인 "canonical form"으로 통일함
    if (q_enu.w() < 0.0) {
        q_enu = tf2::Quaternion(-q_enu.x(), -q_enu.y(), -q_enu.z(), -q_enu.w());
    }

    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();
    odom.pose.pose.orientation.w = q_enu.w();

    // ===== NED → ENU 선속도 변환 =====
    odom.twist.twist.linear.x =  px4_odom->velocity[1];
    odom.twist.twist.linear.y =  px4_odom->velocity[0];
    odom.twist.twist.linear.z = -px4_odom->velocity[2];

    // ===== NED → ENU 각속도 변환 =====
    // 선속도와 동일한 규칙 적용: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
    odom.twist.twist.angular.x =  px4_odom->angular_velocity[1];
    odom.twist.twist.angular.y =  px4_odom->angular_velocity[0];
    odom.twist.twist.angular.z = -px4_odom->angular_velocity[2];

    nav_odom_pub_->publish(odom);
}

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
