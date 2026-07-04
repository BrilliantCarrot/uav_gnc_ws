#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

// ======================================================================
// PX4BridgeNode: guidance_node의 setpoint를 PX4 Offboard 토픽으로 변환
//
// 구독: /guidance/setpoint (nav_msgs/Odometry) — ENU 좌표계
// 퍼블리시:
//   /fmu/in/offboard_control_mode — 100ms heartbeat (필수)
//   /fmu/in/trajectory_setpoint  — 목표 위치 (NED 좌표계)
//   /fmu/in/vehicle_command      — Arm/Disarm, 모드 전환
//
// 좌표계 변환 (ENU → NED):
//   x_ned =  y_enu
//   y_ned =  x_enu
//   z_ned = -z_enu
// ======================================================================
class PX4BridgeNode : public rclcpp::Node
{
public:
  PX4BridgeNode() : Node("px4_bridge_node"), offboard_counter_(0), is_armed_(false)
  {
    // ===== 파라미터 =====
    arm_on_start_    = this->declare_parameter<bool>("arm_on_start", false);
    takeoff_z_enu_   = this->declare_parameter<double>("takeoff_z_enu", 2.0); // 이륙 고도 (ENU, m)

    // ===== 퍼블리셔 =====
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    traj_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    // ===== 구독자 =====
    // guidance_node가 퍼블리시하는 목표 위치 (ENU)
    setpoint_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/guidance/setpoint", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_setpoint_ = msg;
      });

    // PX4 odom 구독 (상태 모니터링용) — BEST_EFFORT QoS 필수
    // SensorDataQoS = BEST_EFFORT + VOLATILE (PX4 표준)
    px4_odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        latest_px4_odom_ = msg;
      });

    // ===== 100ms 타이머 — Offboard heartbeat (필수) =====
    // PX4는 2Hz 이상으로 offboard_control_mode를 받지 못하면 Offboard 모드 해제
    timer_ = this->create_wall_timer(100ms, [this]() { timerCallback(); });

    // 초기 setpoint: 제자리 호버링 (고도 takeoff_z_enu_)
    latest_setpoint_ = nullptr;

    RCLCPP_INFO(this->get_logger(), "PX4 Bridge Node started");
    RCLCPP_INFO(this->get_logger(), "Waiting for /guidance/setpoint...");
  }

private:
  // ===== 타이머 콜백 =====
  void timerCallback()
  {
    // 1. Offboard heartbeat 퍼블리시 (항상)
    publishOffboardControlMode();

    // 2. setpoint 퍼블리시
    if (latest_setpoint_ != nullptr) {
      publishTrajectorySetpoint(latest_setpoint_);
    } else {
      // guidance_node 연결 전엔 현재 위치 유지 (제자리 호버링)
      publishHoverSetpoint();
    }

    // 3. Offboard 모드 진입 + Arm (10번 heartbeat 후)
    if (offboard_counter_ == 10 && arm_on_start_ && !is_armed_) {
      RCLCPP_INFO(this->get_logger(), "Switching to Offboard mode and Arming...");
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
      arm();
    }

    if (offboard_counter_ < 11) offboard_counter_++;
  }

  // ===== Offboard 제어 모드 heartbeat =====
  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position     = true;   // position 제어 사용
    msg.velocity     = false;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
    offboard_mode_pub_->publish(msg);
  }

  // ===== guidance_node setpoint → TrajectorySetpoint 변환 및 퍼블리시 =====
  void publishTrajectorySetpoint(const nav_msgs::msg::Odometry::SharedPtr& odom)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    // ENU → NED 좌표계 변환
    // ENU: x=East,  y=North, z=Up
    // NED: x=North, y=East,  z=Down
    float x_enu = static_cast<float>(odom->pose.pose.position.x);
    float y_enu = static_cast<float>(odom->pose.pose.position.y);
    float z_enu = static_cast<float>(odom->pose.pose.position.z);

    msg.position[0] =  y_enu;   // NED x = ENU y (North)
    msg.position[1] =  x_enu;   // NED y = ENU x (East)
    msg.position[2] = -z_enu;   // NED z = -ENU z (Down)

    // quaternion → yaw 변환 후 ENU → NED yaw 변환
    // NED yaw = -(ENU yaw) + PI/2 (좌표계 회전 보정)
    tf2::Quaternion q(
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w
    );
    double roll, pitch, yaw_enu;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_enu);
    float yaw_ned = static_cast<float>(-yaw_enu + M_PI / 2.0);

    msg.yaw       = yaw_ned;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_setpoint_pub_->publish(msg);
  }

  // ===== setpoint 없을 때 현재 위치 호버링 =====
  void publishHoverSetpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    if (latest_px4_odom_ != nullptr) {
      // PX4 odom은 이미 NED 좌표계 — 위치 그대로 사용
      msg.position[0] = latest_px4_odom_->position[0];
      msg.position[1] = latest_px4_odom_->position[1];
      msg.position[2] = latest_px4_odom_->position[2];

      // quaternion → yaw 변환 (q 배열: [w, x, y, z])
      tf2::Quaternion q(
        latest_px4_odom_->q[1],  // x
        latest_px4_odom_->q[2],  // y
        latest_px4_odom_->q[3],  // z
        latest_px4_odom_->q[0]   // w
      );
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      msg.yaw = static_cast<float>(yaw);
    } else {
      // PX4 odom도 없으면 원점 고도 2m 호버링
      msg.position[0] =  0.0f;
      msg.position[1] =  0.0f;
      msg.position[2] = -static_cast<float>(takeoff_z_enu_); // NED z
      msg.yaw         =  0.0f;
    }

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_setpoint_pub_->publish(msg);
  }

  // ===== Arm 명령 =====
  void arm()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    is_armed_ = true;
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
  }

  // ===== VehicleCommand 퍼블리시 =====
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command          = command;
    msg.param1           = param1;
    msg.param2           = param2;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_cmd_pub_->publish(msg);
  }

  // ===== 멤버 변수 =====
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      vehicle_cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr  px4_odom_sub_;

  nav_msgs::msg::Odometry::SharedPtr      latest_setpoint_;
  px4_msgs::msg::VehicleOdometry::SharedPtr latest_px4_odom_;

  uint64_t offboard_counter_;
  bool     is_armed_;
  bool     arm_on_start_;
  double   takeoff_z_enu_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}