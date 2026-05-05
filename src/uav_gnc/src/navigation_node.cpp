#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "uav_gnc/ekf.h"
#include "uav_gnc/ukf.h"

using namespace std::chrono_literals;

// ============================================================
// NavigationNode
// ============================================================
// [이 노드의 역할]
//   UAV GNC 구조에서 N, 즉 Navigation을 담당하는 노드임.
//   simulation_node가 만든 센서값(/sim/imu, /sim/gps/pos, /lidar/pose_odom)을 받아
//   EKF 또는 UKF로 현재 상태를 추정하고, 그 결과를 /nav/odom으로 발행함.
//
// [프로젝트 전체 흐름에서의 위치]
//   /sim/imu                  → IMU prediction
//   /sim/gps/pos              → GPS position correction
//   /lidar/pose_odom          → LiDAR-derived pose correction
//   EKF/UKF                   → estimated state
//   /nav/odom                 → guidance, planner, controller가 믿고 쓰는 현재 상태
//
// [고도화 프로젝트 8주차에서 추가된 핵심]
//   기존에는 GPS update가 항상 사용되는 구조였음.
//   8주차에서는 use_gps_update, use_lidar_update, use_lidar_init_only를 추가해
//   아래 실험을 가능하게 했음.
//
//     1) GPS EKF baseline          : IMU + GPS
//     2) LiDAR-aided EKF/UKF       : IMU + LiDAR pose correction, GPS OFF
//     3) LiDAR-init + IMU-only     : 첫 LiDAR pose로만 초기화 후 IMU prediction만 사용
//
// [왜 filter_initialized_가 필요한가]
//   예전 구조처럼 EKF/UKF를 무조건 (0,0,0)에서 시작하고 IMU prediction을 바로 돌리면,
//   launch 초반에 잘못된 /nav/odom이 발행되어 guidance/control이 이상한 상태를 믿을 수 있음.
//   그래서 첫 GPS 또는 첫 LiDAR pose correction이 들어오기 전까지는 prediction/publish를 막고,
//   첫 measurement를 기준으로 필터를 초기화함.
// ============================================================
class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode() : Node("navigation_node")
  {
    // ------------------------------------------------------------
    // 기본 topic 파라미터
    // ------------------------------------------------------------
    // yaml 파일에서 topic 이름을 바꿀 수 있게 declare_parameter로 선언함.
    // 이렇게 해두면 코드 재컴파일 없이 실험 조건을 바꿀 수 있음.
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/sim/imu");
    gps_topic_ = this->declare_parameter<std::string>("gps_topic", "/sim/gps/pos");
    out_topic_ = this->declare_parameter<std::string>("out_topic", "/nav/odom");

    // filter_type은 "ekf" 또는 "ukf"로 설정함.
    // 같은 navigation_node 안에서 필터만 바꿔 실험할 수 있게 해둔 구조임.
    filter_type_ = this->declare_parameter<std::string>("filter_type", "ekf");

    // ------------------------------------------------------------
    // Week 8 실험 모드 파라미터
    // ------------------------------------------------------------
    // use_gps_update_:
    //   true이면 /sim/gps/pos가 들어올 때 GPS update 수행함.
    //   false이면 GPS topic은 구독하더라도 보정에는 사용하지 않음.
    use_gps_update_ = this->declare_parameter<bool>("use_gps_update", true);

    // use_lidar_update_:
    //   true이면 /lidar/pose_odom이 들어올 때 LiDAR pose correction을 수행함.
    //   GPS-denied 실험에서 핵심적으로 사용하는 옵션임.
    use_lidar_update_ = this->declare_parameter<bool>("use_lidar_update", false);

    // LiDAR pose correction topic 이름임.
    lidar_pose_topic_ = this->declare_parameter<std::string>("lidar_pose_topic", "/lidar/pose_odom");

    // LiDAR measurement가 현재 추정값과 너무 멀리 떨어져 있으면 outlier로 보고 버리기 위한 threshold임.
    // scan matching 실패나 잘못된 measurement가 필터를 망가뜨리는 것을 막는 방어 장치임.
    lidar_reject_threshold_m_ = this->declare_parameter<double>("lidar_reject_threshold_m", 10.0);

    // use_lidar_init_only_:
    //   첫 LiDAR pose는 초기화에만 쓰고, 그 이후 LiDAR update는 사용하지 않는 모드임.
    //   IMU-only drift를 비교하기 위한 실패/비교군 실험에 사용함.
    this->declare_parameter<bool>("use_lidar_init_only", false);
    this->get_parameter("use_lidar_init_only", use_lidar_init_only_);

    // filter_type 문자열을 bool flag로 변환함.
    // use_ukf_가 true이면 UKF 객체를 사용하고, false이면 EKF 객체를 사용함.
    use_ukf_ = (filter_type_ == "ukf");

    if (use_ukf_) {
      RCLCPP_INFO(this->get_logger(), "[Navigation] Filter: UKF");
    } else {
      RCLCPP_INFO(this->get_logger(), "[Navigation] Filter: EKF");
    }

    // /nav/odom publisher 생성.
    // 이 topic은 guidance_node, control_node, path_planner_node 등이 현재 상태로 사용함.
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    // IMU subscriber 생성.
    // IMU는 고주파 센서이므로 prediction 단계에 사용함.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&NavigationNode::imuCallback, this, std::placeholders::_1));

    // GPS subscriber 생성.
    // use_gps_update_가 false이면 콜백 초반에서 return하므로 실제 보정에는 사용하지 않음.
    gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      gps_topic_, 10,
      std::bind(&NavigationNode::gpsCallback, this, std::placeholders::_1));

    // LiDAR pose subscriber 생성.
    // LiDAR update 또는 LiDAR init-only 모드 중 하나라도 켜져 있으면 필요함.
    if (use_lidar_update_ || use_lidar_init_only_) {
      lidar_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lidar_pose_topic_, 10,
        std::bind(&NavigationNode::lidarPoseCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Navigation Node Started. filter=%s, gps_update=%s, lidar_update=%s, lidar_init_only=%s",
      filter_type_.c_str(),
      use_gps_update_ ? "true" : "false",
      use_lidar_update_ ? "true" : "false",
      use_lidar_init_only_ ? "true" : "false");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time current_time = msg->header.stamp;

    // ------------------------------------------------------------
    // 필터 초기화 전 처리
    // ------------------------------------------------------------
    // 필터가 아직 초기화되지 않았다면 IMU prediction을 수행하지 않음.
    // 이유:
    //   IMU는 위치를 직접 알려주는 센서가 아니라 가속도/각속도를 적분해야 하는 센서임.
    //   초기 위치/자세가 잘못 잡힌 상태에서 적분을 시작하면 /nav/odom이 초반부터 튈 수 있음.
    if (!filter_initialized_) {
      last_time_ = current_time;

      // GPS도 LiDAR도 전혀 쓰지 않는 순수 IMU-only 모드를 실험하고 싶을 때,
      // 초기화 source가 없으면 필터가 영원히 시작되지 않으므로 zero 초기화를 허용함.
      // 다만 Week 8의 IMU-only 비교군은 보통 use_lidar_init_only=true로 첫 pose만 잡는 방식을 사용함.
      if (!use_gps_update_ && !use_lidar_update_ && !use_lidar_init_only_) {
        const Eigen::Vector3d init_p = Eigen::Vector3d::Zero();
        const Eigen::Vector3d init_v = Eigen::Vector3d::Zero();
        const Eigen::Quaterniond init_q = Eigen::Quaterniond::Identity();
        initializeFilter(init_p, init_v, init_q, current_time, "zero / IMU-only mode");
      }

      return;
    }

    // last_time_이 아직 비어 있으면 이번 timestamp만 저장하고 return함.
    // dt를 계산하려면 이전 시간과 현재 시간이 모두 필요하기 때문임.
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // dt가 0 이하이거나 너무 크면 적분하지 않음.
    // launch 직후, rosbag seek, sim reset 같은 상황에서 비정상 dt가 들어올 수 있음.
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    // ROS IMU 메시지를 Eigen 벡터로 변환함.
    // EKF/UKF 내부 수식은 Eigen 기반으로 계산되므로 변환이 필요함.
    Eigen::Vector3d acc(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);

    Eigen::Vector3d gyro(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);

    // control_node가 angular velocity를 사용할 수 있도록 최신 gyro를 보관함.
    // 현재 EKF/UKF 상태에서 bias-corrected gyro를 꺼내는 구조가 아니므로 raw gyro를 사용함.
    current_gyro_ = gyro;

    // ------------------------------------------------------------
    // Prediction step
    // ------------------------------------------------------------
    // IMU prediction은 "센서 보정 없이 물리 모델로 다음 상태를 예측"하는 단계임.
    // GPS/LiDAR update보다 훨씬 높은 주기로 실행되어 /nav/odom을 부드럽게 유지함.
    if (use_ukf_) {
      ukf_.predict(acc, gyro, dt);
    } else {
      ekf_.predict(acc, gyro, dt);
    }

    // prediction 결과를 /nav/odom으로 발행함.
    publishOdometry(msg->header.stamp);
  }

  void gpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // GPS update OFF 실험에서는 GPS 메시지가 들어와도 사용하지 않음.
    // 이 switch 덕분에 GPS-denied 조건을 yaml만으로 만들 수 있음.
    if (!use_gps_update_) {
      return;
    }

    Eigen::Vector3d meas_pos(msg->point.x, msg->point.y, msg->point.z);

    // NaN/Inf measurement는 필터에 넣으면 상태 전체가 망가질 수 있으므로 차단함.
    if (!isFiniteVector(meas_pos)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[Navigation] GPS measurement has NaN/Inf. Skipping update.");
      return;
    }

    // 시뮬레이션에서 초기 GPS noise 때문에 z가 살짝 음수가 될 수 있음.
    // 드론이 지면 아래에서 시작했다고 필터가 믿지 않도록 최소 0으로 제한함.
    meas_pos.z() = std::max(0.0, meas_pos.z());

    // 필터가 아직 초기화되지 않았다면 첫 GPS measurement로 초기화함.
    // 기존처럼 무조건 원점 초기화하는 것보다 훨씬 안전함.
    if (!filter_initialized_) {
      const Eigen::Vector3d init_v = Eigen::Vector3d::Zero();
      const Eigen::Quaterniond init_q = Eigen::Quaterniond::Identity();
      initializeFilter(meas_pos, init_v, init_q, msg->header.stamp, "GPS");
      return;
    }

    // GPS position update 수행.
    if (use_ukf_) {
      ukf_.update_gps(meas_pos);
    } else {
      ekf_.update_gps(meas_pos);
    }
  }

  void lidarPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // LiDAR update도 아니고 init-only도 아니면 LiDAR pose를 사용할 이유가 없음.
    if (!use_lidar_update_ && !use_lidar_init_only_) {
      return;
    }

    // LiDAR pose measurement에서 위치 성분 추출함.
    Eigen::Vector3d meas_pos(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);

    if (!isFiniteVector(meas_pos)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[Navigation] LiDAR pose measurement has NaN/Inf. Skipping update.");
      return;
    }

    // 지면 아래 초기화 방지용 clamp임.
    meas_pos.z() = std::max(0.0, meas_pos.z());

    // LiDAR pose message의 orientation을 quaternion으로 읽음.
    // 현재 update는 위치만 사용하지만, 초기화 시 attitude도 사용할 수 있게 구조를 잡아둠.
    Eigen::Quaterniond meas_q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

    // quaternion이 비정상 값이면 identity로 대체함.
    // quaternion norm이 0에 가까우면 회전으로 사용할 수 없음.
    if (!std::isfinite(meas_q.w()) || !std::isfinite(meas_q.x()) ||
        !std::isfinite(meas_q.y()) || !std::isfinite(meas_q.z()) ||
        meas_q.norm() < 1e-6) {
      meas_q = Eigen::Quaterniond::Identity();
    } else {
      meas_q.normalize();
    }

    // 필터가 아직 초기화되지 않았다면 첫 LiDAR pose로 초기화함.
    // GPS-denied 실험에서는 이 경로가 매우 중요함.
    if (!filter_initialized_) {
      const Eigen::Vector3d init_v = Eigen::Vector3d::Zero();
      initializeFilter(meas_pos, init_v, meas_q, msg->header.stamp, "LiDAR pose");
      return;
    }

    // init-only 모드라면 첫 pose로 초기화만 하고 이후 correction은 사용하지 않음.
    // 이 모드는 IMU prediction만 사용했을 때 drift가 얼마나 커지는지 보여주는 비교군임.
    if (use_lidar_init_only_) {
      return;
    }

    // 현재 추정 위치와 LiDAR measurement 사이 차이를 innovation으로 계산함.
    // 너무 큰 innovation은 outlier일 가능성이 있으므로 reject함.
    const Eigen::Vector3d current_p = getCurrentPosition();
    const double innovation_norm = (meas_pos - current_p).norm();

    if (lidar_reject_threshold_m_ > 0.0 && innovation_norm > lidar_reject_threshold_m_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[Navigation] LiDAR pose rejected. innovation=%.3f m, threshold=%.3f m",
        innovation_norm, lidar_reject_threshold_m_);
      return;
    }

    // ------------------------------------------------------------
    // LiDAR pose correction update
    // ------------------------------------------------------------
    // 현재 EKF/UKF 클래스에는 update_lidar_pose(pos, yaw)가 따로 없으므로,
    // LiDAR pose의 위치 성분을 GPS position measurement와 같은 형태로 넣음.
    // 즉 이름은 update_gps()지만, 수학적으로는 "3D position measurement update"임.
    // 추후 yaw까지 보정하려면 EKF/UKF에 update_lidar_pose(pos, yaw)를 추가하면 됨.
    if (use_ukf_) {
      ukf_.update_gps(meas_pos);
    } else {
      ekf_.update_gps(meas_pos);
    }
  }

  void initializeFilter(
    const Eigen::Vector3d& init_p,
    const Eigen::Vector3d& init_v,
    const Eigen::Quaterniond& init_q,
    const rclcpp::Time& stamp,
    const std::string& source)
  {
    // quaternion은 반드시 단위 quaternion이어야 함.
    // norm이 비정상인 경우 identity로 대체함.
    Eigen::Quaterniond q = init_q;
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
        !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        q.norm() < 1e-6) {
      q = Eigen::Quaterniond::Identity();
    } else {
      q.normalize();
    }

    if (use_ukf_) {
      ukf_.init(init_p, init_v, q);
    } else {
      ekf_.init(init_p, init_v, q);
    }

    filter_initialized_ = true;
    last_time_ = stamp;

    RCLCPP_INFO(
      this->get_logger(),
      "[Navigation] filter initialized by %s: p=(%.3f, %.3f, %.3f)",
      source.c_str(), init_p.x(), init_p.y(), init_p.z());
  }

  Eigen::Vector3d getCurrentPosition() const
  {
    // 현재 활성화된 필터에서 위치만 꺼내는 helper 함수임.
    if (use_ukf_) {
      return ukf_.getPosition();
    }
    return ekf_.getPosition();
  }

  bool isFiniteVector(const Eigen::Vector3d& v) const
  {
    // NaN/Inf 방어용 helper 함수임.
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    // 필터 초기화 전에는 /nav/odom을 발행하지 않음.
    // 잘못된 초기 상태를 guidance/control이 믿게 되는 것을 막기 위함임.
    if (!filter_initialized_) {
      return;
    }

    Eigen::Vector3d p, v;
    Eigen::Quaterniond q;

    // 활성화된 필터에서 position, velocity, attitude를 가져옴.
    if (use_ukf_) {
      p = ukf_.getPosition();
      v = ukf_.getVelocity();
      q = ukf_.getAttitude();
    } else {
      p = ekf_.getPosition();
      v = ekf_.getVelocity();
      q = ekf_.getAttitude();
    }

    // 필터 출력 NaN/Inf 체크.
    // 이 방어 로직이 없으면 제어기에 비정상 값이 들어가서 시뮬레이션이 터질 수 있음.
    const bool is_p_nan = !isFiniteVector(p);
    const bool is_v_nan = !isFiniteVector(v);
    const bool is_q_nan =
      !std::isfinite(q.w()) || !std::isfinite(q.x()) ||
      !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
      q.norm() < 1e-6;

    if (is_p_nan || is_v_nan || is_q_nan) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[Navigation] filter output has NaN/Inf. Skipping /nav/odom publish.");
      return;
    }

    q.normalize();

    // 위치가 말도 안 되게 커지면 필터 발산으로 보고 발행하지 않음.
    // 단위는 m이고, 10000m는 이 프로젝트 스케일에서 사실상 비정상 값임.
    if (std::abs(p.x()) > 10000.0 || std::abs(p.y()) > 10000.0 || std::abs(p.z()) > 10000.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[Navigation] filter diverged. Skipping /nav/odom publish.");
      return;
    }

    // nav_msgs/Odometry 메시지 생성.
    // pose에는 위치/자세, twist에는 선속도/각속도를 넣음.
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();

    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();

    // EKF/UKF 상태에서 angular velocity를 따로 추정하지 않으므로,
    // 가장 최근 IMU gyro 값을 그대로 넣음.
    odom.twist.twist.angular.x = current_gyro_.x();
    odom.twist.twist.angular.y = current_gyro_.y();
    odom.twist.twist.angular.z = current_gyro_.z();

    odom_pub_->publish(odom);
  }

private:
  // 필터 선택 관련 변수
  std::string filter_type_;
  bool use_ukf_{false};
  bool filter_initialized_{false};

  // 실험 모드 제어 변수
  bool use_gps_update_{true};
  bool use_lidar_update_{false};
  bool use_lidar_init_only_{false};
  double lidar_reject_threshold_m_{10.0};

  // 실제 필터 객체
  EKF ekf_;
  UKF ukf_;

  // dt 계산용 이전 timestamp
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  // /nav/odom twist.angular에 넣을 최신 gyro 값
  Eigen::Vector3d current_gyro_{0.0, 0.0, 0.0};

  // topic 이름 저장 변수
  std::string imu_topic_;
  std::string gps_topic_;
  std::string out_topic_;
  std::string lidar_pose_topic_;

  // ROS interface 객체들
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_sub_;
};

int main(int argc, char** argv)
{
  // ROS2 C++ 노드 실행 기본 패턴임.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}