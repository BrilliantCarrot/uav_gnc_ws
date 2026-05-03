#include <mutex>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "uav_gnc/ekf.h" // 우리가 만든 EKF 헤더
#include "uav_gnc/ukf.h"

using namespace std::chrono_literals;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode() : Node("navigation_node")
  {
    // 파라미터 선언: yaml 파일에서 토픽 이름들을 가져와서 유연하게 바꿀 수 있게 함
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/sim/imu");
    gps_topic_ = this->declare_parameter<std::string>("gps_topic", "/sim/gps/pos");
    out_topic_ = this->declare_parameter<std::string>("out_topic", "/nav/odom");
    filter_type_ = this->declare_parameter<std::string>("filter_type", "ukf");  // ← 추가

    const Eigen::Vector3d    init_p = Eigen::Vector3d::Zero();
    const Eigen::Vector3d    init_v = Eigen::Vector3d::Zero();
    const Eigen::Quaterniond init_q = Eigen::Quaterniond::Identity();
    
    // 필터 초기화 (일단 0,0,0에서 시작한다고 가정함)
    // 실제 환경에선 첫 GPS 데이터를 받았을 때 그 위치로 초기화하는 것이 훨씬 안전함
    if (filter_type_ == "ukf") {
        use_ukf_ = true;
        ukf_.init(init_p, init_v, init_q);
        RCLCPP_INFO(this->get_logger(), "[Navigation] Filter: UKF");
    } else {
        use_ukf_ = false;
        ekf_.init(init_p, init_v, init_q);
        RCLCPP_INFO(this->get_logger(), "[Navigation] Filter: EKF");
    }

    // Publisher: 추정된 최종 상태(Odometry)를 제어기나 시각화 노드로 보내기 위한 퍼블리셔 생성함
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    // Subscriber 1: IMU (Prediction 단계 - High Frequency)
    // IMU 데이터를 받는 구독자임. IMU는 100Hz 이상으로 아주 빠르게 들어오므로, 여기서 EKF의 예측(Prediction)을 수행함
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&NavigationNode::imuCallback, this, std::placeholders::_1));

    // Subscriber 2: GPS (Update 단계 - Low Frequency)
    // GPS 데이터를 받는 구독자임. GPS는 10Hz 정도로 느리게 들어오므로, 여기서 EKF의 오차 보정(Update)을 수행함
    gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      gps_topic_, 10,
      std::bind(&NavigationNode::gpsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Navigation Node Started with EKF.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 1. dt 계산
    // 이전 메시지와 현재 메시지 사이의 시간차(dt)를 초 단위로 계산함. 적분(예측)하려면 필수임
    rclcpp::Time current_time = msg->header.stamp;
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // dt가 너무 크거나 작으면 예외 처리 (시뮬레이션 리셋 등 상황 대비)
    // dt가 비정상적으로 크거나 0 이하면 튕기게 함 (시뮬레이션 초기화 시 시간이 꼬이는 현상 방지용임)
    if (dt <= 0.0 || dt > 1.0) return;

    // 2. IMU 데이터 추출 (ROS msg -> Eigen)
    // ROS 메시지 형식을 수학 계산이 편한 Eigen 벡터로 변환함
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // [추가] 제어기로 넘겨주기 위해(D 게인) Gyro 각속도 데이터 저장, (Bias 보정은 EKF에서 꺼내와도 되지만, 일단 자이로의 Raw값을 씀)
    // 제어기의 D-Gain(미분 제어)을 위해 현재 각속도를 따로 저장해둠. (안 그러면 제어기가 드론의 회전 속도를 몰라서 진동함)
    // 더 정교하게 하려면: current_gyro_ = gyro - ekf_.getGyroBias();
    current_gyro_ = gyro; 
    // 3. KF에서의 예측 (Prediction)
    // IMU 가속도와 각속도, 그리고 흐른 시간(dt)을 KF에 밀어 넣어서 현재 위치/자세를 눈감고 추측함
    // 변경
    if (use_ukf_) {
        ukf_.predict(acc, gyro, dt);
    } else {
        ekf_.predict(acc, gyro, dt);
    }

    // 4. 추정된 상태 Publish (/nav/odom)
    // IMU가 들어올 때마다(100Hz) 최신 추정 상태를 계산해서 제어기로 빠르게 쏴줌
    publishOdometry(msg->header.stamp);
  }

  void gpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // 1. GPS 데이터 추출
    // GPS에서 들어온 3차원 위치(x, y, z) 정보만 쏙 빼냄
    Eigen::Vector3d meas_pos(msg->point.x, msg->point.y, msg->point.z);

    // 2. KF를 통한 보정 (Update)
    // 이 GPS 위치를 기준 삼아 여태까지 누적된 IMU 예측 오차를 확 깎아냄 (수정/보정 단계임)
    if (use_ukf_) {
        ukf_.update_gps(meas_pos);
    } else {
        ekf_.update_gps(meas_pos);
    }
    
    // 로그를 가끔 찍어서 동작 확인
    // RCLCPP_INFO(this->get_logger(), "GPS Update: %.2f, %.2f, %.2f", meas_pos.x(), meas_pos.y(), meas_pos.z());
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    // 1. EKF에서 현재 추정된 상태 가져오기
    // 계산이 끝난 최종 위치, 속도, 자세를 EKF 객체에서 꺼내옴
    Eigen::Vector3d p, v;
    Eigen::Quaterniond q;
    if (use_ukf_) {
        p = ukf_.getPosition();
        v = ukf_.getVelocity();
        q = ukf_.getAttitude();
    } else {
        p = ekf_.getPosition();
        v = ekf_.getVelocity();
        q = ekf_.getAttitude();
    }

    // ==========================================
    // [추가된 안전장치] NaN 체크
    // ==========================================
    // 위치(p), 속도(v), 자세(q) 중 하나라도 숫자가 아니면(NaN) 절대 보내지 않음
    // 계산 터진 쓰레기 값을 제어기로 보내면 드론이 미쳐 날뛰기 때문에 여기서 원천 차단함
    bool is_p_nan = std::isnan(p.x()) || std::isnan(p.y()) || std::isnan(p.z());
    bool is_v_nan = std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
    bool is_q_nan = std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z());

    if (is_p_nan || is_v_nan || is_q_nan) {
        // 로그를 너무 많이 찍으면 렉이 걸리니 1초에 한 번만 경고 출력
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "EKF Output is NaN! Skipping publish to protect controller.");
        return; // 여기서 함수 강제 종료 (쓰레기 값 전송 차단)
    }

    // ==========================================
    // [추가된 안전장치] 폭주(Divergence) 체크
    // ==========================================
    // 위치가 10km 이상 튀면 필터가 발산한 것으로 간주하고 차단
    if (std::abs(p.x()) > 10000.0 || std::abs(p.y()) > 10000.0 || std::abs(p.z()) > 10000.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "EKF Diverged (Too large position)! Skipping publish.");
        return;
    }

    // ==========================================
    // 2. 데이터가 정상이면 메시지 생성 및 발행
    // ==========================================
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world"; // 이 데이터는 world 좌표계를 기준으로 한다는 뜻
    odom.child_frame_id = "base_link"; // 이 데이터가 설명하는 대상은 드론 본체(base_link)라는 뜻

    // Position (위치 채워넣기)
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();

    // Orientation (자세 채워넣기)
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Velocity (World frame 기준 속도 채워넣기)
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();

    // Angular Velocity는 EKF 상태에 없다면 IMU 값을 그대로 쓰거나 비워둠 (여기선 생략)
    // 각속도 채워넣기 (NaN 방지 위해 EKF에서 꺼내오는 대신, IMU 콜백에서 저장해둔 current_gyro_ 사용)
    // [추가] 제어기의 D-gain을 위해 각속도(Angular Velocity) 필수
    odom.twist.twist.angular.x = current_gyro_.x();
    odom.twist.twist.angular.y = current_gyro_.y();
    odom.twist.twist.angular.z = current_gyro_.z();
    
    odom_pub_->publish(odom); // 다 채운 메시지 발행
  }

private:
  std::string filter_type_;
  bool use_ukf_{false};
  EKF ekf_;
  UKF ukf_;

  rclcpp::Time last_time_{0}; // 이전 dt 계산을 위해 시간 기억해두는 변수

  // [nan 방지용 추가] 각속도 전달을 위한 변수
  Eigen::Vector3d current_gyro_{0.0, 0.0, 0.0}; 

  std::string imu_topic_;
  std::string gps_topic_;
  std::string out_topic_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gps_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}