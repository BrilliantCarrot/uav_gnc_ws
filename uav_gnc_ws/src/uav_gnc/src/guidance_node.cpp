#include <cmath>
#include <chrono>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "uav_gnc/trajectory.h" // 궤적 생성기 헤더

using namespace std::chrono_literals;

// ======================================================================
// GuidanceNode 클래스: 드론의 유도(Guidance)를 담당하는 ROS2 노드
// ======================================================================
class GuidanceNode : public rclcpp::Node
{
public:
  GuidanceNode() : Node("guidance_node")
  {
    // ---------------------------------------------------
    // 1. 파라미터(Parameter) 불러오기
    // yaml 파일에서 설정값들을 가져옴(하드 코딩 최소화)
    // ---------------------------------------------------
    use_nav_odom_ = this->declare_parameter<bool>("use_nav_odom", true);
    setpoint_topic_ = this->declare_parameter<std::string>("setpoint_topic", "/guidance/setpoint");
    nav_odom_topic_ = this->declare_parameter<std::string>("nav_odom_topic", "/nav/odom");
    sim_odom_topic_ = this->declare_parameter<std::string>("sim_odom_topic", "/sim/odom");
    rate_hz_ = this->declare_parameter<double>("rate_hz", 20.0);
    
    // 어떤 유도 알고리즘을 쓸지 결정하는 스위치
    guidance_mode_ = this->declare_parameter<std::string>("guidance_mode", "multi_snap");

    // 웨이포인트(목표점) 리스트
    wp_x_ = this->declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.0});
    wp_y_ = this->declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});
    // yaml에 추가한 Z축 웨이포인트를 파라미터에서 불러옴 (3D 비행의 핵심임)
    wp_z_ = this->declare_parameter<std::vector<double>>("waypoints_z", std::vector<double>{1.0}); 
    
    // 6주차용 파라미터 (단순 추종)
    accept_radius_ = this->declare_parameter<double>("accept_radius", 0.5); // 목표 반경 도달 기준
    lookahead_dist_ = this->declare_parameter<double>("lookahead_dist", 1.5); // 앞을 내다보는 거리
    
    // 7주차용 파라미터 (궤적 생성용)
    avg_speed_ = this->declare_parameter<double>("avg_speed", 1.0); // 평균 이동 속도
    hold_last_ = this->declare_parameter<bool>("hold_last", true);  // 마지막에 멈출지 여부

    // ---------------------------------------------------
    // 2. Pub / Sub 및 타이머 설정
    // ---------------------------------------------------
    std::string odom_topic = use_nav_odom_ ? nav_odom_topic_ : sim_odom_topic_;
    
    // 드론의 현재 위치(Odometry)를 듣는 Subscriber
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&GuidanceNode::odomCallback, this, std::placeholders::_1));
      
    // 제어기에게 목표 위치(Setpoint)를 명령하는 Publisher
    setpoint_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(setpoint_topic_, 10);

    // rate_hz_(예: 20Hz = 0.05초)마다 onTimer 함수를 실행시키는 타이머
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GuidanceNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Guidance Mode: [%s], 3D Waypoints Loaded.", guidance_mode_.c_str());
  }

private:
  // Yaw 각도(회전)를 쿼터니언(Quaternion)으로 변환해주는 헬퍼 함수
  static geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5); q.x = 0.0; q.y = 0.0; q.z = std::sin(yaw * 0.5);
    return q;
  }

  // ---------------------------------------------------
  // 3. Odometry 콜백 (내 위치 확인 및 궤적 생성 트리거)
  // ---------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 센서/EKF로부터 현재 위치를 업데이트 받음
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z; // 현재 Z 고도 갱신함
    last_frame_id_ = msg->header.frame_id;
    
    // 노드가 켜지고 '최초로' 위치를 수신했을 때 딱 한 번 궤적을 만들어줌
    if (!have_odom_) {
      have_odom_ = true;
      if (guidance_mode_ == "min_jerk" || guidance_mode_ == "min_snap") {
        generateTrajectoryForCurrentSegment(); // 단일 구간 궤적 생성
      } else if (guidance_mode_ == "multi_snap") {
        generateMultiSegmentTrajectory();      // 다중 구간 전체 궤적 한 번에 생성
      }
    }
  }

  // ---------------------------------------------------
  // 4. 단일 구간 궤적 생성 (Min Jerk / Min Snap)
  // 지금 있는 곳에서 '다음' 웨이포인트까지만의 곡선을 만듬
  // ---------------------------------------------------
  void generateTrajectoryForCurrentSegment() {
    if (wp_index_ >= wp_x_.size() - 1) return; // 이미 끝에 도달함
    
    double p0_x = wp_index_ == 0 ? current_x_ : wp_x_[wp_index_];
    double p0_y = wp_index_ == 0 ? current_y_ : wp_y_[wp_index_];
    double p0_z = wp_index_ == 0 ? current_z_ : wp_z_[wp_index_]; // Z축 시작점 추가함
    
    double pf_x = wp_x_[wp_index_ + 1];
    double pf_y = wp_y_[wp_index_ + 1];
    double pf_z = wp_z_[wp_index_ + 1]; // Z축 도착점 추가함

    // 두 점 사이의 거리를 3차원 유클리드 거리로 계산하여 '목표 시간(T)'을 구함 (C++17 std::hypot 3인자 오버로딩 활용)
    double dist = std::hypot(pf_x - p0_x, pf_y - p0_y, pf_z - p0_z);
    current_segment_T_ = std::max(0.1, dist / avg_speed_);

    if (guidance_mode_ == "min_jerk") {
      jerk_x_.generate(p0_x, 0.0, 0.0, pf_x, 0.0, 0.0, current_segment_T_);
      jerk_y_.generate(p0_y, 0.0, 0.0, pf_y, 0.0, 0.0, current_segment_T_);
      jerk_z_.generate(p0_z, 0.0, 0.0, pf_z, 0.0, 0.0, current_segment_T_); // Z축 저크 궤적 생성함
    } else {
      snap_x_.generate(p0_x, 0.0, 0.0, 0.0, pf_x, 0.0, 0.0, 0.0, current_segment_T_);
      snap_y_.generate(p0_y, 0.0, 0.0, 0.0, pf_y, 0.0, 0.0, 0.0, current_segment_T_);
      snap_z_.generate(p0_z, 0.0, 0.0, 0.0, pf_z, 0.0, 0.0, 0.0, current_segment_T_); // Z축 스냅 궤적 생성함
    }
    
    segment_start_time_ = this->now(); // 이 구간을 시작한 '현재 시각' 기록
    is_trajectory_active_ = true;
  }

  // ---------------------------------------------------
  // 5. 다중 구간 궤적 생성 (Multi-segment Minimum Snap)
  // 모든 웨이포인트를 한 번에 스캔해서 부드러운 레이싱 트랙을 만듬
  // ---------------------------------------------------
  void generateMultiSegmentTrajectory() {
    std::vector<double> full_wp_x, full_wp_y, full_wp_z, times;
    
    full_wp_x.push_back(current_x_); // 내 현재 위치를 출발점으로 삽입
    full_wp_y.push_back(current_y_);
    full_wp_z.push_back(current_z_); // Z축 출발점 삽입함

    for (size_t i = 0; i < wp_x_.size(); ++i) {
        double dx = wp_x_[i] - full_wp_x.back();
        double dy = wp_y_[i] - full_wp_y.back();
        double dz = wp_z_[i] - full_wp_z.back(); // Z축 차이 계산함
        
        // 3차원 공간에서의 거리로 구간별 이동 시간을 구함
        double dist = std::hypot(dx, dy, dz); 
        
        if (dist < 0.1) continue; // 웨이포인트가 너무 촘촘하면 무시 (오류 방지)

        full_wp_x.push_back(wp_x_[i]);
        full_wp_y.push_back(wp_y_[i]);
        full_wp_z.push_back(wp_z_[i]); // Z축 웨이포인트 추가함
        times.push_back(dist / avg_speed_); // 각 구간별 시간 할당
    }

    if (times.empty()) return;

    // 수학 엔진(trajectory.cpp)에 배열을 통째로 넘겨서 3D 행렬 방정식을 품
    multi_snap_x_.generate(full_wp_x, times);
    multi_snap_y_.generate(full_wp_y, times);
    multi_snap_z_.generate(full_wp_z, times); // Z축 다중 궤적 연산 돌림

    segment_start_time_ = this->now();
    is_trajectory_active_ = true;
    total_multi_T_ = multi_snap_x_.getTotalTime(); // 총 비행시간 기록
  }

  // ---------------------------------------------------
  // 6. 메인 타이머 루프 (제어기에게 계속 Setpoint 쏘기)
  // 20Hz(0.05초)마다 실행되면서 드론을 이끌어 줌
  // ---------------------------------------------------
  void onTimer() {
    if (!have_odom_) return; // 내 위치를 모르면 명령을 내릴 수 없음

    nav_msgs::msg::Odometry sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = last_frame_id_.empty() ? "world" : last_frame_id_;

    // ==========================================
    // 모드 1: 6주차 (Look-ahead 모드)
    // 수학적 궤적 없이, 목표점 주변(accept_radius)에 도달하면 다음 점으로 넘어감
    // ==========================================
    if (guidance_mode_ == "lookahead") {
      const size_t N = wp_x_.size();
      if (wp_index_ >= N) wp_index_ = N - 1;
      
      double dx = wp_x_[wp_index_] - current_x_;
      double dy = wp_y_[wp_index_] - current_y_;
      double dz = wp_z_[wp_index_] - current_z_; // Z축 방향 벡터 계산함
      
      double dist = std::hypot(dx, dy, dz); // 3D 거리 계산함

      // 목표 반경 안에 들어왔으면 다음 Waypoint로 인덱스 증가
      if (dist < accept_radius_) {
        if (wp_index_ + 1 < N) wp_index_++;
        else if (!hold_last_) return;
      }
      
      // Look-ahead: 현재 위치에서 3D 목표를 향해 일정 거리(L)만큼만 당겨서 명령함
      double spx = (dist < 1e-6) ? wp_x_[wp_index_] : current_x_ + (dx / dist) * std::min(lookahead_dist_, dist);
      double spy = (dist < 1e-6) ? wp_y_[wp_index_] : current_y_ + (dy / dist) * std::min(lookahead_dist_, dist);
      double spz = (dist < 1e-6) ? wp_z_[wp_index_] : current_z_ + (dz / dist) * std::min(lookahead_dist_, dist);

      sp.pose.pose.position.x = spx; 
      sp.pose.pose.position.y = spy;
      sp.pose.pose.position.z = spz; // Z축 목표 위치 세팅함
      
      sp.pose.pose.orientation = yawToQuat(std::atan2(spy - current_y_, spx - current_x_)); // 진행 방향 바라보기

      // Z축을 포함하여 방향 벡터 기반으로 피드포워드 속도 인가함
      sp.twist.twist.linear.x = (dist < 1e-6) ? 0.0 : (dx / dist) * avg_speed_;
      sp.twist.twist.linear.y = (dist < 1e-6) ? 0.0 : (dy / dist) * avg_speed_;
      sp.twist.twist.linear.z = (dist < 1e-6) ? 0.0 : (dz / dist) * avg_speed_;
    } 
    
    // ==========================================
    // 모드 2: 단일 구간 다항식 (Min Jerk / Min Snap)
    // ==========================================
    else if (guidance_mode_ == "min_jerk" || guidance_mode_ == "min_snap") {
      if (is_trajectory_active_) {
        double t = (this->now() - segment_start_time_).seconds(); // 출발 후 흐른 시간
        double spx, spy, spz, vx, vy, vz;

        // 시간에 따른 3D 정답 위치와 속도를 받아옴
        if (guidance_mode_ == "min_jerk") {
          spx = jerk_x_.getPosition(t); spy = jerk_y_.getPosition(t); spz = jerk_z_.getPosition(t);
          vx = jerk_x_.getVelocity(t); vy = jerk_y_.getVelocity(t); vz = jerk_z_.getVelocity(t);
        } else {
          spx = snap_x_.getPosition(t); spy = snap_y_.getPosition(t); spz = snap_z_.getPosition(t);
          vx = snap_x_.getVelocity(t); vy = snap_y_.getVelocity(t); vz = snap_z_.getVelocity(t);
        }
        
        sp.pose.pose.position.x = spx; 
        sp.pose.pose.position.y = spy;
        sp.pose.pose.position.z = spz; // Z축 목표 위치 업데이트함
        
        // 속도 벡터를 이용해 기체가 가야 할 방향(Yaw) 부드럽게 계산 (Yaw는 주로 XY 평면 기준으로 계산 유지)
        current_yaw_ = (std::hypot(vx, vy) > 0.05) ? std::atan2(vy, vx) : current_yaw_;
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        // 다항식에서 계산된 3D 정답 속도를 피드포워드로 바로 넘김
        sp.twist.twist.linear.x = vx;
        sp.twist.twist.linear.y = vy;
        sp.twist.twist.linear.z = vz; // Z축 피드포워드 속도 할당함

        // 이 구간의 할당 시간(T)이 다 끝났다면?
        if (t >= current_segment_T_) {
          wp_index_++;
          if (wp_index_ < wp_x_.size() - 1) generateTrajectoryForCurrentSegment(); // 다음 구간 계산
          else is_trajectory_active_ = false; // 모든 비행 종료
        }
      } else {
        // 비행 종료 후 제자리 유지 (마지막 3D Waypoint에 고정시킴)
        if (!hold_last_) return;
        sp.pose.pose.position.x = wp_x_.back(); 
        sp.pose.pose.position.y = wp_y_.back();
        sp.pose.pose.position.z = wp_z_.back(); // Z축 제자리 유지함
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        sp.twist.twist.linear.x = 0.0;
        sp.twist.twist.linear.y = 0.0;
        sp.twist.twist.linear.z = 0.0;
      }
    }
    
    // ==========================================
    // 모드 3: 다중 구간 (Multi-segment Minimum Snap)
    // 중간에 멈추는 로직 없이 전체 시간(total_multi_T_) 동안 물 흐르듯 진행
    // ==========================================
    else if (guidance_mode_ == "multi_snap") {
      if (is_trajectory_active_) {
        double t = (this->now() - segment_start_time_).seconds();
        
        // 3D 위치 타겟 설정함
        sp.pose.pose.position.x = multi_snap_x_.getPosition(t);
        sp.pose.pose.position.y = multi_snap_y_.getPosition(t);
        sp.pose.pose.position.z = multi_snap_z_.getPosition(t);

        double vx = multi_snap_x_.getVelocity(t);
        double vy = multi_snap_y_.getVelocity(t);
        double vz = multi_snap_z_.getVelocity(t); // Z축 속도 가져옴

        // 0.001초 뒤의 속도를 구해 수치미분으로 정확한 3D 목표 가속도 획득함
        double vx_next = multi_snap_x_.getVelocity(t + 0.001);
        double vy_next = multi_snap_y_.getVelocity(t + 0.001);
        double vz_next = multi_snap_z_.getVelocity(t + 0.001); // Z축 가속도용 미래 속도 구함
        
        double ax = (vx_next - vx) / 0.001;
        double ay = (vy_next - vy) / 0.001;
        double az = (vz_next - vz) / 0.001; // Z축 목표 가속도 연산함
        
        current_yaw_ = (std::hypot(vx, vy) > 0.05) ? std::atan2(vy, vx) : current_yaw_;
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        // 다항식에서 계산된 3D 정답 속도를 피드포워드로 바로 넘김
        sp.twist.twist.linear.x = vx;
        sp.twist.twist.linear.y = vy;
        sp.twist.twist.linear.z = vz; // Z축 피드포워드 속도 인가함

        // Odometry 빈 공간(angular)에 3D 목표 가속도 몰래 담아 보내기
        sp.twist.twist.angular.x = ax;
        sp.twist.twist.angular.y = ay;
        sp.twist.twist.angular.z = az; // Z축 피드포워드 가속도 인가함

        if (t >= total_multi_T_) is_trajectory_active_ = false; // 총 비행시간이 끝나면 종료
      } else {
        if (!hold_last_) return;
        sp.pose.pose.position.x = wp_x_.back(); 
        sp.pose.pose.position.y = wp_y_.back();
        sp.pose.pose.position.z = wp_z_.back(); // Z축 정지 위치 유지함
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        sp.twist.twist.linear.x = 0.0;
        sp.twist.twist.linear.y = 0.0;
        sp.twist.twist.linear.z = 0.0;

        sp.twist.twist.angular.x = 0.0;
        sp.twist.twist.angular.y = 0.0;
        sp.twist.twist.angular.z = 0.0;
      }
    }
    // 최종적으로 계산된 "가야 할 곳(Setpoint) 및 목표 속도/가속도"를 발행하여 제어기로 전달
    setpoint_pub_->publish(sp);
  }

private:
  std::string guidance_mode_{"multi_snap"};
  bool use_nav_odom_{true}, hold_last_{true}, have_odom_{false};
  std::string setpoint_topic_, nav_odom_topic_, sim_odom_topic_, last_frame_id_;
  double rate_hz_{20.0}, accept_radius_{0.5}, lookahead_dist_{1.5}, avg_speed_{1.0};
  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0}, current_yaw_{0.0};
  size_t wp_index_{0};
  std::vector<double> wp_x_, wp_y_, wp_z_; // Z축 웨이포인트 벡터 추가함

  // 궤적 생성기 인스턴스들 (Z축 모두 추가 완료함)
  MinJerkTrajectory jerk_x_, jerk_y_, jerk_z_;
  MinSnapTrajectory snap_x_, snap_y_, snap_z_;
  MultiMinSnapTrajectory multi_snap_x_, multi_snap_y_, multi_snap_z_; 
  
  rclcpp::Time segment_start_time_;
  double current_segment_T_{0.0}, total_multi_T_{0.0};
  bool is_trajectory_active_{false};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}