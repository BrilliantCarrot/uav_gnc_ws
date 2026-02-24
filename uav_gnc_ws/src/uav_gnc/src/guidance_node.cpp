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
    z_ref_ = this->declare_parameter<double>("z_ref", 1.0); // 고정 고도(비행 높이)
    
    // 어떤 유도 알고리즘을 쓸지 결정하는 스위치
    // "lookahead", "min_jerk", "min_snap", "multi_snap" 중 하나를 받음
    guidance_mode_ = this->declare_parameter<std::string>("guidance_mode", "multi_snap");

    // 웨이포인트(목표점) 리스트
    wp_x_ = this->declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{0.0});
    wp_y_ = this->declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{0.0});
    
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

    RCLCPP_INFO(this->get_logger(), "Guidance Mode: [%s]", guidance_mode_.c_str());
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
    current_z_ = msg->pose.pose.position.z;
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
    double pf_x = wp_x_[wp_index_ + 1], pf_y = wp_y_[wp_index_ + 1];

    // 두 점 사이의 거리를 평균 속도로 나누어 '목표 시간(T)'을 계산
    double dist = std::hypot(pf_x - p0_x, pf_y - p0_y);
    current_segment_T_ = std::max(0.1, dist / avg_speed_);

    if (guidance_mode_ == "min_jerk") {
      jerk_x_.generate(p0_x, 0.0, 0.0, pf_x, 0.0, 0.0, current_segment_T_);
      jerk_y_.generate(p0_y, 0.0, 0.0, pf_y, 0.0, 0.0, current_segment_T_);
    } else {
      snap_x_.generate(p0_x, 0.0, 0.0, 0.0, pf_x, 0.0, 0.0, 0.0, current_segment_T_);
      snap_y_.generate(p0_y, 0.0, 0.0, 0.0, pf_y, 0.0, 0.0, 0.0, current_segment_T_);
    }
    
    segment_start_time_ = this->now(); // 이 구간을 시작한 '현재 시각' 기록
    is_trajectory_active_ = true;
  }

  // ---------------------------------------------------
  // 5. 다중 구간 궤적 생성 (Multi-segment Minimum Snap)
  // 모든 웨이포인트를 한 번에 스캔해서 부드러운 레이싱 트랙을 만듬
  // ---------------------------------------------------
  void generateMultiSegmentTrajectory() {
    std::vector<double> full_wp_x, full_wp_y, times;
    
    full_wp_x.push_back(current_x_); // 내 현재 위치를 출발점으로 삽입
    full_wp_y.push_back(current_y_);

    for (size_t i = 0; i < wp_x_.size(); ++i) {
        double dx = wp_x_[i] - full_wp_x.back();
        double dy = wp_y_[i] - full_wp_y.back();
        double dist = std::hypot(dx, dy);
        
        if (dist < 0.1) continue; // 웨이포인트가 너무 촘촘하면 무시 (오류 방지)

        full_wp_x.push_back(wp_x_[i]);
        full_wp_y.push_back(wp_y_[i]);
        times.push_back(dist / avg_speed_); // 각 구간별 시간 할당
    }

    if (times.empty()) return;

    // 수학 엔진(trajectory.cpp)에 배열을 통째로 넘겨서 행렬 방정식을 품
    multi_snap_x_.generate(full_wp_x, times);
    multi_snap_y_.generate(full_wp_y, times);

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

    // [수정] PoseStamped 대신 Odometry 메시지 사용 (위치 + 속도 동시 전달)
    nav_msgs::msg::Odometry sp;
    sp.header.stamp = this->now();
    sp.header.frame_id = last_frame_id_.empty() ? "world" : last_frame_id_;
    sp.pose.pose.position.z = z_ref_; // 고도는 파라미터로 고정 (추후 3D 최적화로 확장 가능)

    // ==========================================
    // 모드 1: 6주차 (Look-ahead 모드)
    // 수학적 궤적 없이, 목표점 주변(accept_radius)에 도달하면 다음 점으로 넘어감
    // ==========================================
    if (guidance_mode_ == "lookahead") {
      const size_t N = wp_x_.size();
      if (wp_index_ >= N) wp_index_ = N - 1;
      
      double dx = wp_x_[wp_index_] - current_x_, dy = wp_y_[wp_index_] - current_y_;
      double dist = std::hypot(dx, dy);

      // 목표 반경 안에 들어왔으면 다음 Waypoint로 인덱스 증가
      if (dist < accept_radius_) {
        if (wp_index_ + 1 < N) wp_index_++;
        else if (!hold_last_) return;
      }
      
      // Look-ahead: 현재 위치에서 목표를 향해 일정 거리(L)만큼만 당겨서 명령
      double spx = (dist < 1e-6) ? wp_x_[wp_index_] : current_x_ + (dx / dist) * std::min(lookahead_dist_, dist);
      double spy = (dist < 1e-6) ? wp_y_[wp_index_] : current_y_ + (dy / dist) * std::min(lookahead_dist_, dist);

      // [수정] pose.position -> pose.pose.position
      sp.pose.pose.position.x = spx; 
      sp.pose.pose.position.y = spy;
      sp.pose.pose.orientation = yawToQuat(std::atan2(spy - current_y_, spx - current_x_)); // 진행 방향 바라보기

      // [추가] 방향 벡터를 기반으로 평균 속도(avg_speed_)만큼 피드포워드 속도 인가
      sp.twist.twist.linear.x = (dist < 1e-6) ? 0.0 : (dx / dist) * avg_speed_;
      sp.twist.twist.linear.y = (dist < 1e-6) ? 0.0 : (dy / dist) * avg_speed_;
      sp.twist.twist.linear.z = 0.0;
    } 
    
    // ==========================================
    // 모드 2: 단일 구간 다항식 (Min Jerk / Min Snap)
    // ==========================================
    else if (guidance_mode_ == "min_jerk" || guidance_mode_ == "min_snap") {
      if (is_trajectory_active_) {
        double t = (this->now() - segment_start_time_).seconds(); // 출발 후 흐른 시간
        double spx, spy, vx, vy;

        // 시간에 따른 정답 위치와 속도를 받아옴
        if (guidance_mode_ == "min_jerk") {
          spx = jerk_x_.getPosition(t); spy = jerk_y_.getPosition(t);
          vx = jerk_x_.getVelocity(t); vy = jerk_y_.getVelocity(t);
        } else {
          spx = snap_x_.getPosition(t); spy = snap_y_.getPosition(t);
          vx = snap_x_.getVelocity(t); vy = snap_y_.getVelocity(t);
        }
        
        // [수정] pose.position -> pose.pose.position
        sp.pose.pose.position.x = spx; 
        sp.pose.pose.position.y = spy;
        
        // 속도 벡터를 이용해 기체가 가야 할 방향(Yaw) 부드럽게 계산
        current_yaw_ = (std::hypot(vx, vy) > 0.05) ? std::atan2(vy, vx) : current_yaw_;
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        // [추가] 다항식에서 계산된 정답 속도를 피드포워드로 바로 넘김
        sp.twist.twist.linear.x = vx;
        sp.twist.twist.linear.y = vy;
        sp.twist.twist.linear.z = 0.0;

        // 이 구간의 할당 시간(T)이 다 끝났다면?
        if (t >= current_segment_T_) {
          wp_index_++;
          if (wp_index_ < wp_x_.size() - 1) generateTrajectoryForCurrentSegment(); // 다음 구간 계산
          else is_trajectory_active_ = false; // 모든 비행 종료
        }
      } else {
        // 비행 종료 후 제자리 유지
        if (!hold_last_) return;
        sp.pose.pose.position.x = wp_x_.back(); 
        sp.pose.pose.position.y = wp_y_.back();
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        // [추가] 멈춘 상태이므로 속도 0
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
        
        // pose.position -> pose.pose.position
        sp.pose.pose.position.x = multi_snap_x_.getPosition(t);
        sp.pose.pose.position.y = multi_snap_y_.getPosition(t);

        double vx = multi_snap_x_.getVelocity(t);
        double vy = multi_snap_y_.getVelocity(t);

        // 0.001초 뒤의 속도를 구해 수치미분으로 정확한 목표 가속도 획득
        double vx_next = multi_snap_x_.getVelocity(t + 0.001);
        double vy_next = multi_snap_y_.getVelocity(t + 0.001);
        double ax = (vx_next - vx) / 0.001;
        double ay = (vy_next - vy) / 0.001;
        
        current_yaw_ = (std::hypot(vx, vy) > 0.05) ? std::atan2(vy, vx) : current_yaw_;
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        // 다항식에서 계산된 정답 속도를 피드포워드로 바로 넘김
        sp.twist.twist.linear.x = vx;
        sp.twist.twist.linear.y = vy;
        sp.twist.twist.linear.z = 0.0;

        // Odometry 빈 공간(angular)에 목표 가속도 몰래 담아 보내기
        sp.twist.twist.angular.x = ax;
        sp.twist.twist.angular.y = ay;
        sp.twist.twist.angular.z = 0.0;

        if (t >= total_multi_T_) is_trajectory_active_ = false; // 총 비행시간이 끝나면 종료
      } else {
        if (!hold_last_) return;
        sp.pose.pose.position.x = wp_x_.back(); 
        sp.pose.pose.position.y = wp_y_.back();
        sp.pose.pose.orientation = yawToQuat(current_yaw_);

        // 멈춘 상태이므로 속도 0
        sp.twist.twist.linear.x = 0.0;
        sp.twist.twist.linear.y = 0.0;
        sp.twist.twist.linear.z = 0.0;

        // 멈춘 상태이므로 가속도 0
        sp.twist.twist.angular.x = 0.0;
        sp.twist.twist.angular.y = 0.0;
        sp.twist.twist.angular.z = 0.0;
      }
    }
    // 최종적으로 계산된 "가야 할 곳(Setpoint) 및 목표 속도"를 발행하여 제어기(ControlNode)로 전달
    setpoint_pub_->publish(sp);
  }

private:
  std::string guidance_mode_{"multi_snap"};
  bool use_nav_odom_{true}, hold_last_{true}, have_odom_{false};
  std::string setpoint_topic_, nav_odom_topic_, sim_odom_topic_, last_frame_id_;
  double rate_hz_{20.0}, z_ref_{1.0}, accept_radius_{0.5}, lookahead_dist_{1.5}, avg_speed_{1.0};
  double current_x_{0.0}, current_y_{0.0}, current_z_{0.0}, current_yaw_{0.0};
  size_t wp_index_{0};
  std::vector<double> wp_x_, wp_y_;

  // 궤적 생성기 인스턴스들
  MinJerkTrajectory jerk_x_, jerk_y_;
  MinSnapTrajectory snap_x_, snap_y_;
  MultiMinSnapTrajectory multi_snap_x_, multi_snap_y_; 
  
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