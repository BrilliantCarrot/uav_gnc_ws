#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

// ============================================================
// LidarPoseCorrectionNode
// ============================================================
// [이 노드의 목적]
//   Week 8 LiDAR-aided navigation 실험에서 EKF/UKF에 넣을
//   LiDAR 기반 위치 보정값을 만들어주는 노드임.
//
// [중요한 전제]
//   이 코드는 FAST-LIO, LIO-SAM처럼 실제 scan matching / factor graph / IMU preintegration을
//   모두 구현한 완전한 LIO가 아님.
//   현재 프로젝트 범위에서는 다음 구조를 먼저 검증하는 것이 목적임.
//
//     /sim/odom + /lidar/points_filtered
//        → LiDAR 관측 가능성 확인
//        → noise가 포함된 LiDAR-like pose measurement 생성
//        → /lidar/pose_odom publish
//        → navigation_node에서 EKF/UKF measurement update에 사용
//
// [왜 ground truth odom을 사용하나]
//   실제 LiDAR odometry라면 point cloud 정합(ICP/NDT/scan-to-map 등)으로 pose를 얻어야 함.
//   하지만 Week 8의 핵심은 "LiDAR pose correction이 EKF/UKF에 들어갔을 때
//   GPS-denied 항법 구조가 성립하는가"를 검증하는 것임.
//   그래서 우선 /sim/odom에 noise, rate 제한, point cloud 유효성 조건을 걸어
//   LiDAR-derived pose measurement를 근사적으로 생성함.
//
// [주의사항]
//   "완전한 LIO 구현" 인가? -> X.
//   "LiDAR-derived pose correction을 EKF/UKF에 융합한 GPS-denied navigation 구조 검증" -> O.
// ============================================================
class LidarPoseCorrectionNode : public rclcpp::Node
{
public:
  LidarPoseCorrectionNode() : Node("lidar_pose_correction_node")
  {
    // ------------------------------------------------------------
    // ROS2 파라미터 선언부
    // ------------------------------------------------------------
    // gt_odom_topic_:
    //   현재 시뮬레이션에서 ground truth 역할을 하는 /sim/odom을 받음.
    //   이 값을 그대로 쓰는 것이 아니라, LiDAR pose measurement처럼 보이도록 noise와 조건을 추가해서 발행함.
    gt_odom_topic_ = declare_parameter<std::string>("gt_odom_topic", "/sim/odom");

    // input_cloud_topic_:
    //   LiDAR 전처리 노드가 출력한 point cloud를 받음.
    //   이 topic은 pose 계산에 직접 쓰는 것이 아니라, "LiDAR가 주변 구조물을 충분히 보고 있는가"를 판단하는 데 사용함.
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/lidar/points_filtered");

    // output_odom_topic_:
    //   navigation_node가 구독할 LiDAR pose correction topic임.
    //   nav_msgs/Odometry를 쓰는 이유는 position, orientation, covariance를 한 메시지에 담기 좋기 때문임.
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/lidar/pose_odom");

    // frame_id_는 pose가 표현되는 기준 좌표계임.
    // 현재 프로젝트에서는 world frame 기준으로 /sim/odom, /nav/odom이 만들어지므로 동일하게 world로 둠.
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    // child_frame_id_는 이 odometry가 설명하는 대상 frame임.
    // 실제 TF를 발행하는 건 아니지만, 메시지 의미를 명확히 하기 위해 지정함.
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "lidar_pose");

    // LiDAR pose correction 발행 주파수임.
    // IMU는 고주파 prediction, LiDAR는 저주파 correction이라는 센서융합 구조를 만들기 위해 5Hz 정도로 설정함.
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);

    // 위치 측정 노이즈 표준편차임.
    // 실제 LiDAR scan matching도 완벽하지 않으므로 x/y/z에 노이즈를 추가해 현실성을 높임.
    pos_noise_std_xy_ = declare_parameter<double>("pos_noise_std_xy", 0.15);
    pos_noise_std_z_ = declare_parameter<double>("pos_noise_std_z", 0.20);

    // yaw 측정 노이즈 표준편차임.
    // 현재 navigation_node에서는 위치 성분을 update_gps() 형태로 쓰고 있지만,
    // 추후 update_lidar_pose(pos, yaw)를 만들면 이 yaw도 직접 보정에 사용할 수 있음.
    yaw_noise_std_deg_ = declare_parameter<double>("yaw_noise_std_deg", 3.0);

    // LiDAR point cloud가 너무 적으면 주변 구조물을 충분히 본 것이 아니라고 판단함.
    // 이 조건을 넣어야 단순히 ground truth를 무조건 복사하는 구조가 아니라,
    // LiDAR 관측 가능할 때만 correction이 들어오는 구조가 됨.
    min_points_for_update_ = declare_parameter<int>("min_points_for_update", 20);

    // point cloud가 너무 오래된 데이터이면 현재 pose와 맞지 않을 수 있으므로 버림.
    // 센서융합에서 timestamp freshness는 매우 중요함.
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", 0.5);

    // 일부 LiDAR update를 의도적으로 누락시키는 비율임.
    // 통신 누락, scan matching 실패 같은 상황을 시험하고 싶을 때 사용함.
    dropout_rate_ = declare_parameter<double>("dropout_rate", 0.0);

    // random seed를 고정해두면 같은 실험을 반복했을 때 noise sequence가 재현 가능함.
    // 포트폴리오/비교 실험에서는 재현성이 중요함.
    random_seed_ = declare_parameter<int>("random_seed", 7);

    // ------------------------------------------------------------
    // 파라미터 안전 보정
    // ------------------------------------------------------------
    // 주파수가 0 이하이면 timer period 계산이 불가능하므로 기본값으로 되돌림.
    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 5.0;

    // point threshold가 음수이면 의미가 없으므로 0으로 clamp함.
    if (min_points_for_update_ < 0) min_points_for_update_ = 0;

    // dropout_rate는 확률이므로 [0, 1] 범위로 제한함.
    dropout_rate_ = std::clamp(dropout_rate_, 0.0, 1.0);

    // ------------------------------------------------------------
    // 난수 생성기 및 분포 초기화
    // ------------------------------------------------------------
    // std::mt19937는 C++의 Mersenne Twister 난수 생성기임.
    // 같은 seed를 넣으면 같은 난수열이 나오므로 실험 재현성이 생김.
    rng_.seed(static_cast<std::uint32_t>(random_seed_));

    // 정규분포 noise를 생성함.
    // 평균 0, 표준편차는 yaml에서 설정한 값으로 둠.
    noise_xy_ = std::normal_distribution<double>(0.0, pos_noise_std_xy_);
    noise_z_ = std::normal_distribution<double>(0.0, pos_noise_std_z_);
    noise_yaw_ = std::normal_distribution<double>(0.0, yaw_noise_std_deg_ * kPi / 180.0);

    // dropout 여부 판단용 균등분포임.
    // 0~1 사이 난수를 뽑아 dropout_rate보다 작으면 이번 update를 버림.
    dropout_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    // ------------------------------------------------------------
    // Subscriber / Publisher 생성
    // ------------------------------------------------------------
    // /sim/odom 구독:
    //   LiDAR pose correction의 기준이 되는 ground truth pose를 가져옴.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      gt_odom_topic_, 10,
      std::bind(&LidarPoseCorrectionNode::odomCallback, this, std::placeholders::_1));

    // /lidar/points_filtered 구독:
    //   point 수와 timestamp만 사용함.
    //   실제 ICP/NDT를 하지 않는 대신, point cloud가 충분할 때만 pose correction을 허용함.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarPoseCorrectionNode::cloudCallback, this, std::placeholders::_1));

    // /lidar/pose_odom 발행:
    //   navigation_node가 이 값을 GPS 대신/또는 GPS와 병렬로 measurement update에 사용함.
    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

    // timer 주기로 pose correction 발행을 시도함.
    // 콜백이 들어올 때마다 바로 발행하지 않는 이유는 LiDAR correction rate를 명확히 제어하기 위함임.
    const int period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, period_ms)),
      std::bind(&LidarPoseCorrectionNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "[LiDAR Pose Correction] gt=%s cloud=%s output=%s rate=%.1fHz min_points=%d noise_xy=%.2f noise_z=%.2f yaw_noise=%.2fdeg",
      gt_odom_topic_.c_str(), input_cloud_topic_.c_str(), output_odom_topic_.c_str(),
      publish_rate_hz_, min_points_for_update_, pos_noise_std_xy_, pos_noise_std_z_, yaw_noise_std_deg_);
  }

private:
  // roll, pitch, yaw를 담는 간단한 구조체임.
  // ROS 메시지는 quaternion을 사용하지만 yaw noise를 넣으려면 yaw angle이 필요하므로 변환용으로 사용함.
  struct Rpy
  {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 최신 /sim/odom을 저장함.
    // timer callback과 subscriber callback이 서로 다른 타이밍에 접근하므로 mutex로 보호함.
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = *msg;
    odom_received_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 최신 point cloud의 timestamp와 point 수만 저장함.
    // PointCloud2에서 width*height는 전체 point 개수임.
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_stamp_ = msg->header.stamp;
    latest_cloud_points_ = static_cast<int>(msg->width * msg->height);
    cloud_received_ = true;
  }

  void onTimer()
  {
    nav_msgs::msg::Odometry odom;
    rclcpp::Time cloud_stamp;
    int cloud_points = 0;

    {
      // 공유 데이터 복사 구간임.
      // lock을 오래 잡고 있으면 subscriber callback이 밀릴 수 있으므로,
      // 필요한 데이터만 지역 변수로 복사하고 lock을 바로 해제하는 구조임.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!odom_received_ || !cloud_received_) return;
      odom = latest_odom_;
      cloud_stamp = latest_cloud_stamp_;
      cloud_points = latest_cloud_points_;
    }

    // point 수가 부족하면 LiDAR 관측이 부실하다고 판단하고 update를 만들지 않음.
    // 예: 장애물/벽을 거의 못 본 경우 scan matching도 불안정할 수 있음.
    if (cloud_points < min_points_for_update_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[LiDAR Pose Correction] not enough cloud points: %d < %d",
        cloud_points, min_points_for_update_);
      return;
    }

    // 오래된 cloud라면 현재 odom과 시간적으로 맞지 않으므로 버림.
    // 센서융합에서 시간 동기화가 안 되면 잘못된 위치 보정이 들어가 필터가 오히려 나빠질 수 있음.
    const double cloud_age = (now() - cloud_stamp).seconds();
    if (cloud_age > max_cloud_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[LiDAR Pose Correction] cloud too old: %.3f sec", cloud_age);
      return;
    }

    // dropout_rate가 설정되어 있으면 일정 확률로 이번 measurement를 누락시킴.
    // 센서 측정 실패 상황을 시험하기 위한 옵션임.
    if (dropout_rate_ > 0.0 && dropout_dist_(rng_) < dropout_rate_) {
      return;
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = now();
    out.header.frame_id = frame_id_;
    out.child_frame_id = child_frame_id_;

    // 위치에는 Gaussian noise를 추가함.
    // 이 값이 navigation_node에서 EKF/UKF measurement update에 들어가므로,
    // noise_std를 크게 하면 보정은 거칠고 약해지고, 작게 하면 ground truth에 더 가깝게 동작함.
    out.pose.pose.position.x = odom.pose.pose.position.x + noise_xy_(rng_);
    out.pose.pose.position.y = odom.pose.pose.position.y + noise_xy_(rng_);
    out.pose.pose.position.z = odom.pose.pose.position.z + noise_z_(rng_);

    // quaternion에서 roll/pitch/yaw를 추출함.
    // yaw noise를 넣기 위해 yaw angle만 따로 보정한 뒤 다시 quaternion으로 변환함.
    const Rpy rpy = quatToRpy(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);

    const double noisy_yaw = normalizeAngle(rpy.yaw + noise_yaw_(rng_));
    const auto q = rpyToQuat(rpy.roll, rpy.pitch, noisy_yaw);

    out.pose.pose.orientation.w = q[0];
    out.pose.pose.orientation.x = q[1];
    out.pose.pose.orientation.y = q[2];
    out.pose.pose.orientation.z = q[3];

    // twist는 일단 /sim/odom 값을 그대로 복사함.
    // 현재 navigation_node에서는 위치 보정 중심으로 사용하므로 twist는 핵심은 아님.
    out.twist = odom.twist;

    // covariance는 이 측정값이 어느 정도 불확실한지 표현하는 메타정보임.
    // 현재 EKF/UKF update_gps() 내부 R_gps_는 별도 고정값을 쓰지만,
    // 나중에 msg covariance 기반 adaptive update를 만들 때 활용 가능함.
    fillCovariance(out);

    pose_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "[LiDAR Pose Correction] published pose=(%.2f, %.2f, %.2f), cloud_points=%d",
      out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z, cloud_points);
  }

  Rpy quatToRpy(double w, double x, double y, double z) const
  {
    // quaternion은 회전을 특이점 없이 표현하기 좋은 4차원 표현임.
    // 다만 yaw만 수정하려면 roll/pitch/yaw로 한 번 변환할 필요가 있음.
    const double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (norm < 1e-12) return {};

    // quaternion은 단위 크기(norm=1)를 가져야 올바른 회전을 뜻함.
    // 입력값에 미세한 수치 오차가 있을 수 있으므로 정규화함.
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;

    Rpy rpy;

    // roll 계산 공식임.
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    rpy.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch 계산 공식임.
    // asin 입력값이 수치 오차로 [-1,1]을 살짝 벗어날 수 있으므로 copysign으로 안전 처리함.
    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
      rpy.pitch = std::copysign(kPi / 2.0, sinp);
    } else {
      rpy.pitch = std::asin(sinp);
    }

    // yaw 계산 공식임.
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp);

    return rpy;
  }

  std::array<double, 4> rpyToQuat(double roll, double pitch, double yaw) const
  {
    // roll/pitch/yaw를 quaternion으로 되돌리는 함수임.
    // 반환 순서는 ROS geometry_msgs Quaternion과 맞추기 위해 [w, x, y, z]로 둠.
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    std::array<double, 4> q;
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
    return q;
  }

  double normalizeAngle(double angle) const
  {
    // 각도는 같은 방향이라도 pi를 넘으면 -pi 근처와 동치가 됨.
    // 필터나 그래프에서 각도 점프를 줄이기 위해 [-pi, pi] 범위로 정규화함.
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
  }

  void fillCovariance(nav_msgs::msg::Odometry & msg) const
  {
    // ROS Odometry covariance는 6x6 행렬을 36개 배열로 펼친 형태임.
    // index 0,7,14는 x,y,z 분산이고 21,28,35는 roll,pitch,yaw 분산임.
    for (double & v : msg.pose.covariance) v = 0.0;
    for (double & v : msg.twist.covariance) v = 0.0;

    const double var_xy = pos_noise_std_xy_ * pos_noise_std_xy_;
    const double var_z = pos_noise_std_z_ * pos_noise_std_z_;
    const double var_yaw = std::pow(yaw_noise_std_deg_ * kPi / 180.0, 2.0);

    // 위치 covariance를 실제 추가한 noise variance와 맞춰줌.
    msg.pose.covariance[0] = var_xy;
    msg.pose.covariance[7] = var_xy;
    msg.pose.covariance[14] = var_z;

    // roll/pitch는 이 노드에서 의미 있게 보정하는 대상이 아니므로 매우 큰 covariance를 둠.
    // 즉, 이 측정값이 roll/pitch를 잘 안다고 주장하지 않도록 함.
    msg.pose.covariance[21] = 999.0;
    msg.pose.covariance[28] = 999.0;

    // yaw는 noise_yaw_를 적용했으므로 그 분산을 기록함.
    msg.pose.covariance[35] = var_yaw;
  }

  static constexpr double kPi = 3.14159265358979323846;

  // ------------------------------------------------------------
  // 파라미터 저장 변수들
  // ------------------------------------------------------------
  std::string gt_odom_topic_;
  std::string input_cloud_topic_;
  std::string output_odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  double publish_rate_hz_{5.0};
  double pos_noise_std_xy_{0.15};
  double pos_noise_std_z_{0.20};
  double yaw_noise_std_deg_{3.0};
  int min_points_for_update_{20};
  double max_cloud_age_sec_{0.5};
  double dropout_rate_{0.0};
  int random_seed_{7};

  // noise 생성을 위한 난수 관련 변수들임.
  std::mt19937 rng_;
  std::normal_distribution<double> noise_xy_;
  std::normal_distribution<double> noise_z_;
  std::normal_distribution<double> noise_yaw_;
  std::uniform_real_distribution<double> dropout_dist_;

  // subscriber callback과 timer callback이 공유하는 최신 데이터임.
  // mutex로 보호하지 않으면 한쪽에서 쓰는 중 다른 쪽에서 읽는 race condition이 생길 수 있음.
  std::mutex mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time latest_cloud_stamp_{0};
  int latest_cloud_points_{0};
  bool odom_received_{false};
  bool cloud_received_{false};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  // ROS2 노드 실행부임.
  // init → spin → shutdown 순서가 rclcpp 기본 패턴임.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPoseCorrectionNode>());
  rclcpp::shutdown();
  return 0;
}