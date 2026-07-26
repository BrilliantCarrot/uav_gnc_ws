#include <chrono>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <random>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
// ROS2/ament에서는 “패키지의 include 디렉토리”가 include 경로로 잡히기 때문에
// 헤더는 include/를 빼고 패키지 경로 기준으로만 씀.
#include <uav_dynamics/multirotor_model.hpp>
#include <uav_dynamics/sixdof.hpp>

using namespace std::chrono_literals;


// 1. sim_node가 /control/wrench를 subscribe 해서 “가장 최근 wrench(힘/토크)”를 저장
// 2. 타이머(dt)마다 sixdof를 한 스텝 적분
// 3. 결과 상태를 /sim/odom으로 publish
class SimNode : public rclcpp::Node
{
public:
  SimNode() : Node("sim_node") // SimNode 노드 생성자, 노드 이름 "sim_node"로 초기화
  {
    // ===== parameters (yaml로 업데이트 계속 필요) =====
    dt_ = this->declare_parameter<double>("dt", 0.01);

    params_.mass = this->declare_parameter<double>("mass", 2.0);
    params_.inertia.x = this->declare_parameter<double>("Ix", 0.02);
    params_.inertia.y = this->declare_parameter<double>("Iy", 0.02);
    params_.inertia.z = this->declare_parameter<double>("Iz", 0.04);
    params_.g = this->declare_parameter<double>("g", 9.80665);

    params_.use_drag = this->declare_parameter<bool>("use_drag", true);
    params_.k1 = this->declare_parameter<double>("k1", 0.15);
    params_.k2 = this->declare_parameter<double>("k2", 0.02);
    enable_body_drag_ = this->declare_parameter<bool>("enable_body_drag", false);
    body_drag_linear_ = parseVec3Param(
      this->declare_parameter<std::vector<double>>("body_drag_linear", {0.0, 0.0, 0.0}),
      "body_drag_linear");
    body_drag_quadratic_ = parseVec3Param(
      this->declare_parameter<std::vector<double>>("body_drag_quadratic", {0.0, 0.0, 0.0}),
      "body_drag_quadratic");

    // robustness 테스트용 바람 외란 파라미터 불러오기
    params_.wind_force.x = this->declare_parameter<double>("wind_x", 0.0);
    params_.wind_force.y = this->declare_parameter<double>("wind_y", 0.0);
    params_.wind_force.z = 0.0; // 먼저 측풍(Crosswind)만 고려

    // 노이즈 표준편차 파라미터 (기본값 설정)
    double n_acc = this->declare_parameter<double>("noise_acc", 0.1); // 가속도계 노이즈 표준편차 (m/s^2)
    double n_gyr = this->declare_parameter<double>("noise_gyr", 0.01); // 자이로 노이즈 표준편차 (rad/s)
    double n_gps = this->declare_parameter<double>("noise_gps", 0.5); // GPS 노이즈 표준편차 (m) - 위치 측정 오차, EKF 튜닝에 중요
    ground_contact_imu_correction_ =
      this->declare_parameter<bool>("ground_contact_imu_correction", true);
    state_log_enabled_ = this->declare_parameter<bool>("state_log_enabled", true);
    state_log_period_ms_ = this->declare_parameter<int>("state_log_period_ms", 1000);

    enable_imu_bias_ = this->declare_parameter<bool>("enable_imu_bias", false);
    accel_bias_random_walk_std_ =
      this->declare_parameter<double>("accel_bias_random_walk_std", 0.0);
    gyro_bias_random_walk_std_ =
      this->declare_parameter<double>("gyro_bias_random_walk_std", 0.0);

    const auto accel_bias_body_param =
      this->declare_parameter<std::vector<double>>("accel_bias_body", {0.0, 0.0, 0.0});
    const auto gyro_bias_body_param =
      this->declare_parameter<std::vector<double>>("gyro_bias_body", {0.0, 0.0, 0.0});

    accel_bias_body_ = parseVec3Param(accel_bias_body_param, "accel_bias_body");
    gyro_bias_body_ = parseVec3Param(gyro_bias_body_param, "gyro_bias_body");

    if (accel_bias_random_walk_std_ < 0.0) accel_bias_random_walk_std_ = 0.0;
    if (gyro_bias_random_walk_std_ < 0.0) gyro_bias_random_walk_std_ = 0.0;

    // [중요] 혹시 0.0이 들어오면 강제로 기본값 설정 (NaN 방지)
    if (n_acc <= 0) n_acc = 0.1;
    if (n_gyr <= 0) n_gyr = 0.01;
    if (n_gps <= 0) n_gps = 0.5;

    // 분포 객체 생성 (평균 0, 표준편차 n)
    dist_acc_ = std::normal_distribution<double>(0.0, n_acc);
    dist_gyr_ = std::normal_distribution<double>(0.0, n_gyr);
    dist_gps_ = std::normal_distribution<double>(0.0, n_gps);
    dist_unit_ = std::normal_distribution<double>(0.0, 1.0);
    
    // 시드값 초기화 (이거 없으면 랜덤 안 될 수도 있음)
    // std::random_device rd; // 헤더 필요: #include <random>
    // generator_.seed(rd()); 
    // 그냥 간단하게 시간으로 시드 주거나, 기본 시드 사용해도 됨.

    // 초기 상태 (나중에 파라미터로 빼기 가능)
    state_.p = {0.0, 0.0, 0.0};
    state_.v = {0.0, 0.0, 0.0};
    state_.q = {1.0, 0.0, 0.0, 0.0};   // body->world
    state_.w = {0.0, 0.0, 0.0};        // body frame angular rate

    // 기본 입력 0
    input_.thrust_body = {0.0, 0.0, 0.0};
    input_.moment_body = {0.0, 0.0, 0.0};

    actuator_mode_ = this->declare_parameter<std::string>("actuator_mode", "direct_wrench");
    if (actuator_mode_ == "multirotor" || actuator_mode_ == "rpm_propeller") {
      const MultirotorConfig config = loadMultirotorConfig();
      MultirotorConfig configured_config = config;
      configured_config.use_rpm_propeller = (actuator_mode_ == "rpm_propeller");
      if (!multirotor_.configure(configured_config)) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid multirotor config. Falling back to direct_wrench mode.");
        actuator_mode_ = "direct_wrench";
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "actuator mode: %s (%s, rotors=%zu, motor_tau=%.3f)",
                    actuator_mode_.c_str(), configured_config.airframe_name.c_str(),
                    configured_config.rotors.size(), configured_config.motor_tau);
      }
    } else {
      actuator_mode_ = "direct_wrench";
      RCLCPP_INFO(this->get_logger(), "actuator mode: direct_wrench");
    }

    // ===== pub/sub =====
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sim/odom", 10);
    // "/sim/odom" 토픽으로 Odometry 메시지를 발행하는 큐 사이즈 10의 퍼블리셔 생성
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sim/imu", 10);
    // "/sim/imu" 토픽으로 Imu 메시지를 발행하는 큐 사이즈 10의 퍼블리셔 생성
    gps_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/sim/gps/pos", 10);
    // "/sim/gps/pos" 토픽으로  메시지를 발행하는 큐 사이즈 10의 퍼블리셔 생성
    actuator_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/sim/actuator/thrusts", 10);
    
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/control/wrench", 10,
      std::bind(&SimNode::wrenchCallback, this, std::placeholders::_1)
    );
    // create_subscription<메시지타입>(토픽이름, QoS, 콜백함수)
    // "/control/wrench" 토픽을 구독하여 힘/토크(WrenchStamped) 데이터를 수신하고,
    // 메시지가 도착하면 클래스 내 wrenchCallback 멤버 함수를 호출하도록 설정, 큐 사이즈는 10으로 설정
    // &SimNode::wrenchCallback → 멤버 함수 주소
    // this → 이 객체의 인스턴스
    // _1 → 첫 번째 인자를 그대로 전달
    // ===== timer =====
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&SimNode::onTimer, this)
    );

    // “dt_ 초마다 onTimer() 함수를 실행하는 타이머를 만듦
    RCLCPP_INFO(this->get_logger(), "simulation_node started (dt=%.4f)", dt_);
    RCLCPP_INFO(this->get_logger(),
      "IMU model: noise_acc=%.4f m/s^2, noise_gyr=%.5f rad/s, bias=%s, "
      "accel_bias=(%.4f, %.4f, %.4f) m/s^2, gyro_bias=(%.5f, %.5f, %.5f) rad/s",
      n_acc, n_gyr, enable_imu_bias_ ? "ON" : "OFF",
      accel_bias_body_.x, accel_bias_body_.y, accel_bias_body_.z,
      gyro_bias_body_.x, gyro_bias_body_.y, gyro_bias_body_.z);
    // SimNode() 생성자 맨 마지막 부분에 추가
    // RCLCPP_WARN(this->get_logger(), "=== SIM DEBUG ===");
    // RCLCPP_WARN(this->get_logger(), "Mass: %f", params_.mass);
    // RCLCPP_WARN(this->get_logger(), "Inertia X: %f", params_.inertia.x);
    // RCLCPP_WARN(this->get_logger(), "Noise Acc Param: %f", dist_acc_.stddev()); // 이거 확인 중요!
  }

private:
  Vec3 parseVec3Param(const std::vector<double>& values, const std::string& name) const
  {
    if (values.size() != 3) {
      RCLCPP_WARN(this->get_logger(),
        "%s must have exactly 3 values. Falling back to zeros.", name.c_str());
      return {0.0, 0.0, 0.0};
    }
    return {values[0], values[1], values[2]};
  }

  MultirotorConfig loadMultirotorConfig()
  {
    constexpr double inv_sqrt2 = 0.7071067811865475;
    const double arm = 0.225;
    const double x = arm * inv_sqrt2;
    const double y = arm * inv_sqrt2;

    MultirotorConfig config;
    config.airframe_name = this->declare_parameter<std::string>("airframe_name", "f450_quad_x");
    config.motor_tau = this->declare_parameter<double>("motor_tau", 0.04);
    config.yaw_torque_to_thrust = this->declare_parameter<double>("yaw_torque_to_thrust", 0.016);
    config.prop_thrust_coeff = this->declare_parameter<double>("prop_thrust_coeff", 1.8e-5);
    config.prop_torque_coeff = this->declare_parameter<double>("prop_torque_coeff", 2.8e-7);
    config.motor_omega_max = this->declare_parameter<double>("motor_omega_max", 900.0);
    config.battery_voltage_nominal =
      this->declare_parameter<double>("battery_voltage_nominal", 14.8);
    config.battery_voltage = this->declare_parameter<double>("battery_voltage", 14.8);
    config.enable_ground_effect =
      this->declare_parameter<bool>("enable_ground_effect", false);
    config.ground_effect_altitude_m =
      this->declare_parameter<double>("ground_effect_altitude_m", 0.8);
    config.ground_effect_gain =
      this->declare_parameter<double>("ground_effect_gain", 0.12);

    const std::vector<std::string> default_names{
      "front_left", "front_right", "rear_right", "rear_left"
    };
    const std::vector<double> default_positions{
       x,  y, 0.0,
       x, -y, 0.0,
      -x, -y, 0.0,
      -x,  y, 0.0
    };
    const std::vector<double> default_yaw_signs{1.0, -1.0, 1.0, -1.0};
    const std::vector<double> default_min{0.0, 0.0, 0.0, 0.0};
    const std::vector<double> default_max{15.0, 15.0, 15.0, 15.0};

    const auto names = this->declare_parameter<std::vector<std::string>>("rotor_names", default_names);
    const auto positions = this->declare_parameter<std::vector<double>>("rotor_positions_body", default_positions);
    const auto yaw_signs = this->declare_parameter<std::vector<double>>("rotor_yaw_torque_signs", default_yaw_signs);
    const auto thrust_min = this->declare_parameter<std::vector<double>>("rotor_thrust_min", default_min);
    const auto thrust_max = this->declare_parameter<std::vector<double>>("rotor_thrust_max", default_max);

    const std::size_t n = names.size();
    if (n == 0 || positions.size() != 3 * n || yaw_signs.size() != n ||
        thrust_min.size() != n || thrust_max.size() != n) {
      RCLCPP_WARN(this->get_logger(),
                  "Rotor parameter size mismatch: names=%zu positions=%zu yaw_signs=%zu min=%zu max=%zu",
                  names.size(), positions.size(), yaw_signs.size(), thrust_min.size(), thrust_max.size());
      return config;
    }

    config.rotors.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      RotorConfig rotor;
      rotor.name = names[i];
      rotor.position_body = {positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]};
      rotor.yaw_torque_sign = yaw_signs[i] >= 0.0 ? 1.0 : -1.0;
      rotor.thrust_min = std::max(0.0, thrust_min[i]);
      rotor.thrust_max = std::max(rotor.thrust_min, thrust_max[i]);
      config.rotors.push_back(rotor);
    }

    return config;
  }

  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    // [추가] 들어온 힘 값이 NaN(숫자가 아님)이면 무시하고 리턴 (방어 코드)
    if (std::isnan(msg->wrench.force.x) || std::isnan(msg->wrench.force.y) || std::isnan(msg->wrench.force.z) ||
        std::isnan(msg->wrench.torque.x) || std::isnan(msg->wrench.torque.y) || std::isnan(msg->wrench.torque.z)) {
        // 디버그용 로그 한번 찍어주기
        // RCLCPP_WARN(this->get_logger(), "Received NaN Wrench! Ignoring...");
        return; 
    }

    // control_node가 /control/wrench에 publish한 힘/토크를
    // sim_node가 받아서 sixdof에 넣을 **Input 구조체(input_)**에 저장하는 역할
    // SharedPtr는 ROS2에서 메시지를 효율적으로 전달하기 위한 스마트 포인터로, 메시지 데이터를 복사하지 않고 참조를 공유할 수 있게 해준다. 
    // 따라서 콜백 함수에서 메시지 데이터를 안전하게 읽을 수 있다.
    // 여기서는 /control/wrench가 "body frame 기준 힘/토크"라고 가정
    // (sixdof도 thrust_body/moment_body를 body frame으로 받도록 설계되어 있음)
    std::lock_guard<std::mutex> lock(mtx_);
    // mutex로 input_에 대한 동시 접근을 방지하여 쓰레드 안전하게 업데이트, RAII 스타일로 lock 관리
    input_.thrust_body = { msg->wrench.force.x,  msg->wrench.force.y,  msg->wrench.force.z };
    input_.moment_body = { msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z };
  }

  void onTimer()
  {
    // 적분(상태 업데이트)
    Input u_copy;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      // mutex로 input_에 대한 동시 접근을 방지하여 쓰레드 안전하게 읽기, RAII 스타일로 lock 관리
      u_copy = input_; // 복사본을 만들어서 뒷부분 적분에 사용
    }

    Input applied_u = u_copy;
    if (actuator_mode_ == "multirotor" || actuator_mode_ == "rpm_propeller") {
      applied_u = multirotor_.update(u_copy, dt_, state_.p.z);
      publishActuatorDebug();
    }

    if (enable_body_drag_) {
      applied_u.thrust_body += computeBodyDragForce(state_);
    }

    // ===== IMU 생성용: 현재 상태에서의 가속도/각속도 계산 =====
    // d.dv: world frame 선형가속도 (중력 포함)
    const Deriv d = derivatives(state_, applied_u, params_);
    const Vec3 gravity{0.0, 0.0, -params_.g};

    Vec3 accel_world_for_imu = d.dv;

    state_ = rk4_step(state_, applied_u, params_, dt_);

    // ===== 지면 충돌(Ground Collision) 방지 로직 =====
    // 시뮬레이터 상에서 드론이 땅(Z=0) 밑으로 뚫고 내려가지 않도록 강제함 (음슴체)
    bool ground_contact = false;
    if (state_.p.z < 0.0) {
        state_.p.z = 0.0;          // 위치를 바닥으로 고정
        if (state_.v.z < 0.0) {
            state_.v.z = 0.0;      // 떨어지던 속도 소멸
        }
        ground_contact = true;
    } else if (state_.p.z <= 1e-6 && state_.v.z <= 1e-6 && accel_world_for_imu.z < 0.0) {
        ground_contact = true;
    }

    // 지면 constraint 때문에 z운동이 막힌 상태에서는 IMU도 자유낙하가 아니라
    // 정지/접촉 상태의 specific force를 내야 한다. 이 보정이 없으면 launch 초반
    // 제어 입력 수신 전 0 thrust 구간을 EKF가 자유낙하로 적분해 /nav/odom z가 크게 내려간다.
    if (ground_contact_imu_correction_ && ground_contact && accel_world_for_imu.z < 0.0) {
        accel_world_for_imu.z = 0.0;
    }

    // specific force = a_world - g_world (world) 를 body로 회전, f = a - g 이 후 body frame으로 변환함.
    // "specific force"는 가속도에서 중력 가속도를 뺀 값으로, IMU의 가속도계가 측정하는 실제 가속도임.
    const Vec3 specific_force_world = accel_world_for_imu - gravity;
    const Vec3 accel_body = state_.q.rotateWorldToBody(specific_force_world);

    updateImuBias();

    // Odometry publish
    nav_msgs::msg::Odometry odom; // 드론/로봇의 상태를 나타내는 표준 메시지 타입
    odom.header.stamp = this->now(); // 메시지가 생성된 시간, stamp를 통해 time synchronization 가능
    odom.header.frame_id = "world"; // world 좌표계 기준
    odom.child_frame_id = "base_link"; // 드론/로봇 본체 프레임

    odom.pose.pose.position.x = state_.p.x;
    odom.pose.pose.position.y = state_.p.y;
    odom.pose.pose.position.z = state_.p.z;

    odom.pose.pose.orientation.w = state_.q.w;
    odom.pose.pose.orientation.x = state_.q.x;
    odom.pose.pose.orientation.y = state_.q.y;
    odom.pose.pose.orientation.z = state_.q.z;

    // 여기서 state_.v는 world frame 속도, state_.w는 body frame 각속도
    // 일단은 그대로 넣고, 나중에 frame 정교화(또는 tf)로 다듬기로
    odom.twist.twist.linear.x = state_.v.x;
    odom.twist.twist.linear.y = state_.v.y;
    odom.twist.twist.linear.z = state_.v.z;

    odom.twist.twist.angular.x = state_.w.x;
    odom.twist.twist.angular.y = state_.w.y;
    odom.twist.twist.angular.z = state_.w.z;

    odom_pub_->publish(odom); // 퍼블리셔로 메시지 발행하여 ROS 네트워크에 전파

    if (state_log_enabled_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), state_log_period_ms_,
        "[Sim State] p=(%.2f, %.2f, %.2f)m v=(%.2f, %.2f, %.2f)m/s w_body=(%.2f, %.2f, %.2f)rad/s input_Fz=%.2fN input_M=(%.3f, %.3f, %.3f)Nm mode=%s",
        state_.p.x, state_.p.y, state_.p.z,
        state_.v.x, state_.v.y, state_.v.z,
        state_.w.x, state_.w.y, state_.w.z,
        applied_u.thrust_body.z,
        applied_u.moment_body.x, applied_u.moment_body.y, applied_u.moment_body.z,
        actuator_mode_.c_str());
    }

    // ===== IMU publish (100 Hz = every step) =====
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = odom.header.stamp;
    imu.header.frame_id = "base_link";

    // Orientation은 일단 GT를 넣어도 되고(나중에 EKF랑 비교 가능), 안 써도 됨
    // Orientation은 쿼터니언 그대로 (보통 AHRS가 처리한다고 가정하나, 여기도 노이즈 넣을 수 있음, 일단 그대로)
    imu.orientation.w = state_.q.w;
    imu.orientation.x = state_.q.x;
    imu.orientation.y = state_.q.y;
    imu.orientation.z = state_.q.z;

    // // Gyro
    // imu.angular_velocity.x = state_.w.x;
    // imu.angular_velocity.y = state_.w.y;
    // imu.angular_velocity.z = state_.w.z;
    // // Accel
    // imu.linear_acceleration.x = accel_body.x;
    // imu.linear_acceleration.y = accel_body.y;
    // imu.linear_acceleration.z = accel_body.z;
    const Vec3 gyro_bias = enable_imu_bias_ ? gyro_bias_body_ : Vec3{0.0, 0.0, 0.0};
    const Vec3 accel_bias = enable_imu_bias_ ? accel_bias_body_ : Vec3{0.0, 0.0, 0.0};

    // IMU = true body-frame measurement + slowly varying bias + white noise.
    imu.angular_velocity.x = state_.w.x + gyro_bias.x + dist_gyr_(generator_);
    imu.angular_velocity.y = state_.w.y + gyro_bias.y + dist_gyr_(generator_);
    imu.angular_velocity.z = state_.w.z + gyro_bias.z + dist_gyr_(generator_);
    imu.linear_acceleration.x = accel_body.x + accel_bias.x + dist_acc_(generator_);
    imu.linear_acceleration.y = accel_body.y + accel_bias.y + dist_acc_(generator_);
    imu.linear_acceleration.z = accel_body.z + accel_bias.z + dist_acc_(generator_);

    imu_pub_->publish(imu);

    step_count_++; // ===== GPS publish (Noise 추가) =====
    if (step_count_ % gps_div_ == 0) {
      geometry_msgs::msg::PointStamped gps;
      gps.header.stamp = odom.header.stamp;
      gps.header.frame_id = "world";
      
      // GPS Position + Noise
      gps.point.x = state_.p.x + dist_gps_(generator_);
      gps.point.y = state_.p.y + dist_gps_(generator_);
      gps.point.z = state_.p.z + dist_gps_(generator_); // 수직 오차도 동일하다고 가정
      
      gps_pub_->publish(gps);
    }
  }

  void publishActuatorDebug()
  {
    const ActuatorDebug& debug = multirotor_.debug();
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(debug.thrust_cmd.size() + debug.thrust_actual.size() +
                     debug.omega_cmd.size() + debug.omega_actual.size());
    msg.data.insert(msg.data.end(), debug.thrust_cmd.begin(), debug.thrust_cmd.end());
    msg.data.insert(msg.data.end(), debug.thrust_actual.begin(), debug.thrust_actual.end());
    msg.data.insert(msg.data.end(), debug.omega_cmd.begin(), debug.omega_cmd.end());
    msg.data.insert(msg.data.end(), debug.omega_actual.begin(), debug.omega_actual.end());
    actuator_pub_->publish(msg);
  }

  Vec3 computeBodyDragForce(const State& state) const
  {
    const Vec3 v_body = state.q.rotateWorldToBody(state.v);
    return {
      -body_drag_linear_.x * v_body.x - body_drag_quadratic_.x * std::abs(v_body.x) * v_body.x,
      -body_drag_linear_.y * v_body.y - body_drag_quadratic_.y * std::abs(v_body.y) * v_body.y,
      -body_drag_linear_.z * v_body.z - body_drag_quadratic_.z * std::abs(v_body.z) * v_body.z
    };
  }

  void updateImuBias()
  {
    if (!enable_imu_bias_) {
      return;
    }

    const double sqrt_dt = std::sqrt(std::max(dt_, 0.0));
    if (accel_bias_random_walk_std_ > 0.0) {
      accel_bias_body_.x += accel_bias_random_walk_std_ * sqrt_dt * dist_unit_(generator_);
      accel_bias_body_.y += accel_bias_random_walk_std_ * sqrt_dt * dist_unit_(generator_);
      accel_bias_body_.z += accel_bias_random_walk_std_ * sqrt_dt * dist_unit_(generator_);
    }
    if (gyro_bias_random_walk_std_ > 0.0) {
      gyro_bias_body_.x += gyro_bias_random_walk_std_ * sqrt_dt * dist_unit_(generator_);
      gyro_bias_body_.y += gyro_bias_random_walk_std_ * sqrt_dt * dist_unit_(generator_);
      gyro_bias_body_.z += gyro_bias_random_walk_std_ * sqrt_dt * dist_unit_(generator_);
    }
  }

private:
  double dt_{0.01};

  Params params_;
  State  state_;
  Input  input_;
  std::string actuator_mode_{"direct_wrench"};
  bool ground_contact_imu_correction_{true};
  bool state_log_enabled_{true};
  int state_log_period_ms_{1000};
  bool enable_body_drag_{false};
  Vec3 body_drag_linear_{0.0, 0.0, 0.0};
  Vec3 body_drag_quadratic_{0.0, 0.0, 0.0};
  bool enable_imu_bias_{false};
  Vec3 accel_bias_body_{0.0, 0.0, 0.0};
  Vec3 gyro_bias_body_{0.0, 0.0, 0.0};
  double accel_bias_random_walk_std_{0.0};
  double gyro_bias_random_walk_std_{0.0};
  MultirotorModel multirotor_;

  std::mutex mtx_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr gps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actuator_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int gps_div_{10}; // GPS는 10Hz로 발행하기 위해 타이머마다 카운트
  int step_count_{0}; // 타이머 콜백이 몇 번 호출되었는지 카운트
  // <random> 라이브러리를 이용하여 정규 분포(가우시안 분포)를 따르는 난수를 생성
  std::default_random_engine generator_;
  std::normal_distribution<double> dist_acc_;
  std::normal_distribution<double> dist_gyr_;
  std::normal_distribution<double> dist_gps_;
  std::normal_distribution<double> dist_unit_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimNode>());
  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("dummy_node");
//   // basic pub test
//   auto pub = node->create_publisher<std_msgs::msg::String>("/dummy/chatter", 10);
//   // basic sub test
//   auto sub = node->create_subscription<std_msgs::msg::String>(
//   "/dummy/in", 10,
//   [node](const std_msgs::msg::String::SharedPtr msg) {
//     RCLCPP_INFO(node->get_logger(), "received: %s", msg->data.c_str());
//   });

//   auto timer = node->create_wall_timer(500ms, [node, pub]() {
//     std_msgs::msg::String msg;
//     msg.data = "hello from dummy_node";
//     pub->publish(msg);
//     // RCLCPP_INFO(node->get_logger(), "publish: %s", msg.data.c_str());
//   });

//   RCLCPP_INFO(node->get_logger(), "dummy_node started");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
