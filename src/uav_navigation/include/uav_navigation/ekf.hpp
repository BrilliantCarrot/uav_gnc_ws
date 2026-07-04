#pragma once

#include <Eigen/Dense>
#include <iostream>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class EKF {
public:
    EKF();
    ~EKF();
    // 초기화: 위치, 속도, 쿼터니언(w,x,y,z)
    void init(const Vector3d& p, const Vector3d& v, const Eigen::Quaterniond& q);
    // 예측 단계 (IMU Data): dt 시간만큼 상태 적분 및 공분산 전파
    // acc: 가속도계 측정값 (m/s^2), gyro: 자이로 측정값 (rad/s)
    // IMU 가속도/각속도를 입력(u)으로 사용하여 상태를 적분
    void predict(const Vector3d& acc, const Vector3d& gyro, double dt);
    // 보정 단계 (GPS Data): GPS 위치 데이터를 측정치(z)로 사용하여 오차를 수정
    // meas_pos: GPS 위치 (x, y, z)
    void update_gps(const Vector3d& meas_pos);
    void update_lidar_pose(const Vector3d& meas_pos, double meas_yaw);
    // Getter
    Vector3d getPosition() const { return x_.segment<3>(0); }
    Vector3d getVelocity() const { return x_.segment<3>(3); }
    // 쿼터니언은 상태 벡터 외부에 따로 관리하거나, 상태 벡터 안에 포함시킬 수 있음.
    // 현재 코드에서는 편의상 "Error State EKF" 구조를 차용하여, 
    // 명목 상태(Nominal State: p, v, q) + 오차 상태(Error State: dp, dv, dtheta, db_a, db_g) 구조를 씀.
    // 하지만 코드를 직관적으로 만들기 위해, 여기서는 "Direct EKF" 형태로 쿼터니언을 직접 업데이트하되,
    // 쿼터니언 정규화를 계속 해주는 방식을 사용함.
    Eigen::Quaterniond getAttitude() const { return q_; }
    Vector3d getAccelBias() const { return x_.segment<3>(9); } // index 9~11
    Vector3d getGyroBias() const { return x_.segment<3>(12); } // index 12~14

private:
    // 전체 상태 벡터 x_ (15차원)
    // 0~2: Pos (px, py, pz)
    // 3~5: Vel (vx, vy, vz)
    // 6~8: Att Error (roll, pitch, yaw) -- 실제 자세는 q_ 변수에 저장
    // 9~11: Accel Bias (b_ax, b_ay, b_az)
    // 12~14: Gyro Bias (b_gx, b_gy, b_gz)
    VectorXd x_; 
    // 오차 공분산 행렬 P_ (15x15)
    MatrixXd P_;
    // 명목 상태(Nominal State)의 자세 (Quaternion)
    Eigen::Quaterniond q_;
    // 파라미터: 노이즈 공분산
    MatrixXd Q_; // Process Noise Covariance 시스템 노이즈 행렬
    MatrixXd R_gps_; // Measurement Noise Covariance (GPS) 측정 노이즈 행렬
    MatrixXd R_lidar_pose_;
    // 상수
    const double g_ = 9.80665;
};