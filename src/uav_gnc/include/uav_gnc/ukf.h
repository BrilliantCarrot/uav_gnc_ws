#pragma once

#include <Eigen/Dense>
#include <iostream>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

// ============================================================
// UKF (Unscented Kalman Filter) — Error-State 구조
//
// [EKF와의 핵심 차이]
//   EKF : 비선형 함수 f(x)를 야코비안 F로 선형 근사하여 공분산 전파
//         P_new = F * P * F^T + Q
//   UKF : 야코비안 없이 "시그마 포인트"를 비선형 함수에 직접 통과시켜
//         공분산 전파 → 2차 정확도 보장, 선형화 오차 없음
//
// [상태 벡터 구조] — EKF와 동일하게 유지 (비교 실험 목적)
//   x_ [0~2]  : Position  (월드 위치, 실제값)
//   x_ [3~5]  : Velocity  (월드 속도, 실제값)
//   x_ [6~8]  : Attitude Error (자세 오차, 업데이트 후 0으로 리셋)
//   x_ [9~11] : Accel Bias
//   x_[12~14] : Gyro Bias
//   q_         : 명목 자세 쿼터니언 (별도 관리)
//
// [시그마 포인트 (Sigma Points)]
//   n=15차원 → 2n+1=31개의 시그마 포인트 생성
//   각 시그마 포인트가 비선형 동역학을 직접 통과하며 공분산을 전파
//   → 자세-가속도 비선형 결합을 EKF보다 정확하게 포착
// ============================================================

class UKF {
public:
    UKF();
    ~UKF();

    // 초기화 — EKF와 동일한 인터페이스
    void init(const Vector3d& p, const Vector3d& v, const Eigen::Quaterniond& q);

    // 예측 단계: IMU 데이터로 시그마 포인트 전파 → 공분산 갱신
    void predict(const Vector3d& acc, const Vector3d& gyro, double dt);

    // 보정 단계: GPS 위치로 상태 보정 (H가 선형이므로 EKF와 수식 동일)
    void update_gps(const Vector3d& meas_pos);

    // Getter — EKF와 동일한 인터페이스 (navigation_node가 구분 없이 호출 가능)
    Vector3d          getPosition()   const { return x_.segment<3>(0); }
    Vector3d          getVelocity()   const { return x_.segment<3>(3); }
    Eigen::Quaterniond getAttitude()  const { return q_; }
    Vector3d          getAccelBias()  const { return x_.segment<3>(9);  }
    Vector3d          getGyroBias()   const { return x_.segment<3>(12); }

private:
    // ── Scaled Unscented Transform 파라미터 ──────────────────────
    // n     : 상태 차원 (15)
    // alpha : 시그마 포인트 확산 범위 (0 < alpha ≤ 1)
    //         작을수록 평균 근처에 집중 / 15차원에서는 0.3이 안정적
    // beta  : 분포 선험 지식 (가우시안이면 2.0이 최적)
    // kappa : 보조 스케일링 (보통 0 또는 3-n)
    // lambda = alpha^2 * (n + kappa) - n
    static constexpr int    n_     = 15;
    static constexpr double alpha_ = 0.3;
    static constexpr double beta_  = 2.0;
    static constexpr double kappa_ = 0.0;

    // 가중치 (생성자에서 lambda로부터 계산)
    double lambda_;  // 스케일링 파라미터
    double Wm0_;     // 평균용 0번 가중치
    double Wc0_;     // 공분산용 0번 가중치 (beta 반영)
    double Wmi_;     // 평균용 i번 가중치 (i=1..2n, 모두 동일)
    double Wci_;     // 공분산용 i번 가중치 (i=1..2n, 모두 동일)

    // ── 상태 (EKF와 동일한 구조) ─────────────────────────────────
    VectorXd           x_;   // 15차원 상태 벡터
    MatrixXd           P_;   // 15×15 오차 공분산
    Eigen::Quaterniond q_;   // 명목 자세 쿼터니언

    // ── 노이즈 행렬 ───────────────────────────────────────────────
    MatrixXd Q_;      // 프로세스 노이즈 공분산 (15×15)
    MatrixXd R_gps_;  // GPS 측정 노이즈 공분산 (3×3)

    const double g_ = 9.80665;

    // ── 내부 헬퍼 함수 ────────────────────────────────────────────

    // 하나의 오차 상태 시그마 포인트를 비선형 동역학으로 전파
    // delta_x      : 15D 오차 상태 시그마 포인트 (입력)
    // pos/vel/q/ba/bg _nom : 현재 스텝의 명목 상태
    // pos/vel/q _nom_new   : 미리 계산된 전파 후 명목 상태 (효율화)
    // 반환값       : 전파 후 15D 오차 상태
    VectorXd propagateSigmaPoint(
        const VectorXd&        delta_x,
        const Vector3d&        pos_nom,
        const Vector3d&        vel_nom,
        const Eigen::Quaterniond& q_nom,
        const Vector3d&        ba_nom,
        const Vector3d&        bg_nom,
        const Vector3d&        acc_meas,
        const Vector3d&        gyro_meas,
        double                 dt,
        const Vector3d&        pos_nom_new,
        const Vector3d&        vel_nom_new,
        const Eigen::Quaterniond& q_nom_new) const;
};
