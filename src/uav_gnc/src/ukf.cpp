#include "uav_gnc/ukf.h"

// ============================================================
// UKF (Unscented Kalman Filter) 구현
//
// [핵심 알고리즘 흐름]
//   predict():
//     ① 명목 상태(pos, vel, q_)를 비선형 적분 (EKF와 동일)
//     ② 현재 공분산 P에서 2n+1=31개 시그마 포인트 생성
//     ③ 각 시그마 포인트(오차 상태 섭동)를 비선형 동역학으로 전파
//     ④ 전파된 시그마 포인트로 가중 평균/공분산 재구성 → P_new
//        (이 단계에서 EKF 야코비안 근사 없이 비선형성이 반영됨)
//
//   update_gps():
//     H가 선형(위치만 추출)이므로 EKF와 수식 동일
//     y = z - Hx,  K = P H^T (HPH^T + R)^{-1},  x += K*y
//
// [EKF 대비 UKF 장점이 드러나는 상황]
//   - 드론이 큰 각도로 기울어질 때 (roll/pitch > 15°)
//     → 가속도-자세 비선형 결합을 F 야코비안 없이 정확하게 전파
//   - 고속 선회, 급기동 등 비선형성이 강한 기동
// ============================================================

UKF::UKF() {
    // ── 시그마 포인트 가중치 계산 ────────────────────────────────
    // λ = α²(n+κ) − n
    // n=15, α=0.3, κ=0 → λ = 0.09*15 − 15 = −13.65
    // Wm₀ = λ/(n+λ),  Wc₀ = Wm₀ + (1−α²+β),  Wmi=Wci = 1/(2(n+λ))
    // 주의: n이 크면 λ < 0이 되어 Wm₀ < 0 이 될 수 있음
    //       이는 Scaled UKF에서 정상적이며, 가중합이 1을 만족하면 올바름
    lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;
    Wm0_ = lambda_ / (n_ + lambda_);
    Wc0_ = Wm0_ + (1.0 - alpha_ * alpha_ + beta_);
    Wmi_ = 1.0 / (2.0 * (n_ + lambda_));
    Wci_ = Wmi_;

    // ── 상태/공분산 초기화 ───────────────────────────────────────
    x_ = VectorXd::Zero(15);
    q_ = Eigen::Quaterniond(1, 0, 0, 0);

    // 초기 공분산 P (EKF와 동일한 값, 비교 공정성 확보)
    P_ = MatrixXd::Identity(15, 15);
    P_.block<3, 3>(0,  0)  *= 1.0;    // Pos
    P_.block<3, 3>(3,  3)  *= 0.1;    // Vel
    P_.block<3, 3>(6,  6)  *= 0.01;   // Att error
    P_.block<3, 3>(9,  9)  *= 0.01;   // Acc Bias
    P_.block<3, 3>(12, 12) *= 0.001;  // Gyro Bias

    // 프로세스 노이즈 Q (EKF와 동일)
    Q_ = MatrixXd::Identity(15, 15);
    Q_.block<3, 3>(0,  0)  *= 0.0;    // Pos (적분 결과, 직접 노이즈 없음)
    Q_.block<3, 3>(3,  3)  *= 0.01;   // Vel noise
    Q_.block<3, 3>(6,  6)  *= 0.001;  // Att noise
    Q_.block<3, 3>(9,  9)  *= 1e-5;   // Acc Bias random walk
    Q_.block<3, 3>(12, 12) *= 1e-6;   // Gyro Bias random walk

    // GPS 측정 노이즈 R (EKF와 동일, 비교 실험 공정성)
    R_gps_ = MatrixXd::Identity(3, 3) * 30.0;
}

UKF::~UKF() {}

void UKF::init(const Vector3d& p, const Vector3d& v, const Eigen::Quaterniond& q) {
    x_.setZero();
    x_.segment<3>(0) = p;
    x_.segment<3>(3) = v;
    q_ = q;
    q_.normalize();
}

// ============================================================
// predict(): Unscented Transform 기반 예측 단계
// ============================================================
void UKF::predict(const Vector3d& acc_meas, const Vector3d& gyro_meas, double dt) {
    if (dt <= 0.0 || dt > 1.0) {
        std::cout << "[UKF Warning] Bad dt: " << dt << std::endl;
        return;
    }

    // ── STEP 1: 현재 명목 상태 추출 ─────────────────────────────
    const Vector3d          pos_nom = x_.segment<3>(0);
    const Vector3d          vel_nom = x_.segment<3>(3);
    const Vector3d          ba_nom  = x_.segment<3>(9);
    const Vector3d          bg_nom  = x_.segment<3>(12);
    const Eigen::Quaterniond q_nom  = q_;

    // ── STEP 2: 명목 상태 비선형 전파 (한 번만 계산) ─────────────
    // 바이어스 보정
    const Vector3d acc_ub_nom  = acc_meas  - ba_nom;
    const Vector3d gyro_ub_nom = gyro_meas - bg_nom;

    // 위치/속도 전파 (뉴턴 운동 법칙)
    const Vector3d acc_world_nom = q_nom * acc_ub_nom - Vector3d(0, 0, g_);
    const Vector3d pos_nom_new   = pos_nom + vel_nom * dt + 0.5 * acc_world_nom * dt * dt;
    const Vector3d vel_nom_new   = vel_nom + acc_world_nom * dt;

    // 자세 전파 (소각 근사 쿼터니언 적분)
    Eigen::Quaterniond dq_nom;
    dq_nom.w()   = 1.0;
    dq_nom.vec() = gyro_ub_nom * dt * 0.5;
    Eigen::Quaterniond q_nom_new = q_nom * dq_nom;
    q_nom_new.normalize();

    // ── STEP 3: 시그마 포인트 생성 ──────────────────────────────
    // (n+λ)*P의 촐레스키 분해: sqrt_P = chol((n+λ)P) 하삼각 행렬
    // X_0     = 0 벡터 (명목 상태 자체, 오차=0)
    // X_i     = +sqrt_P의 i번째 열 (i=1..n)
    // X_{n+i} = -sqrt_P의 i번째 열 (i=1..n)
    const MatrixXd scaled_P = (n_ + lambda_) * P_;
    Eigen::LLT<MatrixXd> llt(scaled_P);

    // 촐레스키 분해 실패 시 (P가 양정치 행렬이 아닌 경우) 정규화 후 재시도
    if (llt.info() != Eigen::Success) {
        // 소규모 정규화를 더해 양정치 강제
        Eigen::LLT<MatrixXd> llt2(scaled_P + 1e-6 * MatrixXd::Identity(n_, n_));
        if (llt2.info() != Eigen::Success) {
            std::cout << "[UKF] Cholesky failed — skipping predict step" << std::endl;
            return;
        }
        const MatrixXd sqrt_P = llt2.matrixL();
        // 재시도 성공 시 아래 sigma point 생성을 위해 지역 변수 사용
        const int num_sigma = 2 * n_ + 1;
        MatrixXd sigma_pts  = MatrixXd::Zero(n_, num_sigma);
        for (int i = 0; i < n_; ++i) {
            sigma_pts.col(i + 1)      =  sqrt_P.col(i);
            sigma_pts.col(i + 1 + n_) = -sqrt_P.col(i);
        }

        // 시그마 포인트 전파
        MatrixXd prop_pts = MatrixXd::Zero(n_, num_sigma);
        for (int j = 0; j < num_sigma; ++j) {
            prop_pts.col(j) = propagateSigmaPoint(
                sigma_pts.col(j),
                pos_nom, vel_nom, q_nom, ba_nom, bg_nom,
                acc_meas, gyro_meas, dt,
                pos_nom_new, vel_nom_new, q_nom_new);
        }

        // 가중 평균/공분산
        VectorXd x_mean = Wm0_ * prop_pts.col(0);
        for (int j = 1; j < num_sigma; ++j) x_mean += Wmi_ * prop_pts.col(j);

        P_ = MatrixXd::Zero(n_, n_);
        VectorXd d0 = prop_pts.col(0) - x_mean;
        P_ += Wc0_ * d0 * d0.transpose();
        for (int j = 1; j < num_sigma; ++j) {
            VectorXd d = prop_pts.col(j) - x_mean;
            P_ += Wci_ * d * d.transpose();
        }
        P_ += Q_;

        x_.segment<3>(0) = pos_nom_new;
        x_.segment<3>(3) = vel_nom_new;
        x_.segment<3>(9)  = ba_nom;
        x_.segment<3>(12) = bg_nom;
        q_ = q_nom_new;
        return;
    }

    const MatrixXd sqrt_P = llt.matrixL();  // 하삼각 행렬 L, P = L*L^T

    // 시그마 포인트 행렬 (15 × 31)
    const int num_sigma = 2 * n_ + 1;
    MatrixXd sigma_pts  = MatrixXd::Zero(n_, num_sigma);
    // X_0 = 0벡터 (col(0)은 초기화 시 이미 0)
    for (int i = 0; i < n_; ++i) {
        sigma_pts.col(i + 1)      =  sqrt_P.col(i);  // 양의 방향 섭동
        sigma_pts.col(i + 1 + n_) = -sqrt_P.col(i);  // 음의 방향 섭동
    }

    // ── STEP 4: 각 시그마 포인트를 비선형 동역학으로 전파 ─────────
    // 핵심: EKF는 F 야코비안으로 선형 근사, UKF는 실제 비선형 함수 통과
    // → 자세 섭동(dθ)이 가속도 방향을 회전시켜 속도/위치에 주는
    //   비선형 영향을 F 없이 정확하게 포착함
    MatrixXd prop_pts = MatrixXd::Zero(n_, num_sigma);
    for (int j = 0; j < num_sigma; ++j) {
        prop_pts.col(j) = propagateSigmaPoint(
            sigma_pts.col(j),
            pos_nom, vel_nom, q_nom, ba_nom, bg_nom,
            acc_meas, gyro_meas, dt,
            pos_nom_new, vel_nom_new, q_nom_new);
    }

    // ── STEP 5: 전파된 시그마 포인트로 가중 평균 계산 ────────────
    // 오차 상태 공간이므로 이상적으로는 ≈ 0,
    // 비선형성으로 인해 미세하게 0이 아닐 수 있음
    VectorXd x_mean = Wm0_ * prop_pts.col(0);
    for (int j = 1; j < num_sigma; ++j) {
        x_mean += Wmi_ * prop_pts.col(j);
    }

    // ── STEP 6: 전파된 공분산 계산 (Unscented Transform 핵심) ─────
    // P_new = Σ Wc_j * (Y_j - ȳ)(Y_j - ȳ)^T + Q
    MatrixXd P_new = MatrixXd::Zero(n_, n_);
    {
        VectorXd d0 = prop_pts.col(0) - x_mean;
        P_new += Wc0_ * d0 * d0.transpose();  // 0번 시그마 포인트 기여
    }
    for (int j = 1; j < num_sigma; ++j) {
        VectorXd d = prop_pts.col(j) - x_mean;
        P_new += Wci_ * d * d.transpose();    // i번 시그마 포인트 기여
    }
    P_new += Q_;  // 프로세스 노이즈 추가

    // ── STEP 7: 상태 업데이트 ───────────────────────────────────
    x_.segment<3>(0)  = pos_nom_new;           // 명목 위치 (전파됨)
    x_.segment<3>(3)  = vel_nom_new;           // 명목 속도 (전파됨)
    x_.segment<3>(6)  = x_mean.segment<3>(6); // 자세 오차 (≈0, 보정 전까지 대기)
    x_.segment<3>(9)  = ba_nom;               // 가속도 바이어스 (random walk, 변화 없음)
    x_.segment<3>(12) = bg_nom;               // 자이로 바이어스 (random walk, 변화 없음)
    q_  = q_nom_new;
    P_  = P_new;

    // NaN 체크
    if (x_.hasNaN() || P_.hasNaN()) {
        std::cout << "[UKF FAIL] State/Cov is NaN at dt=" << dt << std::endl;
    }
}

// ============================================================
// update_gps(): GPS 위치 보정 단계
// H가 선형 (위치 3개만 추출)이므로 EKF와 수식 동일
// 시그마 포인트 기반 업데이트를 쓰면 계산량만 늘고 결과는 같음
// ============================================================
void UKF::update_gps(const Vector3d& meas_pos) {
    // 관측 행렬: H = [I₃ | 0 ... 0] (3×15)
    // GPS는 위치(x_[0:3])만 측정함
    MatrixXd H = MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 0) = Matrix3d::Identity();

    // 칼만 이득: K = P H^T (H P H^T + R)^{-1}
    MatrixXd PHt = P_ * H.transpose();
    MatrixXd S   = H * PHt + R_gps_;  // Innovation Covariance
    MatrixXd K   = PHt * S.inverse();

    // 잔차: y = z - H*x (GPS 위치 - 예측 위치)
    const Vector3d pred_pos = x_.segment<3>(0);
    const Vector3d y        = meas_pos - pred_pos;

    // 상태 보정: x += K*y
    VectorXd dx = K * y;
    x_.segment<3>(0)  += dx.segment<3>(0);   // Pos 보정
    x_.segment<3>(3)  += dx.segment<3>(3);   // Vel 보정
    x_.segment<3>(9)  += dx.segment<3>(9);   // Acc Bias 보정
    x_.segment<3>(12) += dx.segment<3>(12);  // Gyro Bias 보정

    // 자세 오차 주입 (Error Quaternion Injection, EKF와 동일)
    // dθ = dx[6:9] → dq = [1, 0.5*dθ] → q_ = q_ ⊗ dq
    const Vector3d dtheta = dx.segment<3>(6);
    Eigen::Quaterniond dq;
    dq.w()   = 1.0;
    dq.vec() = 0.5 * dtheta;
    q_ = q_ * dq;
    q_.normalize();

    // 공분산 업데이트: P = (I - K H) P
    const MatrixXd I = MatrixXd::Identity(15, 15);
    P_ = (I - K * H) * P_;
}

// ============================================================
// propagateSigmaPoint(): 오차 상태 시그마 포인트 비선형 전파
//
// [입력]
//   delta_x   : 15D 오차 상태 (명목 상태로부터의 섭동)
//   pos/vel/q/ba/bg _nom : 현재 명목 상태
//   acc_meas, gyro_meas  : IMU 측정값
//   dt                   : 적분 시간
//   pos/vel/q _nom_new   : 미리 계산된 명목 전파 결과 (매 호출마다 중복 계산 방지)
//
// [출력]
//   전파된 15D 오차 상태 = (섭동 상태 전파 결과) − (명목 상태 전파 결과)
//
// [UKF의 핵심이 여기 있음]
//   EKF는 dv/dθ = -R*[a]× *dt 야코비안으로 자세 섭동→속도 변화를 선형 근사
//   UKF는 q_pert = q_nom ⊗ dq(dθ)로 실제 자세를 만들어서 완전 비선형 전파
//   → 큰 tilt 각도에서 EKF보다 정확한 공분산 전파
// ============================================================
VectorXd UKF::propagateSigmaPoint(
    const VectorXd&           delta_x,
    const Vector3d&           pos_nom,
    const Vector3d&           vel_nom,
    const Eigen::Quaterniond& q_nom,
    const Vector3d&           ba_nom,
    const Vector3d&           bg_nom,
    const Vector3d&           acc_meas,
    const Vector3d&           gyro_meas,
    double                    dt,
    const Vector3d&           pos_nom_new,
    const Vector3d&           vel_nom_new,
    const Eigen::Quaterniond& q_nom_new) const
{
    // ── 1. 섭동 상태 생성 (명목 상태 + 오차 섭동) ───────────────
    const Vector3d pos_pert = pos_nom + delta_x.segment<3>(0);
    const Vector3d vel_pert = vel_nom + delta_x.segment<3>(3);
    const Vector3d ba_pert  = ba_nom  + delta_x.segment<3>(9);
    const Vector3d bg_pert  = bg_nom  + delta_x.segment<3>(12);

    // 자세 섭동: q_pert = q_nom ⊗ dq(0.5*dθ)
    // 소각 근사로 3D 자세 오차 벡터 → 4D 쿼터니언 변환
    Eigen::Quaterniond dq_pert;
    dq_pert.w()   = 1.0;
    dq_pert.vec() = 0.5 * delta_x.segment<3>(6);
    Eigen::Quaterniond q_pert = q_nom * dq_pert;
    q_pert.normalize();

    // ── 2. 섭동 상태를 비선형 동역학으로 전파 ────────────────────
    // 바이어스 보정 (섭동된 바이어스 사용)
    const Vector3d acc_ub_pert  = acc_meas  - ba_pert;
    const Vector3d gyro_ub_pert = gyro_meas - bg_pert;

    // 위치/속도 전파 (섭동된 자세 q_pert 사용 — 비선형 핵심!)
    // EKF의 F.block<3,3>(3,6) = -R*[a]×*dt 선형 근사 대신
    // 섭동 q_pert로 직접 월드 가속도를 계산함
    const Vector3d acc_world_pert = q_pert * acc_ub_pert - Vector3d(0, 0, g_);
    const Vector3d pos_pert_new   = pos_pert + vel_pert * dt + 0.5 * acc_world_pert * dt * dt;
    const Vector3d vel_pert_new   = vel_pert + acc_world_pert * dt;

    // 자세 전파 (섭동된 자이로 사용)
    Eigen::Quaterniond dq_int_pert;
    dq_int_pert.w()   = 1.0;
    dq_int_pert.vec() = gyro_ub_pert * dt * 0.5;
    Eigen::Quaterniond q_pert_new = q_pert * dq_int_pert;
    q_pert_new.normalize();

    // ── 3. 전파 후 오차 상태 계산: 섭동 결과 − 명목 결과 ─────────
    VectorXd delta_x_new(n_);

    // 위치 오차 (선형 연산이라 선형/비선형 차이 거의 없음)
    delta_x_new.segment<3>(0) = pos_pert_new - pos_nom_new;

    // 속도 오차 (자세 섭동의 비선형 효과가 여기서 드러남)
    delta_x_new.segment<3>(3) = vel_pert_new - vel_nom_new;

    // 자세 오차: δθ = 2 * vec(q_nom_new^{-1} ⊗ q_pert_new)
    // 두 쿼터니언의 차이를 3D 회전 벡터로 변환
    Eigen::Quaterniond dq_err = q_nom_new.inverse() * q_pert_new;
    // 쿼터니언 부호 모호성 제거 (w >= 0 보장)
    if (dq_err.w() < 0.0) { dq_err.coeffs() = -dq_err.coeffs(); }
    delta_x_new.segment<3>(6) = 2.0 * dq_err.vec();

    // 바이어스 오차 (random walk — 전파 후에도 변화 없음)
    delta_x_new.segment<3>(9)  = delta_x.segment<3>(9);   // δba
    delta_x_new.segment<3>(12) = delta_x.segment<3>(12);  // δbg

    return delta_x_new;
}
