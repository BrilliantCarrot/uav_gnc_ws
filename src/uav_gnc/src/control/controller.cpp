#include "uav_gnc/controller.h"
#include <Eigen/Dense>
#include <vector>

// ======================================================================
// controller.cpp
// [1] 공통 헬퍼 함수
// [2] Cascaded PID 제어기 (기존 베이스라인)
// [3] Linear MPC 제어기 (Reference Preview 지원)
// ======================================================================

// 원래 Cascaded PID =====================
// 위치, 속도, 가속도/자세, 모멘트/추력 명령을 만드는 cascaded 컨트롤러.
// 1. 목표 위치(ref.p_ref) 와 현재 위치(s.p) 의 오차로부터
// 2. “어떤 속도로 가야 하는지(v_cmd)”를 만들고
// 3. 그 속도를 만들기 위한 “가속도 명령(a_cmd)”을 만든 뒤
// 4. 그 가속도를 만들기 위해 드론을 “얼마나 기울일지(roll_ref, pitch_ref)”로 바꾸고
// 5. 그 기울기를 따라가게 하는 “모멘트(Mx,My,Mz)”와 “추력(T)”을 계산해서
// 6. 6-DOF 모델(plant)에 입력으로 넣는 제어기 구조.
// =====================

// ── 공통 헬퍼 함수 ────────────────────────────────────────────────────────
// 상한,하한으로 제한하는 함수
static double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// yaw 오차는 179도와 -181도가 사실상 같은 방향인데, 단순 차를 내면 360도 오차로 보임.
// 그래서 각도 오차를 [-π, π] 범위로 제한하여 가장 짧은 방향의 오차로 만드는 함수(최소의 움직임으로 제어).
static double wrap_pi(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// body->world quaternion(q) 기준 ZYX(yaw-pitch-roll) 오일러 추출.
// 6-DOF 상태는 quaternion(s.q)로 자세를 가지고 있음.
// 간단한 자세 제어 PD 구현엔 roll/pitch/yaw가 더 직관적, 따라서 quaternion을 ZYX 순서의 오일러로 변환.
// ※ s.q가 “body→world”인지 “world→body”인지에 따라 부호/해석이 바뀔 수 있음.
void quat_to_euler_zyx(const Quat& q, double& roll, double& pitch, double& yaw) {
    // roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1.0) pitch = std::copysign(M_PI / 2.0, sinp);
    else                         pitch = std::asin(sinp);
    // yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ======================================================================
// [1] 자세 제어 공통 헬퍼 (MPC가 추가되면서 PID와 공유하기 위해 분리)
// 가속도 명령 [ax, ay, az] → thrust + moment
// Step 4~6 로직은 PID 원본과 동일하며 MPC도 이 함수를 재사용함
// ======================================================================
static Input attitude_and_thrust(
    const State& s,
    double ax_cmd, double ay_cmd, double az_cmd,
    double yaw_ref,
    const Params& p, const ControllerGains& g)
{
    Input u;

    double roll, pitch, yaw;
    quat_to_euler_zyx(s.q, roll, pitch, yaw);

    // 가속도 포화 (구동기 보호)
    ax_cmd = clamp(ax_cmd, -g.max_axy_cmd, g.max_axy_cmd);
    ay_cmd = clamp(ay_cmd, -g.max_axy_cmd, g.max_axy_cmd);
    az_cmd = clamp(az_cmd, -g.max_az_cmd,  g.max_az_cmd);

    // =====================
    // 4) 가속도 명령을 기울기로 변환(tilt mapping, 쿼드콥터 드론의 기동을 고려, a_cmd_xy -> roll_ref, pitch_ref)
    // thrust 벡터를 기울여(기체를 기울여) 수평 가속도를 만듦으로 xy 이동.
    // 또한 roll/pitch는 "기체의 앞/옆 방향"과 연관되나 ax_cmd, ay_cmd는 world 좌표(고정 좌표) 기준임.
    // 만약 드론이 yaw로 돌아가 있으면(드론이 현재 회전한 상태라면), "world x 방향으로 가속하라"는 명령을 기체 관점에서 다시 해석됨.
    // 그래서 yaw만 제거한 frame(heading frame)으로 가속도를 바꿈.
    // yaw를 고려한 heading frame로 변환 후 소각 근사.
    // =====================
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // world accel -> heading frame (yaw만 제거한 frame)
    // world에서 본 가속도 벡터(ax, ay)를 드론이 바라보는 방향(heading)에 맞춘 좌표로 돌려놓음.
    // ax_h: 드론 "앞/뒤 방향"으로 필요한 가속도
    // ay_h: 드론 "좌/우 방향"으로 필요한 가속도
    const double ax_h =  cy * ax_cmd + sy * ay_cmd;
    const double ay_h = -sy * ax_cmd + cy * ay_cmd;
    // heading frame의 가속도 명령 -> 기울기 명령 (소각 근사(small angle approximation))
    // 부호는 좌표계 정의에 따라 달라질 수 있음.
    double pitch_ref =  ax_h / p.g;
    double roll_ref  = -ay_h / p.g;
    // 최대 기울기각 제한
    const double max_tilt = g.max_tilt_deg * M_PI / 180.0;
    pitch_ref = clamp(pitch_ref, -max_tilt, max_tilt);
    roll_ref  = clamp(roll_ref,  -max_tilt, max_tilt);
    // yaw는 입력 ref 사용
    const double yaw_ref_ = yaw_ref;

    // =====================
    // 5) Attitude PD -> Moment (간단한 버전)
    // 위 4)의 기울기 명령을 따라가기 위한 모멘트(토크) 명령 생성.
    // yaw는 항상 짧은 방향의 오차가 되도록 함.
    // =====================
    const double e_roll  = roll_ref  - roll;
    const double e_pitch = pitch_ref - pitch;
    const double e_yaw   = wrap_pi(yaw_ref_ - yaw);

    // body rates D항(s.w: body frame)
    const double p_rate = s.w.x;
    const double q_rate = s.w.y;
    const double r_rate = s.w.z;
    // PD 모멘트 명령.
    // P항: 목표 각도(roll_ref/pitch_ref/yaw_ref)로 맞추려는 힘.
    // D항: 회전 속도를 감쇠시켜 진동/오버슈트를 줄임.
    const double Mx = g.kp_att_rp  * e_roll  - g.kd_att_rp  * p_rate;
    const double My = g.kp_att_rp  * e_pitch - g.kd_att_rp  * q_rate;
    const double Mz = g.kp_att_yaw * e_yaw   - g.kd_att_yaw * r_rate;
    // body 기준 roll, pitch, yaw 모멘트 명령, 6-DOF 모델에 입력으로 사용.
    u.moment_body = {Mx, My, Mz};

    // =====================
    // 6) 추력 계산 + 기울기 보상(tilt compensation)
    // 드론이 기울어지면 추력 벡터는 body z축 방향으로 나가고 world z 방향 성분은 줄어듦.
    // 따라서 그만큼 추력을 더 키워줘야 고도 유지.
    // Thrust는 z-up world 가정.
    // =====================
    double T = p.mass * (p.g + az_cmd);

    const double c_r = std::cos(roll);
    const double c_p = std::cos(pitch);
    double denom = clamp(c_r * c_p, 0.2, 1.0); // 분모가 너무 작아져 값이 무한대가 되는 현상 방지한 안정장치
    T = T / denom;
    // thrust saturation
    T = clamp(T, g.thrust_min, g.thrust_max); // 오버슈트 감소용
    // body frame 기준 추력 벡터, z축 방향으로만 추력 작용, 쿼터니언을 통해 world로 변환됨.
    u.thrust_body = {0.0, 0.0, T};

    return u;
}

// ======================================================================
// [2] Cascaded PID 제어기 (기존 베이스라인)
// ======================================================================
// ===== 메인 컨트롤러 (디버그 내역 제외, 추후 디버그 끝나면 전체 수정해서 이용하도록 남김) =====
Input controller_update(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains, double dt, Vec3& int_e_v) {
    Input u;

    // =====================
    // 1) 현재 오일러 추출
    // =====================
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quat_to_euler_zyx(s.q, roll, pitch, yaw);

    // =====================
    // 2) Position -> Velocity (P 제어)
    // 목표 위치로 가기위한 속도 명령 생성.
    // =====================
    const Vec3 e_p = ref.p_ref - s.p;
    // Vec3 v_cmd;
    const double vx_cmd = gains.kp_pos_xy * e_p.x + ref.v_ref.x;
    const double vy_cmd = gains.kp_pos_xy * e_p.y + ref.v_ref.y;
    double vz_cmd = gains.kp_pos_z  * e_p.z + ref.v_ref.z;
    // const double vx_cmd = gains.kp_pos_xy * e_p.x;
    // const double vy_cmd = gains.kp_pos_xy * e_p.y;
    // double vz_cmd = gains.kp_pos_z  * e_p.z;
    // 수평과 수직 dynamics가 다르고, 튜닝도 다르므로 z축은 별도 게인 사용.
    vz_cmd = clamp(vz_cmd, -gains.max_vz_cmd, gains.max_vz_cmd); // 오버슈트 감소용
    
    // =====================
    // 3) Velocity -> Acceleration (P 제어 + 가속도 피드포워드)
    // 원하는 속도를 만들기 위한 가속도 명령 생성.
    // 실제 속도 s.v가 명령보다 느리면 +가속도, 빠르면 -가속도.
    // 결과 가속도 명령은 월드 좌표계 기준으로 필요한 가속도.
    // =====================
    const double e_vx = vx_cmd - s.v.x;
    const double e_vy = vy_cmd - s.v.y;
    const double e_vz = vz_cmd - s.v.z;

    // 오차 누적 (I 제어)
    int_e_v.x += e_vx * dt;
    int_e_v.y += e_vy * dt;
    int_e_v.z += e_vz * dt;

    // Anti-windup (적분기 포화 방지: 바람이 너무 세거나 충돌 시 무한히 커지는 것 방지)
    int_e_v.x = clamp(int_e_v.x, -gains.max_int_vxy, gains.max_int_vxy);
    int_e_v.y = clamp(int_e_v.y, -gains.max_int_vxy, gains.max_int_vxy);
    int_e_v.z = clamp(int_e_v.z, -gains.max_int_vz,  gains.max_int_vz);

    const double ax_cmd = gains.kp_vel_xy * e_vx + gains.ki_vel_xy * int_e_v.x + ref.a_ref.x;
    const double ay_cmd = gains.kp_vel_xy * e_vy + gains.ki_vel_xy * int_e_v.y + ref.a_ref.y;

    double az_cmd = gains.kp_vel_z  * e_vz + gains.ki_vel_z  * int_e_v.z + ref.a_ref.z;
    az_cmd = clamp(az_cmd, -gains.max_az_cmd, gains.max_az_cmd); // 오버슈트 감소용
    // PD만 있을 시
    // const double ax_cmd = gains.kp_vel_xy * (vx_cmd - s.v.x) + ref.a_ref.x;
    // const double ay_cmd = gains.kp_vel_xy * (vy_cmd - s.v.y) + ref.a_ref.y;
    // double az_cmd = gains.kp_vel_z  * (vz_cmd - s.v.z) + ref.a_ref.z;
    az_cmd = clamp(az_cmd, -gains.max_az_cmd, gains.max_az_cmd);  // 오버슈트 감소용


    return attitude_and_thrust(s, ax_cmd, ay_cmd, az_cmd, ref.yaw_ref, params, gains);
}

ControllerOutput controller_update_dbg(const State& s, const Ref& ref,
                                        const Params& params, const ControllerGains& gains)
{
    ControllerOutput out;
    Vec3 dummy{0.0, 0.0, 0.0};
    out.u = controller_update(s, ref, params, gains, 0.0, dummy);
    return out;
}

// ======================================================================
// [3] Linear MPC 제어기 (Reference Preview 지원)
// ======================================================================

void MPCController::init(const MPCParams& mpc_p, const Params& drone_p,
                          const ControllerGains& gains, double dt)
{
    mpc_p_   = mpc_p; // mpc 파라미터 저장
    drone_p_ = drone_p; // 드론 물성치 저장
    gains_   = gains; // PID 게인 저장
    dt_      = dt; // 제어 주기 저장(0.01s)
    precompute(); // K_first 사전 계산 실행
}
// 단순 초기화 함수. 네 개의 파라미터를 멤버 변수에 복사한 뒤 `precompute()`를 호출해서 
// `K_first` 행렬을 미리 계산. `precompute()`가 무거운 연산(행렬 역산 등)을 전부 담당하고, 
// 이후 `update()`는 가벼운 연산만 하는 구조.

// ── setTrajectoryPreview ──────────────────────────────────────────────
// guidance_node로부터 미래 N스텝 궤적을 수신하여 preview_Xref_에 저장
// flat_data 형식: [px0,py0,pz0,vx0,vy0,vz0, px1,..., pxN-1,...,vzN-1]

// 역할:
// guidance_node가 퍼블리시한 `/guidance/trajectory_preview` 토픽 데이터를 받아서 
// MPC가 쓸 수 있는 형태로 변환해 저장한다.
// 입력 데이터 형식:
// `flat_data`는 1차원 배열(vector)인데, 안에 3D 궤적 정보가 시간 순서대로 쭉 이어 붙여져 있음:
// 
// flat_data = [
//   px0, py0, pz0, vx0, vy0, vz0,   ← 1스텝 후 (t + 1×dt)
//   px1, py1, pz1, vx1, vy1, vz1,   ← 2스텝 후 (t + 2×dt)
//   px2, py2, pz2, vx2, vy2, vz2,   ← 3스텝 후 (t + 3×dt)
//   ...
//   px14, py14, pz14, vx14, vy14, vz14  ← 15스텝 후 (t + 15×dt = t + 0.15s)
// ]
// 총 N×6 = 15×6 = 90개 숫자
void MPCController::setTrajectoryPreview(const std::vector<double>& flat_data)
{
    const int expected = mpc_p_.N * nx_;

    // 크기 불일치 시 preview 비활성화 (안전 fallback 보장)
    if (static_cast<int>(flat_data.size()) < expected) {
        has_preview_ = false;
        return;
    }

    preview_Xref_.resize(expected);
    for (int i = 0; i < expected; ++i) {
        preview_Xref_(i) = flat_data[i];
    }
    has_preview_ = true;
}

// ── precompute ────────────────────────────────────────────────────────
// MPC의 핵심 수학을 초기화 시 단 한 번 계산해두는 함수.
// 런타임(100Hz) update()에서는 여기서 만든 K_first_만 써서
// 행렬-벡터 곱 1회(270번 연산)로 최적 가속도 명령을 뽑아냄.
//
// 전체 계산 흐름:
//   Ad, Bd 구성  →  Φ, Γ 구성  →  Q̄, R̄ 구성  →  H 계산  →  K_first_ 저장
void MPCController::precompute()
{
    const int    N  = mpc_p_.N;
    const double dt = dt_;

    // ================================================================
    // [Step 1] Ad, Bd: 드론의 물리 법칙을 행렬로 표현
    //
    // 드론 상태: x = [px, py, pz, vx, vy, vz]ᵀ
    // 드론 입력: u = [ax, ay, az]ᵀ (가속도 명령)
    //
    // hover 근방 선형화 → double integrator 모델:
    //   x_{k+1} = Ad·x_k + Bd·u_k
    //
    // Ad: "입력 없을 때 한 스텝 후 상태 변화"
    //   Ad = [I₃  dt·I₃]  →  p_{k+1} = p_k + v_k·dt  (등속도 운동)
    //        [0₃    I₃ ]  →  v_{k+1} = v_k             (속도 유지)
    //
    // Bd: "가속도 입력이 한 스텝 후 상태에 미치는 영향"
    //   Bd = [0.5·dt²·I₃]  →  p_{k+1} += 0.5·a·dt²  (이차 적분)
    //        [   dt·I₃  ]  →  v_{k+1} += a·dt         (일차 적분)
    //
    // 쉽게 말하면 x_{k+1} = Ad·x_k + Bd·u_k가 드론의 운동 방정식
    // ================================================================
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(nx_, nx_);
    Ad.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;  // 우상단 블록: p += v·dt

    Eigen::MatrixXd Bd = Eigen::MatrixXd::Zero(nx_, nu_);
    Bd.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 0.5 * dt * dt;  // 위 블록: p += 0.5·a·dt²
    Bd.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;              // 아래 블록: v += a·dt

    // ================================================================
    // [Step 2] Ad^k 사전 계산 (k = 0 ~ N)
    //
    // Φ와 Γ를 만들 때 Ad^1, Ad^2, ..., Ad^N이 반복해서 필요함.
    // 매번 새로 곱하면 비효율적이므로 미리 계산해둠.
    //   Ad_pow[k] = Ad^k
    // ================================================================
    std::vector<Eigen::MatrixXd> Ad_pow(N + 1);
    Ad_pow[0] = Eigen::MatrixXd::Identity(nx_, nx_);  // Ad^0 = I
    for (int k = 1; k <= N; ++k) Ad_pow[k] = Ad_pow[k-1] * Ad;

    // ================================================================
    // [Step 3] Φ (N·nx × nx): 현재 상태 x₀의 미래 자연 응답 행렬
    //
    // "Φ: 지금 상태 x₀가 입력이 전혀 없을 때 미래에 어떻게 변하는가"
    //   Φ·x₀ = [Ad¹·x₀]  ← 1스텝 후 자연 상태
    //          [Ad²·x₀]  ← 2스텝 후 자연 상태
    //          [  ⋮   ]
    //          [Ad^N·x₀] ← N스텝 후 자연 상태
    //
    // update()에서 e₀ = Xref - Φ·x₀ 계산에 사용:
    //   Φ·x₀ = "아무 제어 안 하면 관성으로 가는 궤적"
    //   e₀   = "그 관성 궤적과 목표의 차이 → MPC가 교정해야 할 양"
    // ================================================================
    Phi_.resize(N * nx_, nx_);
    for (int k = 0; k < N; ++k)
        Phi_.block(k * nx_, 0, nx_, nx_) = Ad_pow[k + 1];  // k번째 블록 = Ad^(k+1)

    // ================================================================
    // [Step 4] Γ (N·nx × N·nu): 입력 시퀀스의 미래 상태 영향 행렬
    //
    // "j번째 스텝 입력 u_j가 i번째 스텝 상태에 얼마나 영향을 주는가"
    //   Γ[i,j] = Ad^(i-j)·Bd   (i >= j 일 때: 인과율 — 인과율(causality)을 표현, 
    // 미래 입력은 과거에 영향 없음)
    //          = 0               (i < j  일 때)
    // Φ, Γ를 합치면 전체 미래 궤적 표현 가능:
    //   X = Φ·x₀ + Γ·U  (자연응답 + 입력의 기여)
    //
    // 하삼각 블록 구조 (N=3 예시):
    //   Γ = [Bd           0       0  ]  ← u₀만 1스텝 상태에 영향
    //       [Ad·Bd        Bd      0  ]  ← u₀, u₁이 2스텝 상태에 영향
    //       [Ad²·Bd    Ad·Bd     Bd  ]  ← u₀, u₁, u₂가 3스텝 상태에 영향
    // N이 15일때는 크기가 Γ 크기: (N·nx) × (N·nu) = (15×6) × (15×3) = 90 × 45
    // 행 개수 90개, 열 개수 45 개
    //
    // 실제 메모리에 들어가는 크기는 
    // 전체 행렬 크기: 90 × 45 = 4050개 double 값
    // 메모리:        4050 × 8 bytes ≈ 32KB
    //
    // 읽는 방법
    //            u₀       u₁       u₂    ...    u₁₄
    //     ┌──────────────────────────────────────┐
    // x₁  │  Bd       0        0     ...     0  │
    // x₂  │ Ad·Bd     Bd       0     ...     0  │
    // x₃  │Ad²·Bd   Ad·Bd     Bd     ...     0  │
    // x₄  │Ad³·Bd  Ad²·Bd   Ad·Bd   ...     0  │
    //  ⋮  │   ⋮       ⋮        ⋮      ⋱      ⋮  │
    // x₁₅ │Ad¹⁴·Bd Ad¹³·Bd  Ad¹²·Bd ...   Bd  │
    //     └──────────────────────────────────────┘
    // 예를 들어 Γ의 3행 2열 블록 = Ad^(3-2)·Bd = Ad¹·Bd인데, 이 의미는:
    // "2번째 스텝 가속도(u₁)를 줬을 때,
    //  그 효과가 Ad를 한 번 거쳐서 3번째 스텝 상태(x₃)에 도달한다"
    // 인덱스 차이(i-j)가 클수록 더 많은 Ad를 거치니까, 오래 전에 준 입력일수록 
    // 현재 상태에 더 많이 전파(propagate)된 영향을 나타내는 것.
    // ================================================================
    Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(N * nx_, N * nu_);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j <= i; ++j)  // i >= j: 하삼각 (인과율)
            Gamma.block(i * nx_, j * nu_, nx_, nu_) = Ad_pow[i-j] * Bd;

    // ================================================================
    // [Step 5] Q̄, R̄: 비용 가중치 블록 대각 행렬
    //
    // 비용 함수: J = Σ‖x_k - x_ref‖²_Q + Σ‖u_k‖²_R
    // Q̄은 N 스텝 전체에 걸쳐 "위치 오차와 속도 오차를 얼마나 중요하게 볼 것인가"
    // 를 나타내는 블록 대각 행렬. 각 스텝마다 동일한 Q가 반복된다:
    //
    // Q̄ = blkdiag(Q, Q, ..., Q)  (N번 반복, N·nx × N·nx)
    //   Q = diag(q_pos_xy, q_pos_xy, q_pos_z,   ← XY/Z 분리 가중치
    //            q_vel_xy, q_vel_xy, q_vel_z)
    //   Z에 높은 가중치(q_pos_z=200 vs q_pos_xy=100) → 고도 유지 우선
    //
    // R̄ = blkdiag(R, R, ..., R)  (N번 반복, N·nu × N·nu)
    //   R = diag(r_acc_xy, r_acc_xy, r_acc_z)   ← 입력 크기 페널티
    //   값이 클수록 부드럽게, 작을수록 공격적으로 제어
    // ================================================================
    Eigen::VectorXd q_diag(nx_);
    q_diag << mpc_p_.q_pos_xy, mpc_p_.q_pos_xy, mpc_p_.q_pos_z,
              mpc_p_.q_vel_xy, mpc_p_.q_vel_xy, mpc_p_.q_vel_z;

    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(N * nx_, N * nx_);
    for (int k = 0; k < N; ++k)
        Q_bar.block(k * nx_, k * nx_, nx_, nx_) = q_diag.asDiagonal();

    Eigen::VectorXd r_diag(nu_);
    r_diag << mpc_p_.r_acc_xy, mpc_p_.r_acc_xy, mpc_p_.r_acc_z;

    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(N * nu_, N * nu_);
    for (int k = 0; k < N; ++k)
        R_bar.block(k * nu_, k * nu_, nu_, nu_) = r_diag.asDiagonal();

    // ================================================================
    // [Step 6] H: 비용 함수의 Hessian
    //
    // 비용 함수 J를 입력 U에 대해 전개하면:
    // J(U) = ‖Φ·x₀ + Γ·U - Xref‖²_Q̄  +  ‖U‖²_R̄
    //        └──────────────────────┘    └────────┘
    //        "궤적이 목표에서 얼마나 벗어났나"  "입력을 얼마나 썼나"
    //
    // ∂J/∂U = 0 으로 최적해를 구하면 Hessian H가 등장:
    //   H = Γᵀ·Q̄·Γ + R̄   (크기: N·nu × N·nu = 45 × 45)
    //
    // H가 Positive Definite이면 비용 함수가 볼록(convex) → 유일한 최솟값 존재.
    //   Q̄ ≥ 0, R̄ > 0 이면 H > 0 항상 보장됨.
    // ================================================================
    const Eigen::MatrixXd H = Gamma.transpose() * Q_bar * Gamma + R_bar;

    // ================================================================
    // [Step 7] K_first_: 최적 게인 행렬 (핵심 결과물)
    //
    // 비용 J를 최소화하는 최적 입력 시퀀스:
    //   U* = H⁻¹ · Γᵀ · Q̄ · (Xref - Φ·x₀)
    //      = K_mpc · e₀          (e₀ = Xref - Φ·x₀)
    //
    // K_mpc 크기: (N·nu) × (N·nx) = 45 × 90
    //   → K_mpc의 각 블록 행 = 각 스텝의 최적 입력 게인
    //
    // H.ldlt().solve(...) = H⁻¹ · (...)
    //   LDLT 분해를 쓰는 이유: H가 symmetric positive definite 행렬이라
    //   단순 역행렬보다 훨씬 빠르고 수치적으로 안정적임.
    //
    // [Receding Horizon] K_first_ = K_mpc의 맨 위 nu(=3)개 행만 저장
    //   U* = [u₀*]  ← K_first_에 해당, 실제로 드론에 적용
    //        [u₁*]  ← 다음 스텝에서 재계산하므로 버림
    //        [ ⋮ ]
    //   "전체 horizon을 최적화하되, 첫 번째 입력만 실제로 사용하고
    //    다음 스텝에서 다시 최적화한다"는 MPC 철학의 구현.
    //
    // K_first_ 크기: nu × (N·nx) = 3 × 90
    // update()에서: u₀* = K_first_ · e₀ → 행렬-벡터 곱 1회(270번 연산)로 완료
    // ================================================================
    const Eigen::MatrixXd K_mpc = H.ldlt().solve(Gamma.transpose() * Q_bar);
    K_first_ = K_mpc.topRows(nu_);  // 첫 번째 입력 게인만 저장

    initialized_ = true;
}

// ── update ────────────────────────────────────────────────────────────
// 매 100Hz마다 호출되는 MPC 실시간 제어 루프
//
// [전체 흐름]
//   x₀ 구성 → Xref 구성 → e₀ = Xref - Φ·x₀ → u₀* = K_first·e₀ → 자세 제어
//
// [계산 부담]
//   핵심 연산은 행렬-벡터 곱 1회 (K_first: 3×90, e₀: 90×1 → 270 MACs)
//   precompute()에서 K_first_를 사전 계산해두기 때문에 가능한 구조
// ─────────────────────────────────────────────────────────────────────
Input MPCController::update(const State& s, const Ref& ref)
{
    // ═══════════════════════════════════════════════════════════════
    // STEP 1. 현재 상태 벡터 x₀ 구성 (6×1)
    //
    // navigation_node의 EKF가 추정한 현재 상태를 Eigen 벡터로 변환
    // x₀ = [px, py, pz, vx, vy, vz]ᵀ
    //
    // 이게 MPC의 출발점: "나는 지금 여기 있고, 이 속도로 움직이고 있다"
    // ═══════════════════════════════════════════════════════════════
    Eigen::VectorXd x0(nx_);
    x0 << s.p.x, s.p.y, s.p.z,   // 현재 위치 (EKF 추정, world frame)
          s.v.x, s.v.y, s.v.z;   // 현재 속도 (EKF 추정, world frame)

    // ═══════════════════════════════════════════════════════════════
    // STEP 2. 목표 궤적 Xref 구성 (N·nx × 1 = 90×1)
    //
    // Xref = "앞으로 N스텝 동안 드론이 있어야 할 목표 상태 시퀀스"
    // 구조: [x_ref_1, x_ref_2, ..., x_ref_N]
    //        각 x_ref_k = [px, py, pz, vx, vy, vz] (6차원)
    //
    // ┌─ Preview 모드 (정상 동작) ────────────────────────────────┐
    // │ guidance_node가 다항식에서 미래 위치/속도를 직접 계산하여 │
    // │ /guidance/trajectory_preview 토픽으로 퍼블리시한 데이터   │
    // │ Xref[k] = t + k*dt 시점의 실제 궤적 위치/속도            │
    // │ → MPC가 "궤적이 앞으로 이동 중"임을 정확히 인지           │
    // │ → guidance 속도에 맞춰 비행 → 앞서가기/멈춤 현상 완화     │
    // └───────────────────────────────────────────────────────────┘
    // ┌─ Fallback 모드 (초기화 직후 / guidance 꺼진 경우) ────────┐
    // │ 현재 setpoint(ref.p_ref, ref.v_ref)를 N번 그대로 복사     │
    // │ 문제: MPC가 "지금 당장 저기 가야 해"로 해석               │
    // │ → 최대 가속 → guidance 시간표보다 빠르게 날아버림          │
    // │ → 앞서버린 후 역가속 발생 → 진동 또는 중간 지점 멈춤      │
    // └───────────────────────────────────────────────────────────┘
    // ═══════════════════════════════════════════════════════════════
    Eigen::VectorXd X_ref(mpc_p_.N * nx_);

    if (has_preview_) {
        // [Preview 모드]
        // setTrajectoryPreview()가 수신한 미래 궤적 배열을 그대로 사용
        // 형식: [px0,py0,pz0,vx0,vy0,vz0, px1,..., pxN-1,...,vzN-1]
        X_ref = preview_Xref_;
    } else {
        // [Fallback: 상수 참조]
        // 현재 setpoint를 N번 복사 → 시간 파라미터 불일치 문제 발생
        // preview가 안정적으로 수신되면 이 경로는 사실상 사용되지 않음
        Eigen::VectorXd x_ref(nx_);
        x_ref << ref.p_ref.x, ref.p_ref.y, ref.p_ref.z,  // 목표 위치
                 ref.v_ref.x, ref.v_ref.y, ref.v_ref.z;   // 목표 속도
        for (int k = 0; k < mpc_p_.N; ++k)
            X_ref.segment(k * nx_, nx_) = x_ref;  // 동일 setpoint N번 반복
    }

    // ═══════════════════════════════════════════════════════════════
    // STEP 3. 오차 벡터 e₀ 계산 (N·nx × 1 = 90×1)
    //
    // e₀ = Xref - Φ·x₀
    //
    // Φ·x₀: "지금부터 아무 가속도도 주지 않을 때 (관성만으로) 드론이
    //         앞으로 N스텝 동안 어떻게 움직이는가" = 자연 응답
    //   Φ·x₀[k번째 블록] = Ad^(k+1)·x₀  (k+1 스텝 후 자연 상태)
    //   예: 지금 vx=1.0m/s이면, Φ·x₀는 0.01초마다 1cm씩 앞으로 가는 궤적
    //
    // e₀: "자연 응답과 목표 궤적의 차이 → MPC가 교정해야 할 양"
    //   e₀[k번째 블록] = (k스텝 후 목표) - (k스텝 후 자연 응답)
    //   e₀가 크면 MPC가 더 큰 가속도를 출력
    //   e₀ = 0이면 자연 응답이 이미 목표와 일치 → 가속도 불필요
    // ═══════════════════════════════════════════════════════════════
    const Eigen::VectorXd e0 = X_ref - Phi_ * x0;
    //                         └─목표─┘  └─자연응답─┘

    // ═══════════════════════════════════════════════════════════════
    // STEP 4. 최적 첫 번째 가속도 u₀* 계산 (3×1)
    //
    // u₀* = K_first_ · e₀
    //
    // K_first_ (3×90): precompute()에서 사전 계산한 최적 피드백 게인
    //   = (H⁻¹ · Γᵀ · Q̄)의 첫 번째 블록 행 (첫 nu=3 행)
    //   의미: "각 위치/속도 오차가 1단위일 때 최적으로 줘야 할 가속도"
    //   Q/R 비율로 결정됨:
    //     q_pos 크면 → K_first_ 스케일 커짐 → 위치 오차에 강하게 반응
    //     r_acc 크면 → K_first_ 스케일 작아짐 → 입력 아끼며 부드럽게 반응
    //
    // [Receding Horizon 원칙]
    //   최적화는 N스텝 전체(U* = [u₀*, u₁*, ..., u_{N-1}*])에 대해 수행하지만
    //   실제로 드론에 적용하는 건 첫 번째 입력 u₀*만
    //   다음 100Hz 주기에 새로운 상태로 다시 전체 최적화 → 실시간 반응성 보장
    //
    // [계산 비용]
    //   행렬-벡터 곱: (3×90) × (90×1) = 270 MACs → 100Hz 실시간 가능
    // ═══════════════════════════════════════════════════════════════
    const Eigen::VectorXd u0 = K_first_ * e0;
    //                         └──게인──┘ └오차┘

    // ═══════════════════════════════════════════════════════════════
    // STEP 5. 가속도 명령 추출 + 적분 보상항 추가 (MPC+I)
    //
    // u0(0~2): MPC 비례 피드백이 계산한 world frame 가속도 명령 [ax, ay, az]
    //
    // [왜 a_ref(feedforward)를 제거했나?]
    //   MPC의 Xref에 이미 v_ref(목표 속도)가 포함되어 있음.
    //   K_first_가 속도 오차를 통해 가속도를 암묵적으로 계산하므로
    //   a_ref까지 더하면 이중 인가 → 실험으로 확인 후 제거함.
    //
    // [적분 보상항 추가 이유 (MPC+I)]
    //   MPC는 K_first × e₀ 형태의 순수 비례 피드백임.
    //   바람처럼 지속적인 외란이 있으면 정상 상태 위치 오차가 계속 남음.
    //   위치 오차를 dt_씩 적분하여 가속도 보상항으로 더해줌.
    //   → 외란을 누적 감지하여 점진적으로 보상 (PID의 ki_vel과 동일 역할)
    //   anti-windup: 적분값이 max_int_pos를 넘지 않도록 클램핑함.
    // ═══════════════════════════════════════════════════════════════

    // 현재 위치 오차 (world frame): 목표 위치 - 실제 위치
    const double ep_x = ref.p_ref.x - s.p.x;
    const double ep_y = ref.p_ref.y - s.p.y;
    const double ep_z = ref.p_ref.z - s.p.z;

    // 위치 오차 적분 (dt_: 제어 주기, 기본 0.01s)
    integral_pos_.x += ep_x * dt_;
    integral_pos_.y += ep_y * dt_;
    integral_pos_.z += ep_z * dt_;

    // anti-windup: 적분 포화 방지
    const double mi = mpc_p_.max_int_pos;
    integral_pos_.x = std::max(-mi, std::min(mi, integral_pos_.x));
    integral_pos_.y = std::max(-mi, std::min(mi, integral_pos_.y));
    integral_pos_.z = std::max(-mi, std::min(mi, integral_pos_.z));

    // MPC 비례 피드백 + 적분 보상 합산
    const double ax_cmd = u0(0) + mpc_p_.ki_pos_xy * integral_pos_.x;
    const double ay_cmd = u0(1) + mpc_p_.ki_pos_xy * integral_pos_.y;
    const double az_cmd = u0(2) + mpc_p_.ki_pos_z  * integral_pos_.z;

    // ═══════════════════════════════════════════════════════════════
    // STEP 6. 자세 제어 inner loop으로 넘기기
    //
    // MPC outer loop의 역할: "world frame에서 어떤 가속도가 필요한가" 결정
    // attitude_and_thrust inner loop의 역할:
    //   1. tilt mapping:  ax/ay → 드론을 얼마나 기울일지 (roll, pitch 목표)
    //   2. attitude PD:   목표 자세 따라가기 → moment 명령
    //   3. thrust 계산:   az + 중력 보상 + tilt compensation → 추력 크기
    //
    // 반환: Input (thrust_body, moment_body) → simulation_node로 전달
    // ═══════════════════════════════════════════════════════════════
    return attitude_and_thrust(s, ax_cmd, ay_cmd, az_cmd,
                                ref.yaw_ref, drone_p_, gains_);
}