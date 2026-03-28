#pragma once
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "sixdof.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================================================================
// controller.h
// PID Cascaded 제어기 + Linear MPC 제어기 (Reference Preview 지원)
// ======================================================================

// ===== PID 게인 구조체 (기존 유지) =====
struct ControllerGains {
    double kp_pos_xy = 1.0;
    double kp_pos_z  = 1.5;
    double kp_vel_xy = 2.0;
    double kp_vel_z  = 2.5;
    double kp_att_rp  = 8.0;
    double kd_att_rp  = 1.5;
    double kp_att_yaw = 4.0;
    double kd_att_yaw = 0.8;
    double ki_vel_xy = 0.5;
    double ki_vel_z  = 0.5;
    double max_int_vxy = 2.0;
    double max_int_vz  = 2.0;
    double max_tilt_deg = 20.0;
    double max_vxy_cmd  = 2.0;
    double max_axy_cmd  = 4.0;
    double max_vz_cmd   = 2.0;
    double max_az_cmd   = 6.0;
    double thrust_min   = 0.0;
    double thrust_max   = 80.0;
    double moment_max_rp = 0.15;
    double moment_max_y  = 0.10;
};

// ===== 목표 상태 구조체 (기존 유지) =====
struct Ref {
    Vec3   p_ref;
    Vec3   v_ref;
    Vec3   a_ref;
    double yaw_ref = 0.0;
};

// ===== Debug 구조체 (기존 유지) =====
struct ControllerDebug {
    double roll{0.0}, pitch{0.0}, yaw{0.0};
    double roll_ref{0.0}, pitch_ref{0.0}, yaw_ref{0.0};
    Vec3 v_cmd{0.0, 0.0, 0.0};
    Vec3 a_cmd{0.0, 0.0, 0.0};
};

struct ControllerOutput {
    Input u;
    ControllerDebug dbg;
};

// ===== 함수 선언 (기존 유지) =====
void quat_to_euler_zyx(const Quat& q, double& roll, double& pitch, double& yaw);

Input controller_update(const State& s, const Ref& ref, const Params& params,
                         const ControllerGains& gains, double dt, Vec3& int_e_v);

ControllerOutput controller_update_dbg(const State& s, const Ref& ref,
                                        const Params& params, const ControllerGains& gains);

// ======================================================================
//  Linear MPC 파라미터 구조체
// ======================================================================
// init()에서 K_first를 미리 계산해두고, update()에서 K_first × (Xref - Φ·x₀) 곱셈 한 번으로 
// 가속도를 뽑아내는 구조. Xref를 어떻게 채우느냐(상수 vs preview)가 MPC 동작의 핵심 차이를 만듦.

// Linear MPC에서 선형화가 일어나는 지점: 
// hover 상태, vx=0, vy=0, vz=0, roll=0, pitch=0 인 상태를 기준점으로 잡고 선형화
// 원래 드론의 운동 방정식은 비선형, 이걸 hover 근방에서 소각 근사를 적용하면
// R(q) ≈ I  (회전 행렬을 단위 행렬로 근사)
// → p̈ ≈ [ax, ay, az]ᵀ  (드론을 그냥 공중에 떠 있는 점질량으로 취급) -> 여기 Double Integer 모델

// 위치는 속도의 적분
// Ad.block<3,3>(0,3) = I * dt;   // p_{k+1} = p_k + v_k·dt
// // 속도는 가속도의 적분
// Bd.block<3,3>(3,0) = I * dt;   // v_{k+1} = v_k + a_k·dt
// 자세(roll, pitch, yaw)가 완전히 사라짐. 드론을 3차원 공간에 떠 있는 점 하나로 취급하는 것

// 이게 실제로 문제가 되는 경우
// 선형화 오차 = 실제 동역학 - 선형화된 모델의 차이
// 기울기가 커질수록 선형화 오차가 커짐:
// 상황	roll/pitch 각도	선형화 오차	MPC 성능
// 호버링/저속	~5° 이하	거의 없음	양호
// 중속 비행	~15°	중간	약간 저하
// 고기동/레이싱	45° 이상	매우 큼	심각하게 저하
struct MPCParams {
    int    N        = 15;     // 예측 horizon 스텝 수

    // Q 행렬: XY/Z 분리 가중치 (PID에서 Z게인이 XY보다 높았던 설계 철학 반영)
    double q_pos_xy = 100.0; // XY 위치 오차 패널티
    double q_pos_z  = 200.0; // z 위치 오차 패널티
    double q_vel_xy = 10.0; // XY 속도 오차 패널티
    double q_vel_z  = 30.0; // Z 속도 오차 패널티
    // Q 행렬의 대각 원소, 비용함수 J에서 각 상태 오차에 얼마나 민감하게 반응할지 결정
    // J = Σ [위치오차² × q_pos + 속도오차² × q_vel] + Σ [가속도² × r_acc]

    // R 행렬: XY/Z 분리 입력 가중치
    // 가속도 입력 크기에 대한 패널티, 클수록 MPC가 가속도를 아껴서 부드럽게 제어
    double r_acc_xy = 1.0;
    double r_acc_z  = 0.5;
};

// ======================================================================
//  Linear MPC 컨트롤러 클래스 (Reference Preview 지원)
//
//  [원리]
//    hover 근방 선형화 → double integrator 모델
//      x = [px, py, pz, vx, vy, vz]ᵀ,  u = [ax, ay, az]ᵀ
//      x_{k+1} = Ad·xk + Bd·uk
//
//    Condensed MPC:
//      U* = K_first · (Xref - Φ·x₀)    (receding horizon)
//
//  [Reference Preview 추가]
//    기존 문제: Xref를 현재 setpoint 하나로 N번 복사
//      → guidance보다 항상 빠르게 비행 → 앞서버린 후 역가속 → 진동/멈춤
//
//    해결: guidance_node가 미래 N스텝 궤적 배열을 퍼블리시
//      → Xref[k] = t+k*dt 시점의 실제 궤적 위치/속도
//      → MPC가 "궤적이 앞으로 이동 중"임을 정확히 인지
//
//  [아키텍처]
//    MPC outer loop: preview Xref + 현재 상태 → 최적 가속도
//    PID inner loop: 가속도 → tilt mapping → attitude PD → thrust+moment
// ======================================================================
class MPCController {
public:
    // 초기화 (반드시 update() 전에 호출)
    // 수학적으로 무거운 행렬 계산을 전부 미리 함.
    void init(const MPCParams& mpc_p, const Params& drone_p,
              const ControllerGains& gains, double dt);

    // guidance_node로부터 미래 N스텝 궤적 preview 수신
    // flat_data 형식: [px0,py0,pz0,vx0,vy0,vz0, px1,..., pxN-1,...,vzN-1]
    // 크기: N * nx = N * 6
    void setTrajectoryPreview(const std::vector<double>& flat_data);

    // 매 제어 주기 호출 → Input 반환
    // preview가 있으면 preview Xref 사용, 없으면 상수 참조(fallback)
    // 0.01 초마다 호출되는 함수, 내부 연산은:
    // e₀ = Xref - Φ·x₀      // 오차 계산 (90차원 벡터)
    // u₀ = K_first · e₀      // 행렬-벡터 곱 1회 → 가속도 3개
    Input update(const State& s, const Ref& ref);

    bool isInitialized() const { return initialized_; }

private:
    void precompute();

    MPCParams       mpc_p_;
    Params          drone_p_;
    ControllerGains gains_;
    double          dt_{0.01};
    bool            initialized_{false};

    static constexpr int nx_ = 6; // [px, py, pz, vx, vy, vz]
    static constexpr int nu_ = 3; // [ax, ay, az]

    Eigen::MatrixXd Phi_;     // (N·nx) × nx = 90 x 6
    //     **Φ (Phi)**: "지금 상태 x₀를 주면, 아무 입력 없을 때 미래가 어떻게 되는지" 
    // 를 표현하는 행렬. 드론의 관성을 수학으로 표현한 것.
    // Φ = [Ad¹ ]     → 1스텝 후 자연 응답
    //     [Ad² ]     → 2스텝 후 자연 응답
    //     [... ]
    //     [AdN ]     → N스텝 후 자연 응답
    Eigen::MatrixXd K_first_; // nu × (N·nx) = 3 x 90
    // K_first: 이게 MPC의 핵심. 오차를 입력으로 직접 바꾸는 최적 게인 행렬.
    // `precompute()`에서 아래 과정을 거쳐 한 번 계산해 저장:
    // Γ 구성 → H = ΓᵀQ̄Γ + R̄ → K_mpc = H⁻¹ΓᵀQ̄ → K_first = K_mpc의 첫 3행
    // K_first 크기가 3x90이라는 건 90차원의 오차 벡터(미래15스텝 x 6차원 상태)를 받아서 
    // 3차원 가속도 명령을 하나 뽑는다는 것

    // Reference Preview 관련
    bool            has_preview_{false};   // preview 수신 여부
    Eigen::VectorXd preview_Xref_;         // N·nx × 1, guidance가 제공한 미래 궤적
    // 이번 주에 추가한 핵심 부분. Xref(90차원 참조 벡터)를 어떻게 만드느냐에 따라 MPC 동작이 완전히 달라짐.

    // 기존 방식 (has_preview_ = false):
    // x_ref = [p_ref, v_ref]  ← 지금 guidance setpoint 하나
    // Xref = [x_ref, x_ref, x_ref, ..., x_ref]  ← 15번 동일하게 복사
    // MPC 입장: "지금 당장 저기 가야 해" → 최대 가속 → 빠르게 날아 앞서버림

    // Preview 방식 (has_preview_ = true):
    // Xref[0] = t+0.01초의 실제 궤적 위치/속도   ← guidance 다항식에서 직접 계산
    // Xref[1] = t+0.02초의 실제 궤적 위치/속도
    // Xref[14] = t+0.15초의 실제 궤적 위치/속도

    // 스탭이 n=15라서 receding horizon을 15개 보는건데, K_first를미리 계산하는거가 이해가 안돼. 드론이 이동할때마다 K_first를 계산하는거아냐?

    // K_first 사전 계산의 필요성을 검토했다
    // 핵심을 찌른 질문이야. 결론부터 말하면 K_first는 드론 위치와 무관해서 한 번만 계산해도 돼.

    // 왜 K_first가 드론 위치와 무관한가
    // K_first가 만들어지는 과정을 보면 이해가 돼.

    // K_first = 첫 번째 행(H⁻¹ · Γᵀ · Q̄)

    // H = Γᵀ · Q̄ · Γ + R̄
    // 여기서 Γ, Q̄, R̄ 이 세 개가 뭐로 만들어지는지가 핵심이야.

    // Γ → Ad, Bd로 만들어짐  (드론 시스템 행렬, 시간에 따라 변하지 않음)
    // Q̄ → q_pos, q_vel로 만들어짐  (설계자가 정한 가중치, 고정값)
    // R̄ → r_acc로 만들어짐  (설계자가 정한 가중치, 고정값)
    // Ad와 Bd가 뭔지 보면:

    // cpp
    // Ad = [I₃  dt·I₃]   // double integrator — 물리 상수만 들어감
    //      [0₃    I₃ ]

    // Bd = [0.5·dt²·I₃]  // 마찬가지
    //      [   dt·I₃  ]

    // Ad, Bd는 dt(시간 간격) 하나로만 결정됨. 드론이 (0,0,0)에 있든 (5,3,2)에 있든, 
    // 속도가 빠르든 느리든 이 행렬은 변하지 않음.
    // 즉 K_first를 만드는 모든 재료(Ad, Bd, Q̄, R̄)가 상수라서 K_first도 상수임.

    // 드론 위치 정보는 언제 들어가나?
    // K_first는 상수지만, 오차 e₀는 매번 달라져. 드론이 이동할 때마다 달라지는 건 K_first가 아니라 e₀.
    // 매 100Hz마다:

    // x₀ = [현재 px, py, pz, vx, vy, vz]  ← 드론이 이동할 때마다 바뀜
    // Xref = [목표 궤적]                   ← guidance setpoint마다 바뀜
    // e₀ = Xref - Φ·x₀                   ← 그래서 e₀가 매번 달라짐
    // u₀ = K_first · e₀                   ← K_first는 고정, e₀만 바뀜

    // PID와 비교하면:
    // PID:   u = Kp × e(t)          K_first 역할 = Kp (고정 게인)
    // MPC:   u = K_first × e₀       e 역할 = e₀ (매번 계산)
    // PID에서 Kp를 매번 다시 계산하지 않는 것처럼, MPC도 K_first를 매번 계산하지 않음. 
    // 드론 위치 정보는 e₀에 들어가고, K_first는 그 오차를 최적 입력으로 변환하는 고정 게인이야.

    // 언제 K_first를 다시 계산해야 하나?
    // 이 구조의 전제 조건은 시스템이 선형 시불변(LTI) 이라는 것. Ad, Bd가 상수라는 뜻이고, 
    // 현재 모델(double integrator)이 정확히 그 조건을 만족해.

    // 만약 이 조건이 깨지면 K_first를 매번 다시 계산해야 함:

    // 케이스	             설명	                          K_first 재계산 필요?
    // 우리 Linear MPC	     double integrator, 상수 Ad/Bd   ❌ 1회만
    // 질량이 변하는 드론	    Ad/Bd가 바뀜	                ✅ 매번
    // 빠른 기동 (비선형 영역)	선형화 오차 발생               	   ✅ 매번 (LPV-MPC)
    // NMPC	                비선형 모델 직접 사용	           ✅ 매번 (계산 부담 큰 이유)

};