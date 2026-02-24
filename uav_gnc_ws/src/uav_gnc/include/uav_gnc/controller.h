#pragma once
#include <cmath>
#include "sixdof.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ControllerGains {
    // Position -> Velocity
    double kp_pos_xy = 1.0;
    double kp_pos_z  = 1.5;
    // Velocity -> Acceleration
    double kp_vel_xy = 2.0;
    double kp_vel_z  = 2.5;
    // Attitude PD -> Moment
    double kp_att_rp  = 8.0;
    double kd_att_rp  = 1.5;
    double kp_att_yaw = 4.0;
    double kd_att_yaw = 0.8;
    // 최대 기울기 각도 제한 (degree)
    double max_tilt_deg = 20.0;
    // ===== Actuator Saturation(구동기 포화) 방지와 Trajectory Safety(궤적 안전)를 위해 설계된 Hard Constraint =====
    double max_vxy_cmd  = 2.0;   // [m/s]
    double max_axy_cmd  = 4.0;   // [m/s^2]
    double max_vz_cmd   = 2.0;   // [m/s]
    double max_az_cmd   = 6.0;   // [m/s^2]
    double thrust_min   = 0.0;   // [N]
    double thrust_max   = 80.0;  // [N]
    double moment_max_rp = 0.15; // [N*m] Roll/Pitch 모멘트 제한
    double moment_max_y  = 0.10; // [N*m] Yaw 모멘트 제한
};

struct Ref {
    // 목표 위치(월드)와 목표 yaw
    Vec3 p_ref;
    Vec3 v_ref;
    Vec3 a_ref;
    double yaw_ref = 0.0;
};

// ===== Debug 출력용 구조체 =====
// 현재 자세, 명령 자세등도 출력하도록
struct ControllerDebug {
    // 현재 attitude
    double roll{0.0}, pitch{0.0}, yaw{0.0};
    // 명령 attitude (from guidance/tilt mapping)
    double roll_ref{0.0}, pitch_ref{0.0}, yaw_ref{0.0};
    // 속도, 가속도 command
    Vec3 v_cmd{0.0, 0.0, 0.0};
    Vec3 a_cmd{0.0, 0.0, 0.0};
};

struct ControllerOutput {
    Input u;
    ControllerDebug dbg;
};

// Quaternion -> Euler(ZYX) helper를 main에서도 쓰게 헤더에 선언(main에서 오일러 계산해 로깅/검증하도록 링크 가능하게)
void quat_to_euler_zyx(const Quat& q, double& roll, double& pitch, double& yaw);

// 현재 상태 s를 보고, 목표 ref를 따라가도록 입력 u를 계산.
Input controller_update(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains);

// Debug 포함 함수
ControllerOutput controller_update_dbg(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains);