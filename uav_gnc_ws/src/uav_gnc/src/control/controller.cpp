#include "uav_gnc/controller.h"
// =====================
// 위치, 속도, 가속도/자세, 모멘트/추력 명령을 만드는 cascaded 컨트롤러.
// 1. 목표 위치(ref.p_ref) 와 현재 위치(s.p) 의 오차로부터
// 2. “어떤 속도로 가야 하는지(v_cmd)”를 만들고
// 3. 그 속도를 만들기 위한 “가속도 명령(a_cmd)”을 만든 뒤
// 4. 그 가속도를 만들기 위해 드론을 “얼마나 기울일지(roll_ref, pitch_ref)”로 바꾸고
// 5. 그 기울기를 따라가게 하는 “모멘트(Mx,My,Mz)”와 “추력(T)”을 계산해서
// 6. 6-DOF 모델(plant)에 입력으로 넣는 제어기 구조.
// =====================

// 상한/하한으로우로 제한하는 함수.
static double clamp(double v, double lo, double hi) {

    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// yaw 오차는 179도와 -181도가 사실상 같은 방향인데, 단순 차를 내면 360도 오차로 보임.
// 그래서 각도 오차를 [-π, π] 범위로 제한하여 가장 짧은 방향의 오차로 만드는 함수(최소의 움직임으로 제어).
static double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

// body->world quaternion(q) 기준 ZYX(yaw-pitch-roll) 오일러 추출.
// 6-DOF 상태는 quaternion(s.q)로 자세를 가지고 있음.
// 간단한 자세 제어 PD 구현엔 roll/pitch/yaw가 더 직관적, 따라서 quaternion을 ZYX 순서의 오일러로 변환.
// ※ s.q가 “body→world”인지 “world→body”인지에 따라 부호/해석이 바뀔 수 있음.
void quat_to_euler_zyx(const Quat& q, double& roll, double& pitch, double& yaw) {
    // roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (q.w*q.x + q.y*q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x*q.x + q.y*q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    const double sinp = 2.0 * (q.w*q.y - q.z*q.x);
    if (std::fabs(sinp) >= 1.0) pitch = std::copysign(M_PI/2.0, sinp);
    else pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (q.w*q.z + q.x*q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ===== Debug 포함 메인 컨트롤러(clamp로 안전장치 추가) =====
ControllerOutput controller_update_dbg(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains) {
    ControllerOutput out;

    // 1)
    double roll=0.0, pitch=0.0, yaw=0.0;
    quat_to_euler_zyx(s.q, roll, pitch, yaw);

    // 2) Position -> Velocity (P)
    const Vec3 e_p = ref.p_ref - s.p;
    Vec3 v_cmd;
    v_cmd.x = gains.kp_pos_xy * e_p.x;
    v_cmd.y = gains.kp_pos_xy * e_p.y;
    v_cmd.z = gains.kp_pos_z  * e_p.z;
    // v_cmd.x = clamp(v_cmd.x, -gains.max_vxy_cmd, gains.max_vxy_cmd);
    // v_cmd.y = clamp(v_cmd.y, -gains.max_vxy_cmd, gains.max_vxy_cmd);
    // v_cmd.z = clamp(v_cmd.z, -gains.max_vz_cmd, gains.max_vz_cmd); // 오버슈트 감소용

    // 3)
    Vec3 a_cmd;
    a_cmd.x = gains.kp_vel_xy * (v_cmd.x - s.v.x);
    a_cmd.y = gains.kp_vel_xy * (v_cmd.y - s.v.y);
    a_cmd.z = gains.kp_vel_z  * (v_cmd.z - s.v.z);
    // a_cmd.x = clamp(a_cmd.x, -gains.max_axy_cmd, gains.max_axy_cmd);
    // a_cmd.y = clamp(a_cmd.x, -gains.max_axy_cmd, gains.max_axy_cmd);
    // a_cmd.z = clamp(a_cmd.z, -gains.max_az_cmd, gains.max_az_cmd); // 오버슈트 감소용

    // 4)
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    const double ax_h =  cy*a_cmd.x + sy*a_cmd.y;
    const double ay_h = -sy*a_cmd.x + cy*a_cmd.y;

    double pitch_ref =  ax_h / params.g;
    double roll_ref  = -ay_h / params.g;
    const double yaw_ref = ref.yaw_ref;

    const double max_tilt = gains.max_tilt_deg * M_PI / 180.0;
    pitch_ref = clamp(pitch_ref, -max_tilt, max_tilt);
    roll_ref  = clamp(roll_ref,  -max_tilt, max_tilt);

    // 5)
    const double e_roll  = roll_ref  - roll;
    const double e_pitch = pitch_ref - pitch;
    const double e_yaw   = wrap_pi(yaw_ref - yaw);

    const double p_rate = s.w.x;
    const double q_rate = s.w.y;
    const double r_rate = s.w.z;

    double Mx = gains.kp_att_rp  * e_roll  - gains.kd_att_rp  * p_rate;
    double My = gains.kp_att_rp  * e_pitch - gains.kd_att_rp  * q_rate;
    double Mz = gains.kp_att_yaw * e_yaw   - gains.kd_att_yaw * r_rate;
    // 오버슈트 감소용
    // Mx = clamp(Mx, -gains.moment_max_rp, gains.moment_max_rp);
    // My = clamp(My, -gains.moment_max_rp, gains.moment_max_rp);
    // Mz = clamp(Mz, -gains.moment_max_y,  gains.moment_max_y);

    out.u.moment_body = {Mx, My, Mz};

    // 6)
    double T = params.mass * (params.g + a_cmd.z);

    const double c_r = std::cos(roll);
    const double c_p = std::cos(pitch);
    const double denom = clamp(c_r * c_p, 0.2, 1.0);
    T = T / denom;
    // T = clamp(T, gains.thrust_min, gains.thrust_max); // 오버슈트 감소용

    out.u.thrust_body = {0.0, 0.0, T};

    // Debug 채우기 코드
    out.dbg.roll = roll; out.dbg.pitch = pitch; out.dbg.yaw = yaw;
    out.dbg.roll_ref = roll_ref; out.dbg.pitch_ref = pitch_ref; out.dbg.yaw_ref = yaw_ref;
    out.dbg.v_cmd = v_cmd;
    out.dbg.a_cmd = a_cmd;

    return out;
}
// ===== 메인 컨트롤러 (디버그 내역 제외, 추후 디버그 끝나면 전체 수정해서 이용하도록 남김) =====
Input controller_update(const State& s, const Ref& ref, const Params& params, const ControllerGains& gains) {
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
    const double ax_cmd = gains.kp_vel_xy * (vx_cmd - s.v.x) + ref.a_ref.x;
    const double ay_cmd = gains.kp_vel_xy * (vy_cmd - s.v.y) + ref.a_ref.y;
    double az_cmd = gains.kp_vel_z  * (vz_cmd - s.v.z) + ref.a_ref.z;
    az_cmd = clamp(az_cmd, -gains.max_az_cmd, gains.max_az_cmd);  // 오버슈트 감소용

    // =====================
    // 4) 가속도 명령을 기울기로 변환(tilt mapping, 쿼드콥터 드론의 기동을 고려, a_cmd_xy -> roll_ref, pitch_ref)
    // thrust 벡터를 기울여(기체를 기울여) 수평 가속도를 만듦으로 xy 이동.
    // 또한 roll/pitch는 “기체의 앞/옆 방향”과 연관되나 ax_cmd, ay_cmd는 world 좌표(고정 좌표) 기준임.
    // 만약 드론이 yaw로 돌아가 있으면(드론이 현재 회전한 상태라면), “world x 방향으로 가속하라”는 명령을 기체 관점에서 다시 해석됨.
    // 그래서 yaw만 제거한 frame(heading frame)으로 가속도를 바꿈.
    // yaw를 고려한 heading frame로 변환 후 소각 근사.
    // =====================
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // world accel -> heading frame (yaw만 제거한 frame)
    // world에서 본 가속도 벡터(ax, ay)를 드론이 바라보는 방향(heading)에 맞춘 좌표로 돌려놓음.
    // ax_h: 드론 “앞/뒤 방향”으로 필요한 가속도
    // ay_h: 드론 “좌/우 방향”으로 필요한 가속도
    const double ax_h =  cy*ax_cmd + sy*ay_cmd;
    const double ay_h = -sy*ax_cmd + cy*ay_cmd;
    // heading frame의 가속도 명령 -> 기울기 명령 (소각 근사(small angle approximation))
    // 부호는 좌표계 정의에 따라 달라질 수 있음.
    double pitch_ref =  ax_h / params.g;
    double roll_ref  = -ay_h / params.g;
    // 최대 기울기각 제한
    const double max_tilt = gains.max_tilt_deg * M_PI / 180.0;
    pitch_ref = clamp(pitch_ref, -max_tilt, max_tilt);
    roll_ref  = clamp(roll_ref,  -max_tilt, max_tilt);
    // yaw는 입력 ref 사용
    const double yaw_ref = ref.yaw_ref;

    // =====================
    // 5) Attitude PD -> Moment (간단한 버전)
    // 위 4)의 기울기 명령을 따라가기 위한 모멘트(토크) 명령 생성.
    // yaw는 항상 짧은 방향의 오차가 되도록 함.
    // =====================
    const double e_roll  = roll_ref  - roll;
    const double e_pitch = pitch_ref - pitch;
    const double e_yaw   = wrap_pi(yaw_ref - yaw);

    // body rates D항(s.w: body frame)
    const double p_rate = s.w.x;
    const double q_rate = s.w.y;
    const double r_rate = s.w.z;
    // PD 모멘트 명령.
    // P항: 목표 각도(roll_ref/pitch_ref/yaw_ref)로 맞추려는 힘.
    // D항: 회전 속도를 감쇠시켜 진동/오버슈트를 줄임.
    const double Mx = gains.kp_att_rp  * e_roll  - gains.kd_att_rp  * p_rate;
    const double My = gains.kp_att_rp  * e_pitch - gains.kd_att_rp  * q_rate;
    const double Mz = gains.kp_att_yaw * e_yaw   - gains.kd_att_yaw * r_rate;
    // body 기준 roll, pitch, yaw 모멘트 명령, 6-DOF 모델에 입력으로 사용.
    u.moment_body = {Mx, My, Mz};

    // =====================
    // 6) 추력 계산 + 기울기 보상(tilt compensation)
    // 드론이 기울어지면 추력 벡터는 body z축 방향으로 나가고 world z 방향 성분은 줄어듦.
    // 따라서 그만큼 추력을 더 키워줘야 고도 유지.
    // Thrust는 z-up world 가정.
    // =====================
    double T = params.mass * (params.g + az_cmd);

    const double c_r = std::cos(roll);
    const double c_p = std::cos(pitch);
    double denom = clamp(c_r * c_p, 0.2, 1.0); // 분모가 너무 작아져 값이 무한대가 되는 현상 방지한 안정장치
    T = T / denom;
    // thrust saturation
    T = clamp(T, gains.thrust_min, gains.thrust_max); // 오버슈트 감소용
    // body frame 기준 추력 벡터, z축 방향으로만 추력 작용, 쿼터니언을 통해 world로 변환됨.
    u.thrust_body = {0.0, 0.0, T};

    return u;

    return controller_update_dbg(s, ref, params, gains).u;
}