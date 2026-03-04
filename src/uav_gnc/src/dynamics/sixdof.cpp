#include "uav_gnc/sixdof.h"

// =====================
// derivatives
// “6-DOF 강체를 뉴턴–오일러 방정식으로 적분하기 위해 상태(state)의 시간미분(derivative)을 계산
// RK4가 한 스텝 앞으로 진행하려면 ṗ, v̇, q̇, ω̇가 필요, derivatives()를 통해 생성.
// p, v는 world frame, q, ω, 제어입력은 body frame
// 힘은 적분 전 world frame으로 변환 필요, 토크는 body frame에서 계산
// =====================
static Vec3 invI_times(const Vec3& I, const Vec3& tau) {
    return { tau.x / I.x, tau.y / I.y, tau.z / I.z }; // 대각 관성 행렬의 역행렬 곱(각가속도 ẇ 계산용)
}
// 상태 미분값 계산하여 반환(뉴턴-오일러)
// s: 현재 상태, u: 제어 입력, p: 기체 물성치
Deriv derivatives(const State& s, const Input& u, const Params& p) {
    // state는 위치, 속도, 자세(body->world 쿼터니언), 각속도
    Deriv d;
    // Position derivative
    d.dp = s.v;
    // Forces, Thrust in world frame
    // thrust_body는 “기체 기준 body frame의 추력
    // 그런데 위치/속도는 world frame에서 적분
    // 따라서 추력도 world frame으로 변환 필요
    const Vec3 thrust_world = s.q.rotateBodyToWorld(u.thrust_body);
    // world에서 위 방향이+(z-up)이므로 -g, a_g = (0,0,-g)
    const Vec3 gravity{0.0, 0.0, -p.g};

    // Drag계산용 항력 모델 함수 (world frame)
    Vec3 a_drag{0,0,0};
    if (p.use_drag) {
        const double vnorm = s.v.norm();
        const Vec3 Fd = (-p.k1)*s.v + (-p.k2)*vnorm*s.v; // 저속 선형 항력과 중·고속의 제곱 항력 모사, world frame서 계산
        a_drag = Fd / p.mass;
    }
    
    // 최종적인 선형 가속도, 뉴턴 제2법칙의 가속도 형태(뉴턴 병진 운동), 외란 없음
    // d.dv = (thrust_world / p.mass) + gravity + a_drag;

    // 강건함 테스트: 바람에 의한 가속도 (F = ma -> a = F/m)
    Vec3 a_wind = p.wind_force / p.mass; 
    // 최종적인 선형 가속도에 바람 가속도 추가
    d.dv = (thrust_world / p.mass) + gravity + a_drag + a_wind;

    // 자세 미분
    // Quaternion derivative: q_dot = 0.5 * q ⊗ omega_quat
    // omega_quat = [0, w_body](스칼라가 0 + 벡터 w)
    Quat omega{0.0, s.w.x, s.w.y, s.w.z};
    d.dq = Quat::multiply(s.q, omega) * 0.5;

    // 회전 동역학 (body frame):
    // I*w_dot + w x (I*w) = tau  => w_dot = I^{-1} ( tau - w x (I*w) )
    const Vec3 Iw{ p.inertia.x * s.w.x, p.inertia.y * s.w.y, p.inertia.z * s.w.z }; // I*w 계산
    const Vec3 wxIw = Vec3::cross(s.w, Iw); // w x (I*w) 계산(오일러 회전 운동)
    const Vec3 rhs = u.moment_body - wxIw; // 우변 계산: tau - w x (I*w)
    d.dw = invI_times(p.inertia, rhs); // 최종 각가속도 계산

    return d;
}

// =====================
// RK4 적분기 설정, 상태를 dt만큼 한 스텝 앞으로 진행
// =====================
static State add_scaled(const State& s, const Deriv& k, double h) { // k2, k3, k4 상태를 만드는 헬퍼 함수
    State out = s;
    out.p += k.dp * h;
    out.v += k.dv * h;
    out.q = out.q + (k.dq * h);
    out.w += k.dw * h;
    return out;
}

State rk4_step(const State& s, const Input& u, const Params& p, double dt) {
    const Deriv k1 = derivatives(s, u, p); // 시작점 기울기
    const Deriv k2 = derivatives(add_scaled(s, k1, dt*0.5), u, p); // 중간점 기울기1, s에서 k1 방향으로 dt/2만큼 이동한 상태
    const Deriv k3 = derivatives(add_scaled(s, k2, dt*0.5), u, p); // 중간점 기울기2, s에서 k2 방향으로 dt/2만큼 이동한 상태
    const Deriv k4 = derivatives(add_scaled(s, k3, dt), u, p); // 끝점 기울기

    State out = s;
    out.p += (k1.dp + 2.0*k2.dp + 2.0*k3.dp + k4.dp) * (dt/6.0);
    out.v += (k1.dv + 2.0*k2.dv + 2.0*k3.dv + k4.dv) * (dt/6.0);
    out.q = out.q + (k1.dq + (k2.dq*2.0) + (k3.dq*2.0) + k4.dq) * (dt/6.0);
    out.w += (k1.dw + 2.0*k2.dw + 2.0*k3.dw + k4.dw) * (dt/6.0);

    // drift 방지를 위한 쿼터니언 정규
    out.q.normalize();
    return out;
}


