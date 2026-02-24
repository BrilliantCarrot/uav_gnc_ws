#pragma once
#include <cmath>

// =====================
// x, y, z 3차원 벡터
// =====================
struct Vec3 {
    double x{0}, y{0}, z{0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }
    static double dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// =====================
// 쿼터니언 (w, x, y, z)
// q * v_body * q_conj. 같이 바디에서 지구로의 회전을 표현
// =====================
struct Quat {
    double w{1}, x{0}, y{0}, z{0};

    Quat() = default;
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator+(const Quat& o) const { return {w + o.w, x + o.x, y + o.y, z + o.z}; }
    Quat operator*(double s) const { return {w*s, x*s, y*s, z*s}; }

    static Quat multiply(const Quat& a, const Quat& b) {
        
        return {
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
            a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
        };
    }

    Quat conj() const { return {w, -x, -y, -z}; } // 역회전 함수

    void normalize() {
        const double n = std::sqrt(w*w + x*x + y*y + z*z);
        if (n > 0) { w /= n; x /= n; y /= n; z /= n; }
        else { w = 1; x = y = z = 0; }
    } // 수치적분 오차 해결 정규화

    Vec3 rotateBodyToWorld(const Vec3& v_body) const {
        // v_world = q * [0,v] * q_conj
        Quat vq{0, v_body.x, v_body.y, v_body.z};
        Quat out = multiply(multiply(*this, vq), this->conj());
        return {out.x, out.y, out.z};
    } // 바디에서 지구로 회전(바디 힘/속도/축 벡터를 월드로)

    Vec3 rotateWorldToBody(const Vec3& v_world) const {
        // v_body = q_conj * [0,v] * q
        Quat vq{0, v_world.x, v_world.y, v_world.z};
        Quat out = multiply(multiply(this->conj(), vq), *this);
        return {out.x, out.y, out.z};
    } // 지구에서 바디로 회전(월드에 있는 벡터를 기체 기준으로 변환)
};

// =====================
// State & derivative
// =====================
struct State {
    Vec3 p;   // world 위치 (m)
    Vec3 v;   // world 속도 (m/s)
    Quat q;   // body->world attitude
    Vec3 w;   // body angular rate (rad/s), expressed in body frame
};
struct Deriv {
    Vec3 dp; //속도
    Vec3 dv; //가속도
    Quat dq; // 쿼터니언 시간 변화율
    Vec3 dw; //각가속도
};
struct Params { // 기체 물성치
    double mass{2};          // kg(연구용 소형 드론)
    Vec3 inertia{0.02, 0.02, 0.04};
    // diagonal inertia (kg*m^2) [Ix,Iy,Iz], mass와 arm length^2의 곱
    double g{9.80665};         // m/s^2 (z-up world: gravity is -z)
    // Optional drag in world frame: F_drag = - (k1*v + k2*|v|*v)
    bool use_drag{true}; // 항력 모델
    double k1{0.15};
    double k2{0.02};
    Vec3 wind_force{0.0, 0.0, 0.0}; // 추가, World Frame 기준 바람(외란) 힘 (N)
};
struct Input { // 제어 입력
    Vec3 thrust_body;  // N (body 좌표계 기준 힘 추력 벡터)
    Vec3 moment_body;  // N*m (body 프레임, Roll Pitch Yaw 토크(N*m) 벡터)
};

// 상태 미분 계산 (Newton-Euler)
Deriv derivatives(const State& s, const Input& u, const Params& p);

// RK4 적분기 선언
State rk4_step(const State& s, const Input& u, const Params& p, double dt);