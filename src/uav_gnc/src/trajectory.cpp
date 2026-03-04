#include "uav_gnc/trajectory.h"
#include <iostream>
#include <cmath>

// ======================================================================
// 1. MinJerkTrajectory 구현 (5차 다항식)
// ======================================================================
MinJerkTrajectory::MinJerkTrajectory() {}

void MinJerkTrajectory::generate(double p0, double v0, double a0, 
                                 double pf, double vf, double af, double T) {
    T_ = T; // 총 이동 시간 저장
    
    // [초기 조건 대입] t=0 일 때의 위치, 속도, 가속도를 통해 c0, c1, c2는 바로 구해짐
    c_[0] = p0; 
    c_[1] = v0; 
    c_[2] = 0.5 * a0;

    // A: 시간에 대한 행렬
    // x: 구하려는 계수(c0~cn 등)
    // B: 목표로 하는 상태(웨이포인트 위치 등)
    // [도착 조건 행렬 세팅] t=T 일 때의 위치, 속도, 가속도를 맞추기 위해 3x3 행렬 A를 만듬
    // 1행 위치, 2행 속도, 3행 가속도
    Eigen::Matrix3d A;
    A << pow(T, 3),   pow(T, 4),    pow(T, 5),     // 위치 방정식 (c3*T^3 + c4*T^4 + c5*T^5)
         3*pow(T, 2), 4*pow(T, 3),  5*pow(T, 4),   // 속도 방정식 (위치 미분)
         6*T,         12*pow(T, 2), 20*pow(T, 3);  // 가속도 방정식 (속도 미분)

    // 도착해야 할 목표값(pf, vf, af)에서 이미 결정된 초기 조건(c0, c1, c2)이 T초 동안 만들어낸 값을 빼준 나머지 값(B), 남은 목표치를 의미
    // 1행: 위치 오차, 초기 위치 + 초기 속도 이동량 + 초기 가속도 이동량을 뺀 값
    // 2행: 속도 오차, 목표 속도에서 초기 속도 + 초기 가속도로 변한 속도를 뺀 값
    // 3행: 가속도 오차, 목표 가속도에서 초기 가속도를 뺀 변화량
    Eigen::Vector3d B;
    B << pf - p0 - v0*T - 0.5*a0*pow(T, 2),
         vf - v0 - a0*T,
         af - a0;

    // A * x = B 방정식 풀이 (x = A의 역행렬 * B)
    // colPivHouseholderQr()는 일반적인 역행렬(inverse)보다 수치적으로 훨씬 안정적이고 빠른 Eigen의 해법
    // colPivHouseholderQr()는 C++의 선형 대수 라이브러리인 Eigen에서 제공하는 함수로, 행렬의 QR 분해(QR Decomposition)를 수행할 때 사용됨
    // "Column-Pivoting Householder QR Decomposition"
    // 주요 특징: "피벗팅(Pivoting)"
    // 일반적인 QR 분해와 달리 열 피벗팅(Column Pivoting) 기술을 사용
    // 계산 과정에서 독립적인 열(column)을 우선적으로 선택하여 정렬
    // 이 과정 덕분에 일반 QR 분해보다 수치적으로 훨씬 안정적
    // 행렬의 계수(Rank)를 구하는 데 매우 탁월
    // 주로 다음과 같은 상황에서 householderQr() 대신 이 함수를 선택
    // Rank-deficient 행렬 처리: 행렬의 열들이 서로 선형 종속일 가능성이 있을 때(즉, Full Rank가 아닐 때)
    // 최소자승법(Least Squares): Ax=b의 문제에서, A의 상태가 좋지 않을 때 해를 안정적으로 구하기 위해
    // 행렬의 Rank 계산: rank() 함수를 호출하여 행렬의 유효한 차원을 알고 싶을 때
    // 속도: 피벗팅 과정이 추가되므로 일반 householderQr()보다는 약간 느림
    // 정확도: 수치적 오차에 매우 강하며, 특히 행렬이 "특이 행렬(Singular Matrix)"에 가까울 때 훨씬 정확한 결과를 줍니다.
    // 속도가 최우선이고 행렬이 확실히 Full Rank라면 householderQr(), 정확도와 안정성이 중요하다면 colPivHouseholderQr()를 사용
    // Eigen 라이브러리에서 A의 역행렬을 구해 B랑 곱해줌으로써 미지수를 찾아내어 궤적 생성 마무리
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(B);
    
    // 남은 계수 c3, c4, c5 저장 완료!
    c_[3] = x(0); c_[4] = x(1); c_[5] = x(2);
}

// t초일 때의 위치 계산 (다항식 전개), 특정 시점 t에서 드론이 위치해야 할 정답(setpoint)을 계산
double MinJerkTrajectory::getPosition(double t) const { // 현재 위치 계산
    if (t < 0.0) t = 0.0; // 시간 클램핑 안전 장치
    if (t > T_) t = T_; // T_를 넘어가면 마지막 도착 위치 고정, 비행이 끝난 후에도 드론이 도착 지점에 고정 되도록 함
    return c_[0] + c_[1]*t + c_[2]*pow(t, 2) + c_[3]*pow(t, 3) + c_[4]*pow(t, 4) + c_[5]*pow(t, 5); // 5차 다항식 코드
}

// t초일 때의 속도 계산 (위치 다항식을 1번 미분한 형태)
// 제어기(control_node)에 목표 속도를 알려주는 속도 피드포워드 값을 생성
double MinJerkTrajectory::getVelocity(double t) const { // 현재 목표 속도 계산
    if (t < 0.0) t = 0.0;
    if (t > T_) t = T_;
    return c_[1] + 2*c_[2]*t + 3*c_[3]*pow(t, 2) + 4*c_[4]*pow(t, 3) + 5*c_[5]*pow(t, 4); // 위치 다항식을 시간 t에 대해 한번 미분
}

// ======================================================================
// 2. MinSnapTrajectory 구현 (7차 다항식)
// 이미 결정된 앞부분 계수(c0, c1, c2, c3)를 바탕으로 나머지 계수들을 구하기 위해 선형 연립 방정식 Ax = B를 설정
// 사용중인 7차 다항식 위치 공식은: (도착 시간 t=T 일 때)p(T) = c_0 + c_1T + c_2T^2 + c_3T^3 + c_4T^4 + c_5T^5 + c_6T^6 + c_7T^7 = pf
// c_4, c_5, c_6, c_7은 우리가 역행렬을 풀어서 구해야 하는 모르는 값
// ======================================================================
MinSnapTrajectory::MinSnapTrajectory() {}

void MinSnapTrajectory::generate(double p0, double v0, double a0, double j0,
                                 double pf, double vf, double af, double jf, double T) {
    T_ = T;
    
    // t=0 일 때의 초기 조건으로 c0, c1, c2, c3 확정
    c_[0] = p0; 
    c_[1] = v0; 
    c_[2] = 0.5 * a0; 
    c_[3] = (1.0 / 6.0) * j0;

    // t=T 일 때의 조건(위치, 속도, 가속도, Jerk)을 맞추기 위한 4x4 행렬
    // 행렬 A: 목표 시점(T)에서의 미분 값, 목표 시간 T에서듼 c4 부터 c7까지의 항들에 대한 미분 게수들을 담고있음
    // 1행: 위치, 2행: 속도, 3행: 가속도, 4행: 저크
    Eigen::Matrix4d A;
    A << pow(T, 4),    pow(T, 5),     pow(T, 6),      pow(T, 7),
         4*pow(T, 3),  5*pow(T, 4),   6*pow(T, 5),    7*pow(T, 6),
         12*pow(T, 2), 20*pow(T, 3),  30*pow(T, 4),   42*pow(T, 5),
         24*T,         60*pow(T, 2),  120*pow(T, 3),  210*pow(T, 4);
    // 목표 상태와 현재까지 확정된 항들에 의한 상태 간 차이를 계산
    // 도착할 때 필요한 나머지 채우기
    // 목표 지점(도착점)에 도착했을 때 달성해야 하는 목표 위치가 100(pf)이라고 하자
    // 그런데 드론이 처음 출발할 때 가진 조건(초기 위치, 초기 속도 등) 때문에 이미 도착 시간에 30만큼은 도달해 있는 상태인 거
    // 그렇다면 추가로 만들어내야 하는(채워야 하는) 값은 100 - 30 = 70이 됨
    // B 벡터가 바로 "최종 목표값 - 초기 조건들이 만들어낸 값 = 우리가 앞으로 행렬을 풀어 추가로 채워야 할 나머지 값"을 계산하는 코드
    // 이 수식을 "모르는 값 = 아는 값" 형태로 좌우를 넘겨서 정리해 보면 
    // c_4T^4 + c_5T^5 + c_6T^6 + c_7T^7 = pf - (c_0 + c_1T + c_2T^2 + c_3T^3)
    // 우변이 완전히 똑같으므로, 이것이 밑에 B 벡터 첫 째 줄
    // 두 번째 줄 (속도, Velocity): 위치 공식을 1번 미분한 것. 목표 속도(vf)에서, 기존 계수들이 도착 시간에 만들어낸 속도를 뺀 나머지
    // 세 번째 줄 (가속도, Acceleration): 위치 공식을 2번 미분한 것. 목표 가속도(af)에서, 기존 계수들이 만들어낸 가속도를 뺀 나머지
    // 네 번째 줄 (가가속도, Jerk): 위치 공식을 3번 미분한 것. 목표 가가속도(jf)에서, 기존 계수들이 만들어낸 Jerk를 뺀 나머지
    Eigen::Vector4d B;
    B << pf - (c_[0] + c_[1]*T + c_[2]*pow(T, 2) + c_[3]*pow(T, 3)),
         vf - (c_[1] + 2*c_[2]*T + 3*c_[3]*pow(T, 2)),
         af - (2*c_[2] + 6*c_[3]*T),
         jf - (6*c_[3]);

    // 계수 c4, c5, c6, c7 계산
    Eigen::Vector4d x = A.colPivHouseholderQr().solve(B);
    c_[4] = x(0); c_[5] = x(1); c_[6] = x(2); c_[7] = x(3);
}

double MinSnapTrajectory::getPosition(double t) const {
    if (t < 0.0) t = 0.0;
    if (t > T_) t = T_;
    return c_[0] + c_[1]*t + c_[2]*pow(t, 2) + c_[3]*pow(t, 3) + 
           c_[4]*pow(t, 4) + c_[5]*pow(t, 5) + c_[6]*pow(t, 6) + c_[7]*pow(t, 7);
}

double MinSnapTrajectory::getVelocity(double t) const {
    if (t < 0.0) t = 0.0;
    if (t > T_) t = T_;
    return c_[1] + 2*c_[2]*t + 3*c_[3]*pow(t, 2) + 4*c_[4]*pow(t, 3) + 
           5*c_[5]*pow(t, 4) + 6*c_[6]*pow(t, 5) + 7*c_[7]*pow(t, 6);
}


// ======================================================================
// 3. MultiMinSnapTrajectory 구현 (다중 구간 7차 다항식)
// ======================================================================
MultiMinSnapTrajectory::MultiMinSnapTrajectory() {}

void MultiMinSnapTrajectory::generate(const std::vector<double>& waypoints, const std::vector<double>& times) {
    int N = times.size(); // 총 구간의 개수
    if (N == 0 || waypoints.size() != static_cast<size_t>(N + 1)) return;

    // 구간당 8개의 계수가 필요하므로, 전체 풀어야 할 변수의 개수는 8 * N
    int num_vars = 8 * N; 
    
    // 거대한 행렬 A (8N x 8N)와 벡터 B (8N x 1)를 0으로 초기화
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_vars, num_vars);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(num_vars);
    
    int row = 0; // 행렬 A에 식을 한 줄씩 써넣을 때 사용할 인덱스

    // ---------------------------------------------------------
    // 제약조건 1. 맨 처음 출발점 (t=0)
    // 위치는 첫 Waypoint이고 속도, 가속도, Jerk는 모두 0에서 출발
    // ---------------------------------------------------------
    A(row, 0) = 1.0; B(row++) = waypoints[0]; // c0 = 출발 위치
    A(row, 1) = 1.0; B(row++) = 0.0;          // c1 = 초기 속도 0
    A(row, 2) = 2.0; B(row++) = 0.0;          // 2*c2 = 초기 가속도 0
    A(row, 3) = 6.0; B(row++) = 0.0;          // 6*c3 = 초기 Jerk 0

    // ---------------------------------------------------------
    // 제약조건 2. 중간 경유지들 (연속성 보장)
    // i번째 구간이 끝나는 시간(T)에서의 위치와, i+1번째 구간이 시작하는 시간(0)에서의 위치가 
    // 모두 같은 중간 웨이포인트(pi)여야 함
    // ---------------------------------------------------------
    for (int i = 0; i < N - 1; ++i) {
        double T = times[i]; // 현재 i번째 구간을 이동하는 데 걸리는 시간
        int col_i = i * 8;          // 현재 구간의 계수(c0~c7)가 시작되는 열 인덱스
        int col_next = (i + 1) * 8; // 다음 구간의 계수(c0~c7)가 시작되는 열 인덱스

        // (1) i번째 구간이 끝나는 시간(T)에서의 위치 = 다음 Waypoint
        A(row, col_i+0)=1.0; A(row, col_i+1)=T; A(row, col_i+2)=pow(T,2); A(row, col_i+3)=pow(T,3);
        A(row, col_i+4)=pow(T,4); A(row, col_i+5)=pow(T,5); A(row, col_i+6)=pow(T,6); A(row, col_i+7)=pow(T,7);
        B(row++) = waypoints[i+1];

        // (2) i+1번째 구간이 시작하는 시간(t=0)에서의 위치 = 다음 Waypoint
        A(row, col_next+0) = 1.0; 
        B(row++) = waypoints[i+1];

        // (3) 핵심 파트: 미분값들의 연속성 보장 (브레이크를 밟지 않고 매끄럽게 통과하기 위함)
        // 앞 구간이 끝나는 순간 물리량과 뒷 구간이 시작하는 순간 물리량이 완벽하게 일치하여야 함.
        // 수식 원리: [i번째 구간 끝에서의 속도/가속도 등] - [i+1번째 구간 시작점의 속도/가속도 등] = 0

        // i번째 구간의 t=T에서의 미분값들 (A 행렬의 i번째 블록)
        // A(row, col_i+1)=1.0; A(row, col_i+2)=2*T; ... 
        // i+1번째 구간의 t=0에서의 미분값 (A 행렬의 i+1번째 블록)을 뺌
        // A(row, col_next+1) = -1.0; 
        // 두 값의 차이는 0이어야 함 (연속성 보장)
        // B(row++) = 0.0; 
        // 밑의 결과를 통해 i번 구간이 끝나는 직전 속도와 i+1번 구간 시작 속도가 똑같이 같게 된다
        
        // 속도 연속성
        A(row, col_i+1)=1.0; A(row, col_i+2)=2*T; A(row, col_i+3)=3*pow(T,2); A(row, col_i+4)=4*pow(T,3);
        A(row, col_i+5)=5*pow(T,4); A(row, col_i+6)=6*pow(T,5); A(row, col_i+7)=7*pow(T,6);
        A(row, col_next+1) = -1.0; B(row++) = 0.0;

        // 가속도 연속성
        A(row, col_i+2)=2.0; A(row, col_i+3)=6*T; A(row, col_i+4)=12*pow(T,2); A(row, col_i+5)=20*pow(T,3);
        A(row, col_i+6)=30*pow(T,4); A(row, col_i+7)=42*pow(T,5);
        A(row, col_next+2) = -2.0; B(row++) = 0.0;

        // Jerk(3차) 연속성
        A(row, col_i+3)=6.0; A(row, col_i+4)=24*T; A(row, col_i+5)=60*pow(T,2); A(row, col_i+6)=120*pow(T,3); A(row, col_i+7)=210*pow(T,4);
        A(row, col_next+3) = -6.0; B(row++) = 0.0;

        // Snap(4차) 연속성
        A(row, col_i+4)=24.0; A(row, col_i+5)=120*T; A(row, col_i+6)=360*pow(T,2); A(row, col_i+7)=840*pow(T,3);
        A(row, col_next+4) = -24.0; B(row++) = 0.0;

        // Crackle(5차) 연속성
        A(row, col_i+5)=120.0; A(row, col_i+6)=720*T; A(row, col_i+7)=2520*pow(T,2);
        A(row, col_next+5) = -120.0; B(row++) = 0.0;

        // Pop(6차) 연속성
        A(row, col_i+6)=720.0; A(row, col_i+7)=5040*T;
        A(row, col_next+6) = -720.0; B(row++) = 0.0;
    }

    // ---------------------------------------------------------
    // 제약조건 3. 맨 마지막 도착점
    // 위치는 최종 Waypoint이고 속도, 가속도, Jerk는 모두 0으로 멈춤
    // ---------------------------------------------------------
    double T_end = times[N - 1]; // 마지막 구간의 소요 시간
    int col_last = (N - 1) * 8;  // 마지막 구간 계수 시작 위치

    // 최종 위치
    A(row, col_last+0)=1.0; A(row, col_last+1)=T_end; A(row, col_last+2)=pow(T_end,2); A(row, col_last+3)=pow(T_end,3);
    A(row, col_last+4)=pow(T_end,4); A(row, col_last+5)=pow(T_end,5); A(row, col_last+6)=pow(T_end,6); A(row, col_last+7)=pow(T_end,7);
    B(row++) = waypoints[N];

    // 최종 속도 = 0
    A(row, col_last+1)=1.0; A(row, col_last+2)=2*T_end; A(row, col_last+3)=3*pow(T_end,2); A(row, col_last+4)=4*pow(T_end,3);
    A(row, col_last+5)=5*pow(T_end,4); A(row, col_last+6)=6*pow(T_end,5); A(row, col_last+7)=7*pow(T_end,6);
    B(row++) = 0.0;

    // 최종 가속도 = 0
    A(row, col_last+2)=2.0; A(row, col_last+3)=6*T_end; A(row, col_last+4)=12*pow(T_end,2); A(row, col_last+5)=20*pow(T_end,3);
    A(row, col_last+6)=30*pow(T_end,4); A(row, col_last+7)=42*pow(T_end,5);
    B(row++) = 0.0;

    // 최종 Jerk = 0
    A(row, col_last+3)=6.0; A(row, col_last+4)=24*T_end; A(row, col_last+5)=60*pow(T_end,2); A(row, col_last+6)=120*pow(T_end,3); A(row, col_last+7)=210*pow(T_end,4);
    B(row++) = 0.0;

    // ---------------------------------------------------------
    // 행렬 풀이 (8N 개의 계수를 한 번에 계산!)
    // 위 단게까지는 A, B를 채움
    // 밑 단계에서 실제 Ax=B 연립방정식을 풀어 c0, c1, ..., c8N-1 등 다항식 계수들을 찾음
    // A(Knowns/Rules): 궤적의 미분 규칙과 시간(T) 정보가 담긴 8N x 8N 행렬
    // B(Targets): 가야 할 웨이포인트 좌표와 "오차는 0 이어야 함"이라는 목표가 담긴 벡터
    // x(Unknowns): 진짜 알고 싶은 다항식 계수들, coeffs_
    // ---------------------------------------------------------
    coeffs_ = A.colPivHouseholderQr().solve(B);
    times_ = times; // 나중에 getPosition에서 쓸 수 있도록 구간 시간들 저장
}

// 전체 시간 t일 때 위치 구하기
double MultiMinSnapTrajectory::getPosition(double t) const {
    if (times_.empty()) return 0.0;
    
    // 현재 t가 몇 번째 구간에 속하는지 찾음
    int idx = getSegmentIndex(t); 
    
    // 해당 구간의 시작 시간을 빼서, 구간 내에서의 로컬 시간(t_local)으로 변환
    double t_local = t - getStartTime(idx); 
    
    // 해당 구간의 계수가 저장된 인덱스
    int c = idx * 8; 
    
    // 로컬 시간을 이용해 7차 다항식 전개
    return coeffs_(c) + coeffs_(c+1)*t_local + coeffs_(c+2)*pow(t_local,2) + coeffs_(c+3)*pow(t_local,3) +
           coeffs_(c+4)*pow(t_local,4) + coeffs_(c+5)*pow(t_local,5) + coeffs_(c+6)*pow(t_local,6) + coeffs_(c+7)*pow(t_local,7);
}

// 전체 시간 t일 때 속도 구하기
double MultiMinSnapTrajectory::getVelocity(double t) const {
    if (times_.empty()) return 0.0;
    int idx = getSegmentIndex(t); // 현재 전체 시간 t가 몇 번째 구간에 속하는지 인덱스를 찾음
    double t_local = t - getStartTime(idx); // 전체 시간에서 해당 구간이 시작된 시간을 빼서, 구 구간 안에서 흐른 시간으로 바꿈
    // 예를 들어, 3번째 구간이 전체 시간 5초에 시작했고 현재가 6.5초라면, t_local은 1.5초가 됨
    
    int c = idx * 8; // 계수 위치 찾기
    // 각 구간은 7차 다항식을 사용하므로 한 구간당 8개의 계수(c_0 ~ c_7)를 가짐
    // 현재 구간(idx)의 계수들이 거대한 계수 벡터(coeffs_)의 어디서부터 시작되는지 그 주소(인덱스)를 계산하는 것
    // coeffs_는 모든 구간의 계수를 한 줄로 길게 세워놓은 형태이기 때문에, 특정 구간의 정보를 쓰려면 그 구간의 데이터가 시작되는 '시작 주소(Offset)'를 알아야 함
    // 우리가 원하는 구간(idx)의 계수가 시작되는 번호를 찾으려면, "앞에 있는 구간들이 차지한 칸수만큼 건너뛰어야" 함
    // 0번 구간을 보고 싶을 때: 앞에 건너뛸 구간이 0개, -> 0 x 8 = 0번부터 시작
    // 1번 구간을 보고 싶을 때: 앞에 0번 구간(8칸)을 건너뛰어야 함 -> 1 x 8 = 8번부터 시작
    // 2번 구간을 보고 싶을 때: 앞에 0번, 1번 구간(총 16칸)을 건너뛰어야 함 -> 2 x 8 = 16번부터 시작
    // 따라서 임의의 idx번째 구간의 시작 위치 c는 항상 idx x 8이 되는 수학적 규칙이 생기는 것

    // 반환값은 7차 위치 다항식을 한 번 미분한 결과
    return coeffs_(c+1) + 2*coeffs_(c+2)*t_local + 3*coeffs_(c+3)*pow(t_local,2) + 4*coeffs_(c+4)*pow(t_local,3) +
           5*coeffs_(c+5)*pow(t_local,4) + 6*coeffs_(c+6)*pow(t_local,5) + 7*coeffs_(c+7)*pow(t_local,6);
}

// 전체 비행 소요 시간 반환
double MultiMinSnapTrajectory::getTotalTime() const {
    double t = 0;
    for (double dt : times_) t += dt;
    return t;
}

// 유틸리티: 현재 시간 t가 속한 구간의 인덱스 찾기
int MultiMinSnapTrajectory::getSegmentIndex(double t) const {
    double accumulated_t = 0.0;
    for (size_t i = 0; i < times_.size(); ++i) {
        accumulated_t += times_[i];
        if (t <= accumulated_t) return i; // 누적 시간보다 작으면 이 구간임
    }
    return times_.size() - 1; // t가 초과하면 마지막 구간 인덱스 반환
}

// 유틸리티: 특정 구간의 시작 시간 찾기
double MultiMinSnapTrajectory::getStartTime(int idx) const {
    double start_t = 0.0;
    for (int i = 0; i < idx; ++i) start_t += times_[i];
    return start_t;
}