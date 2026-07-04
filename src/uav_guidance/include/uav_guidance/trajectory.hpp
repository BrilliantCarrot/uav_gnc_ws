#pragma once

#include <Eigen/Dense>
#include <vector>

// ======================================================================
// 1. MinJerkTrajectory (최소 가가속도 궤적 생성기)
// 목적: 점과 점 사이를 이동할 때 가가속도(Jerk)의 변화를 최소화하여 부드럽게 이동
// 모터 추력은 가속도와 관련있음
// 수학적 특징: 5차 다항식 (Quintic Polynomial) 사용
// ======================================================================
class MinJerkTrajectory {
public:
    MinJerkTrajectory();
    
    // 궤적의 계수를 계산(생성)하는 함수
    // p0, v0, a0: 출발점의 위치, 속도, 가속도
    // pf, vf, af: 도착점의 위치, 속도, 가속도
    // T: 이 구간을 이동하는 데 걸리는 총 시간
    void generate(double p0, double v0, double a0, double pf, double vf, double af, double T);
    
    // 특정 시간 t (0 <= t <= T)일 때, 드론이 있어야 할 '위치'를 반환
    double getPosition(double t) const;
    
    // 특정 시간 t일 때, 드론이 가져야 할 '속도'를 반환 (Feedforward 제어에 유용함)
    double getVelocity(double t) const;

private:
    double c_[6]{0.0}; // 5차 다항식의 계수 6개 (c0, c1, c2, c3, c4, c5)를 저장하는 배열
    double T_{0.0};    // 이 궤적이 끝나는 총 시간 (t가 T_를 넘어가면 궤적이 종료됨)
};


// ======================================================================
// 2. MinSnapTrajectory (최소 스냅 궤적 생성기)
// 목적: 쿼드콥터의 모터 토크 변화율(Snap)을 최소화하여 진동을 줄이며 가장 안정적인 비행을 유도
// 수학적 특징: 7차 다항식 (Septic Polynomial) 사용
// p(t)=c0​+c1​t+c2​t2+c3​t3+c4​t4+c5​t5+c6​t6+c7​t7
// 계수가 8개인 이유: 하나의 구간(Segment)를 정의할 때, 양 끝점(시작점과 도착점)에서 고정하고싶은 경계조건이 4개씩이기 때문
// 시작점(t=0): 위치, 속도, 가속도, 저크
// 도착점(t=T): 위치, 속도, 가속도, 저크
// 총 8개의 조건을 만족하는 유일한 해를 구하기 위해 8개 미지수가 필요
// ======================================================================
class MinSnapTrajectory {
public:
    MinSnapTrajectory();
    
    // 궤적 계수 계산 (Jerk 조건이 추가됨)
    // j0, jf: 출발점과 도착점의 가가속도(Jerk). 보통 0으로 설정하여 부드러운 출발/정지를 유도함
    void generate(double p0, double v0, double a0, double j0, 
                  double pf, double vf, double af, double jf, double T);
    
    // 특정 시간 t에서의 위치 반환
    double getPosition(double t) const;
    
    // 특정 시간 t에서의 속도 반환
    double getVelocity(double t) const;

private:
    double c_[8]{0.0}; // 7차 다항식의 계수 8개 (c0 ~ c7)를 저장하는 배열
    double T_{0.0};    // 총 소요 시간
};


// ======================================================================
// 3. MultiMinSnapTrajectory (다중 구간 최소 스냅 궤적 생성기)
// 목적: 여러 개의 Waypoint를 한 번에 주면, 중간 경유지에서 멈추지 않고 
//       속도와 가속도를 매끄럽게 유지하며 전체 경로의 Snap을 최소화함
// 수학적 특징: 구간이 N개라면, 7차 다항식 N개를 하나로 이어붙임 (총 8*N개의 계수 연산)
// ======================================================================
class MultiMinSnapTrajectory {
public:
    MultiMinSnapTrajectory();
    
    // 전체 궤적 생성
    // waypoints: 지나가야 할 모든 점들의 리스트 (예: x좌표들)
    // times: 각 점과 점 사이(구간)를 이동하는 데 할당된 시간들의 리스트
    void generate(const std::vector<double>& waypoints, const std::vector<double>& times);
    
    // 전체 비행 시작 후 시간 t가 흘렀을 때의 위치 반환
    double getPosition(double t) const;
    
    // 전체 비행 시작 후 시간 t가 흘렀을 때의 속도 반환
    double getVelocity(double t) const;
    
    // 전체 웨이포인트를 도는 데 걸리는 총 합산 시간 반환
    double getTotalTime() const;

private:
    // 시간 t가 현재 몇 번째 구간(Segment)을 지나고 있는지 찾아내는 헬퍼 함수
    // 예: 1구간이 2초, 2구간이 3초 걸리는데 t=4초라면, 현재는 2번째 구간임!
    int getSegmentIndex(double t) const;
    
    // 특정 구간(idx)이 시작되는 누적 시간을 반환
    double getStartTime(int idx) const;

    // 구간별로 계수가 8개씩 있으므로, 배열 대신 길이가 유동적인 Eigen 벡터를 사용해 계수를 몽땅 저장
    Eigen::VectorXd coeffs_; 
    
    // 각 구간별 소요 시간을 기억해두는 리스트
    std::vector<double> times_; 
};