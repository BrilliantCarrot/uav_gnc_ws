# ✈️ uav_gnc_ws: Autonomous Drone GNC Framework

> **"비행 동역학 모델링부터 최적화 기반 유도 제어까지, 자율 비행의 전 과정을 구현합니다."**
> <br> 본 프로젝트는 ROS2와 PX4를 활용하여 임무별 무인기에 최적화된 유도(Guidance), 항법(Navigation), 제어(Control) 시스템을 설계하고 검증하는 통합 프레임워크입니다.

## 🎯 Project Overview
임무 요구사항에 따른 자율 비행 시스템 아키텍처를 설계하고, 6-DOF 시뮬레이션을 통해 알고리즘의 안정성을 검증합니다. 실제 비행체 통합을 고려하여 Modern C++과 ROS2를 기반으로 개발되었습니다.



## 🚀 Key Features
* **6-DOF Dynamics Modeling:** 외부 라이브러리 의존 없이 직접 구현한 드론 6자유도 동역학 모델 기반 시뮬레이션 환경 구축
* **State Estimation (EKF):** IMU 및 GPS 데이터를 융합하여 센서 노이즈 환경에서도 정밀한 상태 추정 수행
* **Optimal Trajectory Generation:** Minimum Snap 기반의 궤적 최적화를 통해 부드럽고 효율적인 경로 생성
* **Cascaded Control:** Position - Velocity - Attitude로 이어지는 계층적 제어 구조 설계
* **ROS2 Software Architecture:** 멀티스레딩 및 Node 기반 설계를 통해 인지/항법/제어 모듈 간 데이터 통신 최적화

## 🛠️ Tech Stack
* **Languages:** C++ (Modern C++), Python (for plotting & analysis)
* **Frameworks:** ROS2 Humble, PX4 Autopilot, MAVLink
* **Tools:** Gazebo, RViz2, MATLAB/Simulink (Logic Verification)
* **Algorithms:** EKF, Trajectory Optimization, 6-DOF Physics Engine