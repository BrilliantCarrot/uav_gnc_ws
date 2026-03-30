# UAV GNC System — ROS2-based Autonomous Flight

> **End-to-end Guidance · Navigation · Control** system for unmanned aerial vehicles, built from scratch using ROS2 Humble and C++.

---

## Overview

This project implements a complete UAV GNC (Guidance, Navigation, and Control) pipeline entirely from scratch — without relying on PX4, ArduPilot, or any off-the-shelf autopilot stack. Every core module, from the 6-DOF physics simulator to the optimal controller, was designed and implemented independently.

**Key highlights:**
- 6-DOF Newton-Euler flight dynamics simulator with RK4 integration
- 15-State Error-State EKF fusing IMU (100 Hz) and GPS (10 Hz)
- Multi-Segment Minimum Snap trajectory optimization (8N × 8N matrix system)
- Cascaded PID controller with feedforward and integral disturbance rejection
- Linear MPC (Condensed formulation) with precomputed K_first — 100 Hz real-time
- Robustness testing under constant wind disturbance (1 N) and GPS noise (0.5 m)

---

## System Architecture

```
<img width="2931" height="587" alt="Image" src="https://github.com/user-attachments/assets/aa5ba3c1-1e75-44e6-8b9a-891a478f7f41" />
```

| Node | Responsibility | Rate |
|------|---------------|------|
| `simulation_node` | 6-DOF RK4 integrator, IMU/GPS noise injection, wind disturbance | 100 Hz |
| `navigation_node` | 15-State Error-State EKF (IMU + GPS sensor fusion) | 100 Hz predict / 10 Hz update |
| `guidance_node` | Multi-Segment Minimum Snap trajectory, Reference Preview for MPC | 20 Hz |
| `control_node` | Cascaded PID or Linear MPC outer loop + attitude PD inner loop | 100 Hz |
| `tracking_eval_node` | Real-time 3D Cross-Track Error, CSV logging | 20 Hz |

---

## Features

### Navigation — 15-State Error-State EKF
- **State vector:** position (3), velocity (3), attitude error δθ (3), accelerometer bias (3), gyroscope bias (3)
- **Prediction (100 Hz):** IMU-driven system model integration with Jacobian propagation
- **Update (10 Hz):** GPS position measurement, Kalman gain computation, quaternion error injection
- Numerical stability via error-state (δθ) representation — avoids quaternion singularities

### Guidance — Multi-Segment Minimum Snap
- Solves an **8N × 8N** constrained linear system (Eigen HouseholderQR) for N trajectory segments
- Guarantees continuity up to snap (4th derivative) at all intermediate waypoints — no stop-and-go behavior
- **Z-axis decoupled** to linear interpolation to prevent Runge's Phenomenon in altitude
- **Reference Preview** for MPC: publishes future N-step position/velocity array at 20 Hz

### Control — Cascaded PID
- Three-loop cascade: position → velocity → attitude
- Feedforward velocity (v_ref) and acceleration (a_ref) from guidance polynomial
- Integral term with anti-windup for steady-state disturbance rejection
- XY/Z gain separation matching physical decoupled dynamics

### Control — Linear MPC (Condensed Formulation)
- Hover-linearized double integrator model: `x = [px, py, pz, vx, vy, vz]`
- Prediction horizon N = 15, control period dt = 0.01 s
- **Precomputed K_first** (3 × 90): runtime reduces to a single matrix-vector multiply → 100 Hz capable
- XY/Z separated Q/R weights reflecting physical priority differences
- Reference Preview integration: `Xref[k] = trajectory at t + k·dt`
- MPC+I variant: integral compensation term added to handle persistent wind disturbance

---

## Performance Results

### 4-Case Robustness Comparison

| Case | Wind | GPS Noise | Controller | Flight Time | RMSE 3D | Completed |
|------|------|-----------|-----------|------------|---------|-----------|
| Case 1 | None | 0.01 m | PID | 18.5 s | 0.685 m | ✅ |
| Case 2 | None | 0.01 m | MPC | 19.2 s | 0.699 m | ✅ |
| Case 3 | 1.0 N (X/Y) | 0.5 m | PID | 18.4 s | 0.826 m | ✅ |
| Case 4 | 1.0 N (X/Y) | 0.5 m | MPC+I | 18.6 s | 1.603 m | ✅ |

**Robustness:** PID degraded by ×1.21 under wind; MPC+I degraded by ×2.29.

> Trajectory: 9-waypoint heptagon with 3D altitude variation (Z: 1.0 ~ 2.0 m),  
> avg_speed = 1.5 m/s, sim/nav evaluated separately (EKF + GPS 0.5 m noise)

### Key Findings — MPC vs PID Analysis

Under ideal (no-wind) conditions, Linear MPC and Cascaded PID perform comparably. Under constant wind disturbance, PID outperforms MPC due to its integral term absorbing the persistent bias. The MPC failure root cause was identified as **time-parameterization mismatch**: the guidance publishes time-indexed setpoints at 20 Hz while MPC at 100 Hz aggressively chases each setpoint, consistently overshooting the guidance schedule.

This behavior directly matches the finding in Foehn et al. (IROS 2021, arXiv:2108.13205). The architecturally correct solution is **MPCC** (Model Predictive Contouring Control) with arc-length parameterization.

---

## Troubleshooting Log (Selected)

| Issue | Root Cause | Solution |
|-------|-----------|----------|
| EKF NaN divergence on integration | Angular velocity missing from `/nav/odom` → D-gain damping lost | Populated `twist.angular` with raw gyro data |
| Z-axis altitude ringing (Runge's Phenomenon) | 7th-order polynomial overfitting on short Z segments | Decoupled Z to linear interpolation |
| MPC overshoots guidance schedule | Time-parameterized guidance + standard MPC structural mismatch | Reference Preview (partial fix); MPCC identified as full solution |
| MPC feedforward double-injection | `a_ref` added on top of `Xref` already containing `v_ref` | Removed `a_ref` feedforward from MPC path |
| MPC+I over-correction under wind | ki=0.3 caused integral over-compensation with time-based guidance | Reduced ki=0.15, max_int_pos=4.0 |

---

## Prerequisites

```bash
# ROS2 Humble (Ubuntu 22.04)
# Eigen3
sudo apt install libeigen3-dev

# ROS2 dependencies
sudo apt install ros-humble-tf2-geometry-msgs ros-humble-nav-msgs
```

---

## Build & Run

```bash
# Clone and build
cd ~/uav_gnc_ws
colcon build --symlink-install
source install/setup.bash

# Run (default: MPC mode, wind + noise enabled)
ros2 launch uav_gnc bringup.launch.py

# Switch to PID mode
# Edit src/uav_gnc/config/control.yaml: use_mpc: false
# Then re-launch (symlink-install: no rebuild needed)

# Plot results
python3 plot_result.py
```

### Configuration Files

| File | Key Parameters |
|------|---------------|
| `config/simulation.yaml` | `wind_x`, `wind_y`, `noise_gps`, `noise_acc`, `noise_gyr` |
| `config/guidance.yaml` | `guidance_mode`, `waypoints_x/y/z`, `avg_speed` |
| `config/control.yaml` | `use_mpc`, `mpc_N`, `mpc_q_pos_xy/z`, `mpc_ki_pos_xy` |
| `config/navigation.yaml` | EKF covariance tuning (`R_gps`, `Q_acc_bias`) |

---

## Repository Structure

```
uav_gnc_ws/
├── src/uav_gnc/
│   ├── config/                  # YAML parameter files
│   ├── include/uav_gnc/
│   │   ├── controller.h         # PID + Linear MPC class definitions
│   │   ├── sixdof.h             # 6-DOF dynamics structures
│   │   ├── ekf.h                # 15-State EKF class
│   │   └── trajectory.h         # Trajectory generator classes
│   ├── launch/
│   │   └── bringup.launch.py    # Full system launch
│   └── src/
│       ├── guidance_node.cpp    # Multi-Snap + Reference Preview
│       ├── navigation_node.cpp  # EKF sensor fusion
│       ├── control_node.cpp     # PID / MPC switching
│       ├── simulation_node.cpp  # 6-DOF physics + noise
│       ├── control/controller.cpp
│       ├── dynamics/sixdof.cpp
│       ├── ekf.cpp
│       ├── trajectory.cpp
│       └── evaluation/tracking_eval_node.cpp
└── plot_result.py               # Result visualization
```

---

## Tech Stack

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![C++](https://img.shields.io/badge/C++-17-blue)
![Eigen3](https://img.shields.io/badge/Eigen-3.4-green)
![Python](https://img.shields.io/badge/Python-3.10-yellow)

- **Framework:** ROS2 Humble
- **Language:** C++17 (core), Python3 (analysis/visualization)
- **Linear Algebra:** Eigen3
- **Visualization:** RViz2, Matplotlib (mpl_toolkits.mplot3d)
- **Build System:** colcon / CMake

---

## Future Work

- **MPCC** (Model Predictive Contouring Control): arc-length parameterization to resolve time-parameterization mismatch — the architecturally correct MPC solution
- **Geometric Controller SE(3)**: nonlinear dynamics, suitable for aggressive maneuvers
- **A\* path planning + LiDAR/SLAM**: obstacle avoidance in unknown environments
- **MPC + RL**: reinforcement learning for adaptive Q/R matrix tuning

---

## References

1. Foehn et al., "Time-Optimal Planning for Quadrotor Waypoint Flight," *IROS 2021* — [arXiv:2108.13205](https://arxiv.org/abs/2108.13205)
2. MathWorks, "Trajectory Optimization and Control of Flying Robot Using Nonlinear MPC" — MATLAB Documentation
3. Hassani et al., "Performance Evaluation of Control Strategies for Autonomous Quadrotors," *Complexity* (2024)
4. Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors," *ICRA 2011*

---

*Developed as a personal GNC portfolio project. All modules implemented independently.*