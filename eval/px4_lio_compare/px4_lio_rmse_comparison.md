# PX4 EKF2 GPS/LIO Fusion RMSE Comparison

## Evaluation Cases

| Case | Estimator configuration | Meaning |
|---|---|---|
| `gps_only` | GPS enabled, external vision disabled | PX4 EKF2 baseline |
| `gps_lio` | GPS enabled, LIO external vision enabled | GPS + FAST-LIO2 fusion |
| `gps_denied_lio_baro` | GPS disabled, LIO horizontal position + barometer height | GPS-denied fallback |

## Full Mission Tracking Error

This table includes the whole flight, including takeoff and initial altitude transient.

| Case | Samples | Duration [s] | RMSE XY [m] | RMSE Z [m] | RMSE 3D [m] | Max 3D [m] | Final 3D [m] |
|---|---:|---:|---:|---:|---:|---:|---:|
| GPS only | 828 | 41.35 | 0.253 | 0.809 | 0.848 | 1.983 | 0.234 |
| GPS + LIO | 924 | 46.15 | 0.489 | 0.891 | 1.016 | 2.012 | 0.425 |
| GPS-denied LIO + baro | 978 | 48.85 | 0.385 | 0.960 | 1.034 | 2.022 | 0.061 |

## Stabilized Tracking Error After 10 Seconds

This table excludes the takeoff transient and is more useful for comparing steady waypoint tracking behavior.

| Case | Samples | RMSE XY [m] | RMSE Z [m] | RMSE 3D [m] | Max 3D [m] | Final 3D [m] |
|---|---:|---:|---:|---:|---:|---:|
| GPS only | 627 | 0.290 | 0.032 | 0.292 | 0.609 | 0.234 |
| GPS + LIO | 723 | 0.552 | 0.121 | 0.565 | 1.133 | 0.425 |
| GPS-denied LIO + baro | 777 | 0.432 | 0.338 | 0.549 | 1.976 | 0.061 |

## Interpretation

The baseline GPS-only case produced the lowest steady-state tracking RMSE in this run. Adding LIO together with GPS did not improve tracking RMSE in the current tuning; it increased XY error, which suggests the external vision input is usable but still needs covariance, delay, frame, and fusion tuning before it can outperform the GPS baseline.

The GPS-denied case is the most important result from a portfolio perspective. Full LIO-only fusion was unstable earlier, but using LIO horizontal position with barometer height allowed the UAV to complete the mission without GPS. Its steady 3D RMSE was higher than GPS-only, but the final goal error was the smallest in this run.

The correct engineering claim is:

> FAST-LIO2 odometry was connected to PX4 EKF2 as external vision. In GPS-denied mode, blindly fusing full LIO position/velocity was unstable, so the fusion configuration was changed to use LIO horizontal position with barometric height. This produced stable GPS-denied waypoint flight and enabled quantitative comparison against GPS-only and GPS+LIO cases.

