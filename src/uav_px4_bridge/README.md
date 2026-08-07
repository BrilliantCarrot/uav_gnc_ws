# UAV PX4 Bridge

This package is the PX4/Gazebo SITL integration path for the UAV GNC project.
It keeps the custom `uav_dynamics` simulator available for algorithm study, but
lets the production-style runtime use PX4 and Gazebo for flight dynamics,
low-level control, actuator handling, and safety logic.

## Runtime Responsibility

Custom simulation mode:

```text
uav_control -> uav_dynamics/simulation_node -> /sim/odom
navigation_node -> /nav/odom
guidance_node -> /guidance/setpoint
```

PX4 SITL mode:

```text
PX4 + Gazebo -> /fmu/out/vehicle_odometry
px4_odom_converter -> /nav/odom
guidance_node -> /guidance/setpoint
px4_bridge_node -> /fmu/in/trajectory_setpoint
PX4 internal controllers -> Gazebo vehicle motion
```

In PX4 SITL mode, `uav_control` and `uav_dynamics/simulation_node` are not in
the main flight loop. PX4 handles position/velocity/attitude/rate control and
motor output. The UAV GNC project provides higher-level planning and guidance.

## Typical Startup

Terminal 1 starts PX4 SITL and Gazebo, for example from the PX4 container:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Terminal 2 starts the XRCE-DDS agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3 starts this ROS2 bridge:

```bash
source ~/px4_msgs_ws/install/setup.bash
source ~/uav_gnc_ws/install/setup.bash
ros2 launch uav_px4_bridge px4_bringup.launch.py
```

The default launch keeps arming manual:

```text
arm_on_start:=false
```

That is safer while testing. After the bridge is publishing offboard heartbeat
and trajectory setpoints, arm and switch mode from the PX4 shell as needed.

## Current Control Level

The default bridge uses PX4 position-velocity offboard control:

```text
/guidance/setpoint  nav_msgs/Odometry, ENU
        -> px4_bridge_node
/fmu/in/trajectory_setpoint  px4_msgs/TrajectorySetpoint, NED
```

Supported bridge modes:

```text
offboard_mode:=position
offboard_mode:=velocity
offboard_mode:=position_velocity
```

`position_velocity` is the recommended mission mode for time-parameterized
multi-snap references. In testing, position-only offboard reached the mission
but showed larger reference lag because PX4 only received the moving target
position. Passing the velocity feedforward reduced the observed maximum
reference tracking error from about 2.17 m to about 0.66 m.

Use `offboard_mode:=position` as a conservative hover and basic connectivity
check. Use `offboard_mode:=position_velocity` for normal waypoint trajectory
tracking.

## LIO/VIO Direction

The final GPS-denied architecture should feed LIO/VIO output into PX4 as an
external odometry source:

```text
Gazebo LiDAR/IMU or real sensors
        -> FAST-LIO / VIO backend
        -> PX4 external vision/local odometry input
        -> PX4 EKF2
        -> /fmu/out/vehicle_odometry
        -> /nav/odom
```

Before using LIO/VIO as a primary estimator, run it in shadow mode and compare
its odometry against PX4/Gazebo truth or PX4 vehicle odometry. Check delay,
jump, timeout, and RMSE before enabling EKF2 fusion.
