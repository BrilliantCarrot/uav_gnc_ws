# GPS-Denied Outdoor LIO/VIO Environment

This environment is for future GPS-denied navigation validation with both LIO
and VIO backends. It is intentionally separate from the current ROS-only
`simulation_node` flow so the existing GNC tests are not changed.

## Why This World Exists

FAST-LIO2 needs enough stable 3D geometry for scan matching. A nearly empty
outdoor field produces too few useful points, so `/lio/odom` can jump or time
out. VIO also needs visual features such as corners, texture, contrast, and
non-repeating patterns. The world therefore contains:

- ground plane and asymmetric ground markings,
- building walls and corners,
- poles, boxes, signs, and tree-like geometry,
- colored panels and non-repeating visual features.

The goal is not photorealism. The goal is to provide useful geometry for LiDAR
and useful image features for stereo/RGB-D odometry.

## Added Assets

- `worlds/gps_denied_outdoor_lio_vio.world.sdf`
- `worlds/uav_gnc_lio_px4.world.sdf`
- `models/f450_lio_vio_sensor_rig/model.sdf`
- `models/x500_lidar/model.sdf`
- `config/gz_lio_vio_bridge.yaml`
- `config/vio_interface.yaml`
- `launch/gps_denied_lio_vio_world.launch.py`
- `launch/px4_lio_shadow.launch.py`
- `tools/run_px4_x500_lidar_lio_world.sh`

## Intended Topic Contract

LiDAR/LIO:

```text
/lidar/points_raw -> FAST-LIO2 -> /lio/odom
/sim/imu          -> FAST-LIO2
```

Stereo/RGB-D VIO:

```text
/stereo/left/image_raw
/stereo/right/image_raw
/rgbd/image_raw
/rgbd/depth/image_raw
/sim/imu
        -> VIO or RGB-D odometry backend -> /vio/odom
```

Navigation:

```text
/lio/odom or /vio/odom
        -> localization_manager quality gates
        -> /nav/odom
```

`/sim/odom` should remain evaluation-only in GPS-denied validation.

## Usage

After building and sourcing the workspace:

```bash
ros2 launch uav_bringup gps_denied_lio_vio_world.launch.py
```

This launches only the Gazebo sensor validation world. It does not run PX4.

## PX4 X500 + LiDAR Shadow Validation

For PX4 SITL, use an X500 model variant with an added 3D LiDAR and IMU. The
world contains walls, poles, boxes, and ground markings so FAST-LIO2 receives
enough stable geometry.

Terminal 1, PX4/Gazebo:

```bash
cd ~/uav_gnc_ws
./tools/run_px4_x500_lidar_lio_world.sh
```

If PX4 is running inside Docker and cannot see `~/uav_gnc_ws`, copy or mount
these two paths into the container before launching PX4:

```text
src/uav_bringup/models/x500_lidar
src/uav_bringup/worlds/uav_gnc_lio_px4.world.sdf
```

Terminal 2, Micro XRCE-DDS agent:

```bash
docker exec -it px4_sitl /bin/bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3, PX4 offboard bridge:

```bash
cd ~/uav_gnc_ws
source /opt/ros/humble/setup.bash
source ~/px4_msgs_ws/install/setup.bash
source install/setup.bash
ros2 launch uav_px4_bridge px4_bringup.launch.py
```

Terminal 4, FAST-LIO2 shadow pipeline:

```bash
cd ~/uav_gnc_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source external/fast_lio2_install/setup.bash
ros2 launch uav_bringup px4_lio_shadow.launch.py
```

This shadow pipeline does not feed `/lio/odom` back into PX4 or `/nav/odom`.
It only checks whether Gazebo LiDAR/IMU can drive FAST-LIO2 while PX4 flies.

Expected topic checks:

```bash
ros2 topic hz /gazebo/lidar/points_raw
ros2 topic hz /lidar/points_raw
ros2 topic hz /gazebo/imu_raw
ros2 topic hz /lio/imu
ros2 topic hz /lio/odom
```

If bridge startup fails, inspect native Gazebo topics:

```bash
gz topic -l
```

Then update `config/gz_lio_vio_bridge.yaml` to match the exact topic names
published by the installed Gazebo version.

## Next Implementation Step

1. Verify Gazebo publishes dense `/lidar/points_raw`.
2. Feed `/lidar/points_raw` and `/sim/imu` to FAST-LIO2.
3. Verify `/lio/odom` is stable before using it as `/nav/odom`.
4. Add a VIO backend such as OpenVINS, VINS-Fusion, ORB-SLAM3, or RTAB-Map.
5. Extend `localization_manager_node` to select/fuse `/lio/odom` and
   `/vio/odom` with timeout, jump, covariance, and quality checks.
