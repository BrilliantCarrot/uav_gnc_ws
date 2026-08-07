#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
UAV_GNC_WS="${UAV_GNC_WS:-$HOME/uav_gnc_ws}"

PX4_GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
PX4_GZ_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
UAV_MODEL_DIR="$UAV_GNC_WS/src/uav_bringup/models/x500_lidar"
UAV_WORLD_FILE="$UAV_GNC_WS/src/uav_bringup/worlds/uav_gnc_lio_px4.world.sdf"
PX4_LIO_MODEL_DIR="$PX4_GZ_MODELS_DIR/x500_lidar"
PX4_LIO_AIRFRAME_ID="${PX4_LIO_AIRFRAME_ID:-4019}"
PX4_LIO_AIRFRAME_NAME="gz_x500_lidar"

if [[ ! -d "$PX4_DIR" ]]; then
  echo "PX4_DIR not found: $PX4_DIR" >&2
  exit 1
fi

if [[ ! -f "$UAV_WORLD_FILE" ]]; then
  echo "PX4 LIO world not found: $UAV_WORLD_FILE" >&2
  exit 1
fi

if [[ ! -f "$PX4_GZ_MODELS_DIR/x500/model.sdf" ]]; then
  echo "PX4 x500 base model not found: $PX4_GZ_MODELS_DIR/x500/model.sdf" >&2
  exit 1
fi

mkdir -p "$PX4_GZ_MODELS_DIR" "$PX4_GZ_WORLDS_DIR"

rm -rf "$PX4_LIO_MODEL_DIR"
mkdir -p "$PX4_LIO_MODEL_DIR"

sed "0,/<model name=['\"]x500['\"]>/s//<model name='x500_lidar'>/" \
  "$PX4_GZ_MODELS_DIR/x500/model.sdf" > "$PX4_LIO_MODEL_DIR/model.base.sdf"

awk '
  /<\/model>/ && !inserted {
    print "    <link name=\"lidar_link\">"
    print "      <pose relative_to=\"base_link\">0 0 0.14 0 0 0</pose>"
    print "      <inertial><mass>0.18</mass><inertia><ixx>0.0002</ixx><iyy>0.0002</iyy><izz>0.0002</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
    print "      <sensor name=\"lidar_3d\" type=\"gpu_lidar\">"
    print "        <pose>0 0 0 0 0 0</pose>"
    print "        <always_on>1</always_on>"
    print "        <visualize>true</visualize>"
    print "        <update_rate>10</update_rate>"
    print "        <topic>/gazebo/lidar/points</topic>"
    print "        <lidar>"
    print "          <scan>"
    print "            <horizontal><samples>1024</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>"
    print "            <vertical><samples>32</samples><resolution>1</resolution><min_angle>-0.43633</min_angle><max_angle>0.43633</max_angle></vertical>"
    print "          </scan>"
    print "          <range><min>0.5</min><max>60.0</max><resolution>0.02</resolution></range>"
    print "          <noise><type>gaussian</type><mean>0</mean><stddev>0.015</stddev></noise>"
    print "        </lidar>"
    print "      </sensor>"
    print "    </link>"
    print "    <joint name=\"lidar_joint\" type=\"fixed\"><parent>base_link</parent><child>lidar_link</child></joint>"
    print "    <link name=\"lio_imu_link\">"
    print "      <pose relative_to=\"base_link\">0 0 0.02 0 0 0</pose>"
    print "      <inertial><mass>0.02</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
    print "      <sensor name=\"lio_imu\" type=\"imu\">"
    print "        <always_on>1</always_on><update_rate>200</update_rate><topic>/gazebo/imu</topic>"
    print "        <imu>"
    print "          <angular_velocity><x><noise type=\"gaussian\"><mean>0</mean><stddev>0.002</stddev></noise></x><y><noise type=\"gaussian\"><mean>0</mean><stddev>0.002</stddev></noise></y><z><noise type=\"gaussian\"><mean>0</mean><stddev>0.002</stddev></noise></z></angular_velocity>"
    print "          <linear_acceleration><x><noise type=\"gaussian\"><mean>0</mean><stddev>0.03</stddev></noise></x><y><noise type=\"gaussian\"><mean>0</mean><stddev>0.03</stddev></noise></y><z><noise type=\"gaussian\"><mean>0</mean><stddev>0.03</stddev></noise></z></linear_acceleration>"
    print "        </imu>"
    print "      </sensor>"
    print "    </link>"
    print "    <joint name=\"lio_imu_joint\" type=\"fixed\"><parent>base_link</parent><child>lio_imu_link</child></joint>"
    inserted = 1
  }
  { print }
' "$PX4_LIO_MODEL_DIR/model.base.sdf" > "$PX4_LIO_MODEL_DIR/model.sdf"
rm -f "$PX4_LIO_MODEL_DIR/model.base.sdf"

cat > "$PX4_LIO_MODEL_DIR/model.config" <<'MODEL_CONFIG'
<?xml version="1.0"?>
<model>
  <name>x500_lidar</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>UAV GNC Project</name>
  </author>
  <description>PX4 x500 model with a 3D LiDAR and auxiliary IMU for FAST-LIO2 shadow validation.</description>
</model>
MODEL_CONFIG

ln -sfn "$UAV_WORLD_FILE" "$PX4_GZ_WORLDS_DIR/uav_gnc_lio_px4.sdf"
ln -sfn "$UAV_WORLD_FILE" "$PX4_GZ_WORLDS_DIR/uav_gnc_lio_px4.world.sdf"

unset PX4_GZ_MODEL
export PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_0}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-uav_gnc_lio_px4}"
export PX4_GZ_WORLDS="${PX4_GZ_WORLDS:-$PX4_GZ_WORLDS_DIR}"
export GZ_SIM_RESOURCE_PATH="$PX4_GZ_MODELS_DIR:$PX4_GZ_WORLDS_DIR:$UAV_GNC_WS/src/uav_bringup/models:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$PX4_GZ_MODELS_DIR:$PX4_GZ_WORLDS_DIR:$UAV_GNC_WS/src/uav_bringup/models:${IGN_GAZEBO_RESOURCE_PATH:-}"

# This custom LiDAR validation world can trip PX4's magnetic-field strength
# preflight check while the simulated compass is otherwise present and usable.
# Keep the normal estimator running, but do not block SITL arming on this check.
export PX4_PARAM_COM_ARM_MAG_STR="${PX4_PARAM_COM_ARM_MAG_STR:-0}"
export PX4_PARAM_EKF2_MAG_CHECK="${PX4_PARAM_EKF2_MAG_CHECK:-0}"
export PX4_PARAM_NAV_DLL_ACT="${PX4_PARAM_NAV_DLL_ACT:-0}"
export PX4_PARAM_NAV_RCL_ACT="${PX4_PARAM_NAV_RCL_ACT:-0}"
export PX4_PARAM_COM_RCL_EXCEPT="${PX4_PARAM_COM_RCL_EXCEPT:-4}"
export PX4_PARAM_COM_DISARM_PRFLT="${PX4_PARAM_COM_DISARM_PRFLT:-0}"

echo "PX4_GZ_MODEL_NAME=$PX4_GZ_MODEL_NAME"
echo "PX4_GZ_WORLD=$PX4_GZ_WORLD"
echo "PX4 x500_lidar generated from: $PX4_GZ_MODELS_DIR/x500/model.sdf"
echo "PX4 x500_lidar model dir: $PX4_LIO_MODEL_DIR"
echo "PX4 world link: $PX4_GZ_WORLDS_DIR/uav_gnc_lio_px4.sdf -> $UAV_WORLD_FILE"

cd "$PX4_DIR"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [[ ! -x "$PX4_BIN" ]]; then
  make px4_sitl_default
fi

PX4_BUILD_AIRFRAME_DIR="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
mkdir -p "$PX4_BUILD_AIRFRAME_DIR"
cat > "$PX4_BUILD_AIRFRAME_DIR/${PX4_LIO_AIRFRAME_ID}_${PX4_LIO_AIRFRAME_NAME}" <<'AIRFRAME'
#!/bin/sh
#
# @name Gazebo x500 LiDAR
#
# @type Quadrotor
#

. ${R}etc/init.d-posix/airframes/4001_gz_x500

# Custom SITL/LIO validation runs without QGroundControl or RC.
param set-default NAV_DLL_ACT 0
param set-default NAV_RCL_ACT 0
param set-default COM_RCL_EXCEPT 4
param set-default COM_DISARM_PRFLT 0

# The custom outdoor LiDAR world can trip magnetic-field strength checks.
param set-default COM_ARM_MAG_STR 0
param set-default EKF2_MAG_CHECK 0
AIRFRAME

export PX4_SIM_MODEL="${PX4_SIM_MODEL:-$PX4_LIO_AIRFRAME_NAME}"
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-$PX4_LIO_AIRFRAME_ID}"

PX4_ROOTFS="$PX4_DIR/build/px4_sitl_default/rootfs"
rm -f "$PX4_ROOTFS/parameters.bson" "$PX4_ROOTFS/parameters_backup.bson"

cd "$PX4_DIR/build/px4_sitl_default/bin"
exec "$PX4_BIN"
