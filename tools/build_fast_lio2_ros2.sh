#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_LIO_DIR="${WS_DIR}/external/FAST_LIO_ROS2"
LIVOX_DIR="${WS_DIR}/external/livox_ros_driver2_stub"
BUILD_DIR="${WS_DIR}/external/fast_lio2_build"
INSTALL_DIR="${WS_DIR}/external/fast_lio2_install"
LOG_DIR="${WS_DIR}/external/fast_lio2_log"

if [[ ! -d "${FAST_LIO_DIR}" ]]; then
  echo "Missing ${FAST_LIO_DIR}. Clone Ericsii/FAST_LIO_ROS2 first." >&2
  exit 1
fi

if [[ ! -d "${LIVOX_DIR}" ]]; then
  echo "Missing ${LIVOX_DIR}." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

python3 - <<'PY' "${FAST_LIO_DIR}"
from pathlib import Path
import sys

root = Path(sys.argv[1])
cmake = root / "CMakeLists.txt"
package = root / "package.xml"
laser = root / "src" / "laserMapping.cpp"

text = cmake.read_text()
text = text.replace("find_package(pcl_ros REQUIRED)\n", "")
text = text.replace("  pcl_ros\n", "")
if "find_package(tf2 REQUIRED)" not in text:
    text = text.replace(
        "find_package(visualization_msgs REQUIRED)\n",
        "find_package(visualization_msgs REQUIRED)\nfind_package(tf2 REQUIRED)\nfind_package(tf2_ros REQUIRED)\n",
    )
if "  tf2\n" not in text:
    text = text.replace(
        "  visualization_msgs\n",
        "  visualization_msgs\n  tf2\n  tf2_ros\n",
    )
cmake.write_text(text)

text = package.read_text()
text = text.replace("  <depend>pcl_ros</depend>\n", "")
if "tf2_ros" not in text:
    text = text.replace("  <depend>tf2</depend>\n", "  <depend>tf2</depend>\n  <depend>tf2_ros</depend>\n")
package.write_text(text)

text = laser.read_text()
text = text.replace("#include <tf2_ros/transform_broadcaster.h>", "#include <tf2_ros/transform_broadcaster.hpp>")
laser.write_text(text)
PY

colcon --log-base "${LOG_DIR}" build \
  --base-paths "${LIVOX_DIR}" "${FAST_LIO_DIR}" \
  --build-base "${BUILD_DIR}" \
  --install-base "${INSTALL_DIR}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

echo
echo "FAST-LIO2 ROS2 build complete."
echo "Source it with:"
echo "  source ${INSTALL_DIR}/setup.bash"
