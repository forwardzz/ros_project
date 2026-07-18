#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_USER="${REMOTE_USER:-yy}"
REMOTE_HOST="${REMOTE_HOST:-192.168.43.24}"
ROBOT_WS="${ROBOT_WS:-/home/yy/inspection_migrated_ws}"
TARGET="${REMOTE_USER}@${REMOTE_HOST}"

packages=(
    YbImuLib
    robot_monitor_interfaces
    robot_mission_utils
    imu_ros2_device
    inspection_robot_dwa_controller
    rf2o_laser_odometry
    sllidar_ros2
    inspection_robot_hardware
    inspection_robot_mission
    inspection_robot_safety
    inspection_robot_bringup
)

echo "[deploy] target: ${TARGET}:${ROBOT_WS}"
ssh "${TARGET}" "mkdir -p '${ROBOT_WS}/src' '${ROBOT_WS}/maps'"

rsync -az --delete \
    --exclude '/.git/' \
    --exclude '/build/' \
    --exclude '/install/' \
    --exclude '/log/' \
    --exclude '/maps/***' \
    --exclude '.pytest_cache/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    "${SCRIPT_DIR}/" "${TARGET}:${ROBOT_WS}/"

if [[ "${DEPLOY_ONLY:-0}" == "1" ]]; then
    echo "[deploy] source sync complete; remote maps were preserved"
    exit 0
fi

package_words="${packages[*]}"
ssh "${TARGET}" "ROBOT_WS='${ROBOT_WS}' PACKAGE_WORDS='${package_words}' bash -s" <<'REMOTE_BUILD'
set -euo pipefail
cd "${ROBOT_WS}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1

for package in ${PACKAGE_WORDS}; do
    echo "[build] ${package}"
    if ! colcon build \
        --symlink-install \
        --packages-select "${package}" \
        --allow-overriding "${package}" \
        --executor sequential \
        --parallel-workers 1; then
        echo "[error] ${package} failed; stop here and only clear build/${package} before retrying" >&2
        exit 1
    fi
    set +u
    source install/setup.bash
    set -u
done
REMOTE_BUILD

echo "[deploy] sync and sequential robot build complete"
