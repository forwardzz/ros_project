#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}/ros_ws"
RVIZ_CONFIG="${WORKSPACE}/src/sllidar_ros2/rviz/sllidar_ros2.rviz"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
REMOTE_USER="${REMOTE_USER:-yy}"
REMOTE_HOST="${REMOTE_HOST:-192.168.43.24}"
REMOTE_WORKSPACE="${REMOTE_WORKSPACE:-/home/yy/ros2_ws}"
REMOTE_MAP="${REMOTE_MAP:-/home/yy/ros2_ws/map_name.yaml}"
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/ros_project_control_${UID}.lock"

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "Tracked robot control console is already running."
    exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -d "${WORKSPACE}/src" ]]; then
    echo "ROS workspace not found: ${WORKSPACE}" >&2
    exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
    echo "RViz config not found: ${RVIZ_CONFIG}" >&2
    exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

cd "${WORKSPACE}"
colcon build --packages-select \
    robot_monitor_interfaces \
    robot_mission_utils \
    mapping_bringup \
    robot_control_ui

set +u
source "${WORKSPACE}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

terminate_pid() {
    local pid="${1:-}"
    [[ -n "${pid}" ]] || return 0
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || return 0
    for _ in {1..20}; do
        kill -0 -- "-${pid}" 2>/dev/null || return 0
        sleep 0.1
    done
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

cleanup_stale_local() {
    local patterns=(
        "${WORKSPACE}/install/robot_control_ui/lib/robot_control_ui/robot_control_ui"
        "ros2 launch robot_control_ui ui.launch.py"
        "rviz2 -d ${RVIZ_CONFIG}"
    )
    local pattern
    local pid
    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ -n "${pid}" ]] && kill -TERM "${pid}" 2>/dev/null || true
        done < <(pgrep -f -- "${pattern}" || true)
    done
    sleep 1
    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ -n "${pid}" ]] && kill -KILL "${pid}" 2>/dev/null || true
        done < <(pgrep -f -- "${pattern}" || true)
    done
}

UI_PID=""
RVIZ_PID=""
cleanup() {
    local status=$?
    trap - EXIT INT TERM HUP
    terminate_pid "${UI_PID}"
    terminate_pid "${RVIZ_PID}"
    wait "${UI_PID}" 2>/dev/null || true
    wait "${RVIZ_PID}" 2>/dev/null || true
    exit "${status}"
}
trap cleanup EXIT INT TERM HUP

cleanup_stale_local

setsid ros2 launch robot_control_ui ui.launch.py \
    remote_user:="${REMOTE_USER}" \
    remote_host:="${REMOTE_HOST}" \
    workspace_path:="${REMOTE_WORKSPACE}" \
    map_path:="${REMOTE_MAP}" 9>&- &
UI_PID=$!

setsid rviz2 -d "${RVIZ_CONFIG}" 9>&- &
RVIZ_PID=$!

set +e
wait -n "${UI_PID}" "${RVIZ_PID}"
STATUS=$?
set -e
exit "${STATUS}"
