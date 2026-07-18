#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-all}"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-${SCRIPT_DIR}/install/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
REMOTE_USER="${REMOTE_USER:-yy}"
REMOTE_HOST="${REMOTE_HOST:-192.168.43.24}"
ROBOT_WS="${ROBOT_WS:-/home/yy/inspection_migrated_ws}"
MAP_FILE="${MAP_FILE:-${ROBOT_WS}/maps/inspection_map.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-${SCRIPT_DIR}/install/inspection_robot_bringup/share/inspection_robot_bringup/rviz/sllidar_ros2.rviz}"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS setup 文件不存在: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "工作空间尚未构建，找不到: ${WS_SETUP}" >&2
    echo "请先执行: cd ${SCRIPT_DIR} && source ${ROS_SETUP} && colcon build --symlink-install" >&2
    exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
    RVIZ_CONFIG="${SCRIPT_DIR}/src/inspection_robot_bringup/rviz/sllidar_ros2.rviz"
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
    echo "RViz 配置文件不存在: ${RVIZ_CONFIG}" >&2
    exit 1
fi

source "${ROS_SETUP}"
source "${WS_SETUP}"
# ROS_LOCALHOST_ONLY is deprecated in Jazzy and can silently block the
# cross-host GUI workflow when inherited as 1. Use the current discovery knob.
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION

gui_args=(
    launch inspection_robot_gui gui.launch.py
    "workspace:=${ROBOT_WS}"
    "map:=${MAP_FILE}"
    "ros_setup:=${ROS_SETUP}"
    "remote_user:=${REMOTE_USER}"
    "remote_host:=${REMOTE_HOST}"
)

pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]:-}"; do
        if kill -0 -- "-${pid}" 2>/dev/null; then
            kill -TERM -- "-${pid}" 2>/dev/null || true
        fi
    done
    for _ in {1..30}; do
        any_running=false
        for pid in "${pids[@]:-}"; do
            if kill -0 -- "-${pid}" 2>/dev/null; then
                any_running=true
                break
            fi
        done
        if [[ "${any_running}" == "false" ]]; then
            break
        fi
        sleep 0.1
    done
    for pid in "${pids[@]:-}"; do
        if kill -0 -- "-${pid}" 2>/dev/null; then
            kill -KILL -- "-${pid}" 2>/dev/null || true
        fi
    done
    for pid in "${pids[@]:-}"; do
        wait "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

start_gui() {
    echo "[start] GUI: ros2 ${gui_args[*]}"
    setsid ros2 "${gui_args[@]}" &
    pids+=("$!")
}

start_rviz() {
    echo "[start] RViz: ${RVIZ_CONFIG}"
    setsid rviz2 -d "${RVIZ_CONFIG}" &
    pids+=("$!")
}

case "${MODE}" in
    gui)
        exec ros2 "${gui_args[@]}"
        ;;
    rviz)
        exec rviz2 -d "${RVIZ_CONFIG}"
        ;;
    all)
        start_gui
        start_rviz
        wait -n "${pids[@]}"
        ;;
    *)
        echo "用法: $0 [gui|rviz|all]" >&2
        exit 2
        ;;
esac
