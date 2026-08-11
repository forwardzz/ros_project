#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${SCRIPT_DIR}/ros_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
REMOTE_USER="${REMOTE_USER:-yy}"
REMOTE_HOST="${REMOTE_HOST:-192.168.43.31}"
REMOTE_WORKSPACE="${REMOTE_WORKSPACE:-/home/yy/ros2_ws}"
REMOTE_MAP="${REMOTE_MAP:-/home/yy/ros2_ws/map_name.yaml}"
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/ros_project_control_${UID}.lock"
DDS_CONFIG="${HOME}/cyclonedds_unicast.xml"
DETECT_SCRIPT="${SCRIPT_DIR}/scripts/robot_ip_detect.py"
LAST_IP_FILE="${HOME}/.ros_device_ip"

# WSLg exposes both Wayland and X11.  RViz/Qt5 is more reliable through
# XWayland, and Qt requires its runtime directory to be private to the user.
if [[ -d "/mnt/wslg" && -n "${DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    if [[ "${XDG_RUNTIME_DIR:-}" == "/mnt/wslg/runtime-dir" ]]; then
        if [[ -O "${XDG_RUNTIME_DIR}" ]]; then
            chmod 700 "${XDG_RUNTIME_DIR}"
        else
            echo "[GUI] 警告：无法修正 ${XDG_RUNTIME_DIR} 的所有者/权限" >&2
        fi
    fi
fi

exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
    echo "Tracked robot control console is already running."
    exit 1
fi

# ---------------------------------------------------------------------------
# Build choice: interactive at startup; SKIP_BUILD=1 skips (non-interactive).
# ---------------------------------------------------------------------------
BUILD_CHOICE="build"
if [[ "${SKIP_BUILD:-0}" == "1" ]]; then
    BUILD_CHOICE="skip"
elif [[ -t 0 ]]; then
    read -r -p "[BUILD] 编译工作区后启动？[Y=编译(默认) / n=跳过编译] " BUILD_ANSWER
    case "${BUILD_ANSWER}" in
        ""|y|Y|yes|YES) BUILD_CHOICE="build" ;;
        *) BUILD_CHOICE="skip" ;;
    esac
fi
if [[ "${BUILD_CHOICE}" == "skip" && ! -f "${WORKSPACE}/install/setup.bash" ]]; then
    echo "[BUILD] install/setup.bash 不存在，强制执行编译" >&2
    BUILD_CHOICE="build"
fi
if [[ "${BUILD_CHOICE}" == "build" ]]; then
    echo "[BUILD] 将编译主机所需包及本地依赖"
else
    echo "[BUILD] 跳过编译，直接启动（本次不重新构建）"
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -d "${WORKSPACE}/src" ]]; then
    echo "ROS workspace not found: ${WORKSPACE}" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Resolve the robot IP so the UI always starts with the correct address.
# ---------------------------------------------------------------------------
if [[ "${REMOTE_HOST}" == "__AUTO__" ]]; then
    RESOLVED_IP=""
    # fast path: try the last-known address first, then fall back to a scan
    if [[ -f "${LAST_IP_FILE}" ]]; then
        LAST_IP="$(cat "${LAST_IP_FILE}" | tr -d '[:space:]')"
        if [[ -n "${LAST_IP}" ]] && \
            python3 "${DETECT_SCRIPT}" validate --ip "${LAST_IP}" >/dev/null 2>&1 && \
            python3 "${DETECT_SCRIPT}" fast-check --ip "${LAST_IP}" --user "${REMOTE_USER}" >/dev/null 2>&1; then
            RESOLVED_IP="${LAST_IP}"
            echo "[NET] 上次地址仍在线：${LAST_IP}"
        else
            echo "[NET] 上次地址 ${LAST_IP:-（无记录）} 不可达，开始扫描 ..."
        fi
    fi
    if [[ -z "${RESOLVED_IP}" ]]; then
        echo "[NET] 正在自动检测设备 IP ..."
        DETECT_OUT="$(python3 "${DETECT_SCRIPT}" detect --user "${REMOTE_USER}" 2>/dev/null || true)"
    RESOLVED_IP="$(printf '%s' "${DETECT_OUT}" | python3 -c 'import sys,json
try:
    print(json.load(sys.stdin).get("robot") or "")
except Exception:
    print("")' 2>/dev/null || true)"
    fi
    if [[ -z "${RESOLVED_IP}" ]]; then
        echo "[NET] 未自动发现设备。当前候选："
        printf '%s' "${DETECT_OUT}" | python3 -c 'import sys,json
try:
    for c in json.load(sys.stdin).get("candidates", []):
        print("  - %s (%s)" % (c["ip"], c["hostname"]))
except Exception:
    pass' 2>/dev/null || true
        echo "  请确认：手机热点已开启，电脑与树莓派都已连接同一热点。"
        read -r -p "[NET] 手动输入设备 IP（直接回车则跳过，稍后在界面中 Scan LAN 选择）: " USER_IP
        USER_IP="$(printf '%s' "${USER_IP}" | tr -d '[:space:]')"
        if [[ -n "${USER_IP}" ]]; then
            if python3 "${DETECT_SCRIPT}" validate --ip "${USER_IP}" >/dev/null 2>&1; then
                RESOLVED_IP="${USER_IP}"
            else
                echo "[NET] 无效 IP：${USER_IP}" >&2
                exit 1
            fi
        fi
    fi
    if [[ -n "${RESOLVED_IP}" ]]; then
        REMOTE_HOST="${RESOLVED_IP}"
        printf '%s' "${RESOLVED_IP}" > "${LAST_IP_FILE}"
        echo "[NET] 使用设备 IP：${REMOTE_HOST}（已记住）"
    else
        echo "[NET] 未指定设备 IP；界面启动后请使用 Scan LAN 选择。" >&2
        REMOTE_HOST="192.168.43.31"
    fi
fi

# ---------------------------------------------------------------------------
# (Re)generate the CycloneDDS configs for the current network and sync the
# robot-side file, so switching networks cannot crash rviz2/UI anymore.
# ---------------------------------------------------------------------------
generate_cyclone_xml() {
    local local_ip="$1"
    local peer_ip="$2"
    cat <<XML
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces>
        <NetworkInterface address="${local_ip}"/>
      </Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${local_ip}"/>
        <Peer address="${peer_ip}"/>
      </Peers>
      <Ports>
        <Base>7460</Base>
        <DomainGain>250</DomainGain>
        <ParticipantGain>2</ParticipantGain>
        <UnicastMetaOffset>10</UnicastMetaOffset>
        <UnicastDataOffset>11</UnicastDataOffset>
        <MulticastMetaOffset>0</MulticastMetaOffset>
        <MulticastDataOffset>1</MulticastDataOffset>
      </Ports>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
}

WSL_IP="$(python3 "${DETECT_SCRIPT}" wsl-ip --for-host "${REMOTE_HOST}" 2>/dev/null || true)"
if [[ -n "${WSL_IP}" && -n "${REMOTE_HOST}" ]]; then
    generate_cyclone_xml "${WSL_IP}" "${REMOTE_HOST}" > "${DDS_CONFIG}"
    echo "[NET] 本机 Cyclone 配置已生成（${WSL_IP} -> ${REMOTE_HOST}）"
    if ssh -o BatchMode=yes -o ConnectTimeout=3 "${REMOTE_USER}@${REMOTE_HOST}" \
        "cat > /home/yy/cyclonedds_unicast.xml" \
        <<< "$(generate_cyclone_xml "${REMOTE_HOST}" "${WSL_IP}")" 2>/dev/null; then
        echo "[NET] 树莓派 Cyclone 配置已同步"
    else
        echo "[NET] 警告：树莓派 Cyclone 配置同步失败（免密 SSH 不可用？）" >&2
    fi
else
    echo "[NET] 警告：无法确定本机接口 IP，Cyclone 配置未更新" >&2
fi
if [[ ! -f "${DDS_CONFIG}" ]]; then
    echo "CycloneDDS config not found: ${DDS_CONFIG}" >&2
    exit 1
fi

set +u
source "${ROS_SETUP}"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${DDS_CONFIG}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

cd "${WORKSPACE}"
if [[ "${BUILD_CHOICE}" == "build" ]]; then
    colcon build --packages-up-to \
        mapping_bringup \
        robot_control_ui
fi

set +u
source "${WORKSPACE}/install/setup.bash"
set -u

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
cleanup() {
    local status=$?
    trap - EXIT INT TERM HUP
    terminate_pid "${UI_PID}"
    wait "${UI_PID}" 2>/dev/null || true
    exit "${status}"
}
trap cleanup EXIT INT TERM HUP

cleanup_stale_local

echo "[UI] Qt interface"
setsid ros2 launch robot_control_ui ui.launch.py \
    remote_user:="${REMOTE_USER}" \
    remote_host:="${REMOTE_HOST}" \
    workspace_path:="${REMOTE_WORKSPACE}" \
    map_path:="${REMOTE_MAP}" 9>&- &
UI_PID=$!

set +e
wait "${UI_PID}"
STATUS=$?
set -e
exit "${STATUS}"
