#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
TEMP_DIR="$(mktemp -d /tmp/rplidar_c1_verify.XXXXXX)"
LOG_FILE="${TEMP_DIR}/runtime.log"
PARAM_FILE="${TEMP_DIR}/params.yaml"
FINAL_LOG="${ROBOT_ROOT}/logs/rplidar_c1_verify_$(date +%Y%m%d_%H%M%S)_$(basename "${TEMP_DIR}").log"
launch_pid=""
launch_pgid=""
transition_locked=false
mode_locked=false

cleanup() {
    if [[ -n "${launch_pgid}" ]]; then
        kill -INT -- "-${launch_pgid}" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 -- "-${launch_pgid}" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 -- "-${launch_pgid}" 2>/dev/null; then
            kill -TERM -- "-${launch_pgid}" 2>/dev/null || true
            sleep 1
        fi
        if kill -0 -- "-${launch_pgid}" 2>/dev/null; then
            kill -KILL -- "-${launch_pgid}" 2>/dev/null || true
        fi
    elif [[ -n "${launch_pid}" ]]; then
        kill -INT "${launch_pid}" 2>/dev/null || true
    fi
    if [[ -n "${launch_pid}" ]]; then
        wait "${launch_pid}" 2>/dev/null || true
    fi
    if [[ -s "${LOG_FILE}" ]] \
        && mkdir -p "$(dirname "${FINAL_LOG}")" \
        && install -m 0644 "${LOG_FILE}" "${FINAL_LOG}"; then
        echo "RPLIDAR verification log: ${FINAL_LOG}" >&2
    fi
    rm -rf -- "${TEMP_DIR}"
    if [[ "${mode_locked}" == true ]]; then
        robot_lock_release_mode
        mode_locked=false
    fi
    if [[ "${transition_locked}" == true ]]; then
        robot_lock_release_transition
        transition_locked=false
    fi
}
trap cleanup EXIT

source "${ROBOT_ROOT}/scripts/robot_mode_lock.sh"
robot_lock_prepare
robot_lock_acquire_transition 15 \
    || { echo "ERROR: another robot mode transition is in progress." >&2; exit 1; }
transition_locked=true
if ! robot_lock_acquire_mode; then
    echo "ERROR: another robot mode owns the chassis/lidar." >&2
    exit 1
fi
mode_locked=true

source /opt/ros/humble/setup.bash
source "${ROBOT_ROOT}/install/setup.bash"
set -u

existing="$(pgrep -af 'rplidar_node|wheeltec_lidar.launch.py' || true)"
[[ -z "${existing}" ]] \
    || { echo "ERROR: a lidar process is already running:\n${existing}" >&2; exit 1; }
if command -v fuser >/dev/null 2>&1; then
    serial_users="$(fuser /dev/wheeltec_lidar 2>/dev/null || true)"
    [[ -z "${serial_users}" ]] \
        || { echo "ERROR: lidar serial port is busy (PID:${serial_users})." >&2; exit 1; }
fi

robot_lock_release_transition
transition_locked=false
exec 8>&-

setsid ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py \
    lidar_type:=rplidar_c1 >"${LOG_FILE}" 2>&1 &
launch_pid=$!
launch_pgid="$(ps -o pgid= -p "${launch_pid}" | tr -d ' ')"

ready=false
for _ in $(seq 1 30); do
    topic_info="$(ros2 topic info /scan 2>/dev/null || true)"
    publisher_count="$(awk '/Publisher count:/ {print $3}' <<<"${topic_info}")"
    if [[ "${publisher_count:-0}" -eq 1 ]]; then
        ready=true
        break
    fi
    sleep 0.5
done
[[ "${ready}" == true ]] \
    || { echo "ERROR: /scan did not acquire exactly one publisher" >&2; tail -n 80 "${LOG_FILE}"; exit 1; }

timeout 10 ros2 param dump /rplidar_node >"${PARAM_FILE}" \
    || { echo "ERROR: unable to read /rplidar_node parameters" >&2; exit 1; }
python3 - "${PARAM_FILE}" <<'PY'
import sys

import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    document = yaml.safe_load(stream)
if not isinstance(document, dict) or "/rplidar_node" not in document:
    raise SystemExit("parameter dump does not contain /rplidar_node")
node_document = document["/rplidar_node"]
if not isinstance(node_document, dict):
    raise SystemExit("invalid /rplidar_node parameter document")
params = node_document.get("ros__parameters")
if not isinstance(params, dict):
    raise SystemExit("parameter dump lacks ros__parameters")
expected = {
    "serial_port": "/dev/wheeltec_lidar",
    "serial_baudrate": 460800,
    "frame_id": "laser_link",
    "scan_mode": "Standard",
    "enable_angle_crop_func": True,
    "angle_crop_min": 90.0,
    "angle_crop_max": 270.0,
}
for key, expected_value in expected.items():
    if key not in params:
        raise SystemExit(f"missing lidar parameter: {key}")
    actual = params[key]
    if actual != expected_value:
        raise SystemExit(f"{key}: {actual!r} != {expected_value!r}")
    print(f"{key}: {actual}")
PY

frequency_output="$(timeout --signal=INT --kill-after=2s 8s \
    env PYTHONUNBUFFERED=1 ros2 topic hz /scan --window 20 2>&1 || true)"
echo "${frequency_output}"
grep -q 'average rate:' <<<"${frequency_output}" \
    || { echo "ERROR: unable to measure /scan frequency" >&2; exit 1; }
frequency="$(awk '/average rate:/ {value=$3} END {print value}' \
    <<<"${frequency_output}")"
python3 - "${frequency}" <<'PY'
import sys

frequency = float(sys.argv[1])
if not 8.0 <= frequency <= 12.0:
    raise SystemExit(f"unexpected /scan frequency: {frequency} Hz")
print(f"validated /scan frequency: {frequency:.2f} Hz")
PY

cleanup
launch_pid=""
launch_pgid=""
trap - EXIT

sleep 1
remaining="$(pgrep -af 'rplidar_node|wheeltec_lidar.launch.py' || true)"
[[ -z "${remaining}" ]] \
    || { echo "ERROR: lidar process remained after verification:\n${remaining}" >&2; exit 1; }

grep -E 'lidar_type|SLLIDAR|health status|current scan mode' "${FINAL_LOG}" \
    | tail -n 40 || true
echo "RPLIDAR C1 runtime verification passed. Log: ${FINAL_LOG}"
