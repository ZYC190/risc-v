#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="/home/zyc/robot2"
LOG_FILE="${ROBOT_ROOT}/competition_system.log"
PID_FILE="/tmp/competition_system.pid"
LAUNCH_PATTERN='ros2 launch home_patrol competition_system.launch.py'
FOREGROUND=false

case "${1:-}" in
    "")
        ;;
    --foreground)
        FOREGROUND=true
        ;;
    *)
        echo "Usage: $0 [--foreground]" >&2
        exit 2
        ;;
esac

source "${ROBOT_ROOT}/scripts/robot_mode_lock.sh"
robot_lock_prepare
robot_lock_acquire_transition 15 \
    || { echo "ERROR: another robot mode transition is in progress." >&2; exit 1; }

if ! robot_lock_acquire_mode; then
    robot_lock_release_transition
    if pgrep -f "${LAUNCH_PATTERN}" >/dev/null 2>&1; then
        echo "Competition system is already running."
        exit 0
    fi
    echo "ERROR: another robot mode owns the chassis/lidar." >&2
    exit 1
fi
trap 'robot_lock_release_transition' EXIT

# A legacy/manual launch may predate the shared lock.  Do not pretend it is
# protected; require one clean stop/restart so every future mode is serialized.
if pgrep -f "${LAUNCH_PATTERN}" >/dev/null 2>&1; then
    echo "ERROR: an unmanaged competition launch is running; stop it first." >&2
    exit 1
fi

enable_mobile_base=false
if [[ -c /dev/wheeltec_controller && -c /dev/wheeltec_lidar ]]; then
    "${ROBOT_ROOT}/scripts/nav_preflight.sh" navigation
    enable_mobile_base=true
    echo "Hardware profile: full competition system (voice + environment + chassis + lidar)"
else
    echo "Hardware profile: voice/environment debug mode (chassis or lidar is not connected)"
    [[ -c /dev/wheeltec_mic ]] \
        || echo "WARNING: /dev/wheeltec_mic is unavailable; sound direction may not work." >&2
    [[ -c /dev/air_sensor ]] \
        || echo "WARNING: /dev/air_sensor is unavailable; environment data will wait for reconnection." >&2
fi
export JARVIS_ENABLE_BASE_ROTATION="${enable_mobile_base}"

cd "${ROBOT_ROOT}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

launch_command=(
    ros2 launch home_patrol competition_system.launch.py
    "enable_mobile_base:=${enable_mobile_base}"
    "auto_enable_interaction:=true"
)

# Keep the USB speaker loud enough for competition demonstrations.  Baidu TTS
# already uses its maximum volume; this mixer level raises the actual speaker
# output while leaving headroom to avoid the distortion of a 100% setting.
if ! amixer -q sset PCM 80% unmute; then
    echo "WARNING: unable to set USB speaker volume; continuing with current level." >&2
fi

if [[ "${FOREGROUND}" == true ]]; then
    robot_lock_release_transition
    trap - EXIT
    exec 8>&-
    exec "${launch_command[@]}"
fi

# ros2 launch must be detached from the SSH output pipe.  The child inherits
# FD 9 and therefore owns the robot mode lock for its full lifetime.
if [[ -f "${LOG_FILE}" ]]; then
    mv -f "${LOG_FILE}" "${LOG_FILE}.previous"
fi

# The child closes transition FD 8 before exec, but deliberately inherits the
# locked mode FD 9.  Thus any wrapper failure closes only the parent's FD9;
# a surviving launch continues to own the lock.
(
    exec 8>&-
    exec nohup setsid "${launch_command[@]}"
) </dev/null >"${LOG_FILE}" 2>&1 &
launch_pid=$!
echo "${launch_pid}" >"${PID_FILE}"

sleep 2
if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Competition system failed to start. Recent log:"
    tail -n 40 "${LOG_FILE}" || true
    rm -f "${PID_FILE}"
    exit 1
fi

# The child never inherited transition FD 8. Mode FD 9 remains locked when
# this wrapper exits and closes only the parent's copy.
robot_lock_release_transition
trap - EXIT

echo "Competition system started in background (PID ${launch_pid})."
echo "Log: ${LOG_FILE}"
