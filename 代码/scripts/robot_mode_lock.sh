#!/usr/bin/env bash

# Source this file from robot mode start/stop scripts.  FD 8 serializes mode
# transitions; FD 9 is inherited by the long-lived ros2 launch process and
# prevents a second mode from taking ownership of the chassis/lidar.

robot_lock_prepare() {
    command -v flock >/dev/null 2>&1 \
        || { echo "ERROR: flock is required for robot mode ownership." >&2; return 1; }

    local robot_root="${ROBOT_ROOT:-/home/zyc/robot2}"
    local runtime_base="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
    if [[ ! -d "${runtime_base}" || ! -O "${runtime_base}" ]]; then
        runtime_base="${robot_root}/.runtime"
    fi

    ROBOT_LOCK_DIR="${runtime_base}/wheeltec-robot2"
    local previous_umask
    previous_umask="$(umask)"
    umask 077
    mkdir -p "${ROBOT_LOCK_DIR}" || { umask "${previous_umask}"; return 1; }
    chmod 0700 "${ROBOT_LOCK_DIR}" || { umask "${previous_umask}"; return 1; }
    umask "${previous_umask}"

    [[ -d "${ROBOT_LOCK_DIR}" && -O "${ROBOT_LOCK_DIR}" && ! -L "${ROBOT_LOCK_DIR}" ]] \
        || { echo "ERROR: unsafe robot lock directory: ${ROBOT_LOCK_DIR}" >&2; return 1; }

    ROBOT_TRANSITION_LOCK="${ROBOT_LOCK_DIR}/transition.lock"
    ROBOT_MODE_LOCK="${ROBOT_LOCK_DIR}/mode.lock"
    [[ ! -L "${ROBOT_TRANSITION_LOCK}" && ! -L "${ROBOT_MODE_LOCK}" ]] \
        || { echo "ERROR: robot lock path must not be a symlink." >&2; return 1; }

    exec 8>"${ROBOT_TRANSITION_LOCK}"
    exec 9>"${ROBOT_MODE_LOCK}"
    chmod 0600 "${ROBOT_TRANSITION_LOCK}" "${ROBOT_MODE_LOCK}"
}

robot_lock_acquire_transition() {
    flock -w "${1:-15}" 8
}

robot_lock_release_transition() {
    flock -u 8 2>/dev/null || true
}

robot_lock_acquire_mode() {
    flock -n 9
}

robot_lock_wait_mode() {
    flock -w "${1:-5}" 9
}

robot_lock_release_mode() {
    flock -u 9 2>/dev/null || true
}
