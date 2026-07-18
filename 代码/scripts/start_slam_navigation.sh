#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
source "${ROBOT_ROOT}/scripts/robot_mode_lock.sh"
robot_lock_prepare
robot_lock_acquire_transition 15 \
    || { echo "ERROR: another robot mode transition is in progress." >&2; exit 1; }
if ! robot_lock_acquire_mode; then
    robot_lock_release_transition
    echo "ERROR: another robot mode owns the chassis/lidar." >&2
    exit 1
fi
trap 'robot_lock_release_mode; robot_lock_release_transition' EXIT

"${ROBOT_ROOT}/scripts/nav_preflight.sh" mapping gmapping
source /opt/ros/humble/setup.bash
source "${ROBOT_ROOT}/install/setup.bash"
set -u
cd "${ROBOT_ROOT}"

robot_lock_release_transition
exec 8>&-
trap - EXIT
exec ros2 launch wheeltec_nav2 wheeltec_nav2_for_slam.launch.py \
    start_base:=true \
    start_lidar:=true \
    start_waypoint_cycle:=false \
    lidar_type:=rplidar_c1
