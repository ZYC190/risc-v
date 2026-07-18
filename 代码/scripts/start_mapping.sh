#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
MAPPER="${1:-gmapping}"

case "${MAPPER}" in
    gmapping|cartographer) ;;
    *)
        echo "Usage: $0 [gmapping|cartographer]" >&2
        exit 2
        ;;
esac

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

"${ROBOT_ROOT}/scripts/nav_preflight.sh" mapping "${MAPPER}"

source /opt/ros/humble/setup.bash
source "${ROBOT_ROOT}/install/setup.bash"
set -u
cd "${ROBOT_ROOT}"

# FD 9 remains inherited by ros2 launch; allow stop/start transitions only
# after preflight has completed and this process is ready to exec the mode.
robot_lock_release_transition
exec 8>&-
trap - EXIT

case "${MAPPER}" in
    gmapping)
        echo "Mapping safety chain: cmd_vel_nav -> velocity_smoother -> collision_monitor -> cmd_vel"
        echo "RPLIDAR 90-270 degree crop still requires physical-direction verification."
        exec ros2 launch wheeltec_nav2 wheeltec_mapping.launch.py \
            mapper:=gmapping \
            start_base:=true \
            start_lidar:=true \
            start_safety:=true \
            lidar_type:=rplidar_c1
        ;;
    cartographer)
        echo "Mapping safety chain: cmd_vel_nav -> velocity_smoother -> collision_monitor -> cmd_vel"
        echo "RPLIDAR 90-270 degree crop still requires physical-direction verification."
        exec ros2 launch wheeltec_nav2 wheeltec_mapping.launch.py \
            mapper:=cartographer \
            start_base:=true \
            start_lidar:=true \
            start_safety:=true \
            lidar_type:=rplidar_c1
        ;;
esac
