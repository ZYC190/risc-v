#!/usr/bin/env bash
set -u

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
source "${ROBOT_ROOT}/scripts/robot_mode_lock.sh"
robot_lock_prepare || exit 1
robot_lock_acquire_transition 15 \
    || { echo "ERROR: another robot mode transition is in progress." >&2; exit 1; }
trap 'robot_lock_release_transition' EXIT

declare -A protected_pids=()
ancestor_pid="$$"
while [[ "${ancestor_pid}" =~ ^[0-9]+$ ]] && [[ "${ancestor_pid}" -gt 1 ]]; do
    protected_pids["${ancestor_pid}"]=1
    ancestor_pid="$(ps -o ppid= -p "${ancestor_pid}" 2>/dev/null | tr -d ' ')"
done
self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"

signal_matching_processes() {
    local signal="$1"
    local pattern="$2"
    local pid pgid sid
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        # A caller such as `bash -c 'stop; ros2 launch ...'` can itself contain
        # one of the match strings.  Never signal this script or its ancestors.
        [[ -z "${protected_pids[${pid}]+protected}" ]] || continue
        pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
        sid="$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ')"
        [[ -n "$pgid" ]] || continue

        # Group signalling is reserved for a dedicated setsid launch session.
        # Otherwise terminate only the exact managed PID; later patterns cover
        # its known children without risking an unrelated process in its PGID.
        if [[ -n "${sid}" && "${sid}" == "${pgid}" && "${pgid}" != "${self_pgid}" ]]; then
            kill "-${signal}" -- "-${pgid}" 2>/dev/null || true
        else
            kill "-${signal}" -- "${pid}" 2>/dev/null || true
        fi
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

patterns=(
    # Mapping launch starts its own chassis and lidar. Include it here so a
    # Ctrl-C/SSH disconnect cannot leave a second base or radar stack alive
    # before the competition system is started again.
    'ros2 launch slam_gmapping slam_gmapping.launch.py'
    'ros2 launch wheeltec_cartographer cartographer.launch.py'
    'ros2 launch wheeltec_nav2 wheeltec_mapping.launch.py'
    'ros2 launch wheeltec_nav2 wheeltec_nav2_for_slam.launch.py'
    '/scripts/verify_lidar_runtime.sh'
    '/slam_gmapping/.*/slam_gmapping'
    '/cartographer_ros/.*/cartographer_node'
    '/cartographer_ros/.*/occupancy_grid_node'
    '/wheeltec_robot_keyboard/.*/wheeltec_keyboard'
    'ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py'
    'component_container_isolated.*__node:=nav2_container'
    'component_container.*__node:=nav2_container'
    '/nav2_controller/.*/controller_server'
    '/nav2_planner/.*/planner_server'
    '/nav2_smoother/.*/smoother_server'
    '/nav2_behaviors/.*/behavior_server'
    '/nav2_bt_navigator/.*/bt_navigator'
    '/nav2_waypoint_follower/.*/waypoint_follower'
    '/nav2_waypoint_cycle/.*/nav2_waypoint_cycle'
    '/nav2_map_server/.*/map_server'
    '/nav2_amcl/.*/amcl'
    '/nav2_lifecycle_manager/.*/lifecycle_manager'
    '/nav2_collision_monitor/collision_monitor'
    '/nav2_velocity_smoother/velocity_smoother'
    '/rplidar_ros/.*/rplidar_node'
    '/yolov8_ros2/.*/yolov8_node'
    '/wheeltec_arm_control/.*/arm_control'
    '/wheeltec_ui_dashboard/.*/ui_dashboard'
    '/jobot_mic/.*/myagv_mic_node'
    '/jarvis_voice/.*/jarvis_node'
    '/jarvis_voice/.*/air_sensor_node'
    # The following child signatures let us recover even when the parent
    # ros2 launch process already exited and all remaining nodes have PPID 1.
    '/turn_on_wheeltec_robot/.*/wheeltec_robot_node'
    '/robot_state_publisher/.*/robot_state_publisher'
    '/robot_localization/.*/ekf_node'
    '/tf2_ros/.*/static_transform_publisher'
    '/home_patrol/.*/navigation_manager'
    '/home_patrol/.*/home_navigation_manager'
    '/home_patrol/.*/patrol_node'
    '/home_patrol/.*/map_http_server'
    '/robot_mqtt_bridge/.*/unified_bridge'
    # competition_system.launch.py also starts this Python node.  When the
    # launch parent is interrupted it can survive with PPID 1 while retaining
    # Fast DDS shared-memory files.
    '/joint_state_publisher/joint_state_publisher'
    'ros2 launch home_patrol competition_system.launch.py'
)

# Stop independently-created mode groups and the main launch group. Every
# process is started in a dedicated session, so killing its PGID also collects
# orphaned siblings from an interrupted ros2 launch.
for pattern in "${patterns[@]}"; do
    signal_matching_processes TERM "$pattern"
done

sleep 4

# Escalate only the same narrowly-matched competition processes that ignored
# the graceful shutdown window.
for pattern in "${patterns[@]}"; do
    signal_matching_processes KILL "$pattern"
done

sleep 1

# Do not report success or erase recovery PID files unless every managed
# process signature is gone.  This makes a failed cleanup visible to callers.
remaining_pids=()
for pattern in "${patterns[@]}"; do
    while read -r pid; do
        [[ -n "${pid}" ]] || continue
        [[ -z "${protected_pids[${pid}]+protected}" ]] || continue
        remaining_pids+=("${pid}")
    done < <(pgrep -f "${pattern}" 2>/dev/null || true)
done

if [[ ${#remaining_pids[@]} -gt 0 ]]; then
    mapfile -t remaining_pids < <(printf '%s\n' "${remaining_pids[@]}" | sort -un)
    echo 'ERROR: managed robot processes remain after TERM/KILL:' >&2
    for pid in "${remaining_pids[@]}"; do
        ps -o pid=,ppid=,pgid=,args= -p "${pid}" >&2 || true
    done
    echo 'Recovery PID files were preserved.' >&2
    exit 1
fi

if ! robot_lock_wait_mode 5; then
    echo 'ERROR: robot mode lock is still held after process cleanup.' >&2
    echo 'Recovery PID files were preserved.' >&2
    exit 1
fi
robot_lock_release_mode

rm -f /tmp/home_navigation_nav2.pid /tmp/competition_system.pid

# Fast DDS leaves shared-memory files behind after an unclean launch exit.
# Remove only this user's files, and only when no process still has any of
# them open; otherwise a later start can stall while creating ROS 2 nodes.
if command -v lsof >/dev/null 2>&1 \
    && ! lsof /dev/shm/fastrtps_* >/dev/null 2>&1; then
    find /dev/shm -maxdepth 1 -type f -user "$(id -u)" \
        -name 'fastrtps_*' -delete 2>/dev/null || true
fi

echo 'Competition system stopped.'
