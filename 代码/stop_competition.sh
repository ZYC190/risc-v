#!/usr/bin/env bash
set -u

stop_matching_groups() {
    local pattern="$1"
    local pid pgid
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
        [[ -n "$pgid" ]] || continue
        kill -TERM -- "-$pgid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

# Stop independently-created navigation/arm/vision groups first, then the
# main competition launch group. This also handles orphaned PPID=1 processes.
stop_matching_groups 'ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py'
stop_matching_groups '/yolov8_ros2/.*/yolov8_node'
stop_matching_groups '/wheeltec_arm_control/.*/arm_control'
stop_matching_groups '/wheeltec_ui_dashboard/.*/ui_dashboard'
stop_matching_groups '/jobot_mic/.*/myagv_mic_node'
stop_matching_groups 'ros2 launch home_patrol competition_system.launch.py'

sleep 4

# Escalate only the same narrowly-matched competition processes that ignored
# the graceful shutdown window.
for pattern in \
    'ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py' \
    '/yolov8_ros2/.*/yolov8_node' \
    '/wheeltec_arm_control/.*/arm_control' \
    '/wheeltec_ui_dashboard/.*/ui_dashboard' \
    '/jobot_mic/.*/myagv_mic_node' \
    'ros2 launch home_patrol competition_system.launch.py'; do
    while read -r pid; do
        [[ -n "$pid" ]] || continue
        pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
        [[ -n "$pgid" ]] || continue
        kill -KILL -- "-$pgid" 2>/dev/null || true
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
done

rm -f /tmp/home_navigation_nav2.pid
echo 'Competition system stopped.'
