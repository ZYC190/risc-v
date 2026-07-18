#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"

source /opt/ros/humble/setup.bash
if [[ -f "${ROBOT_ROOT}/install/setup.bash" ]]; then
    source "${ROBOT_ROOT}/install/setup.bash"
fi
set -u

cd "${ROBOT_ROOT}"

# CMake can consider a stale zero-byte link product up to date.  This board
# had exactly that state for wheeltec_robot_node, so force a package clean
# before rebuilding whenever the build-side executable is missing or invalid.
base_build_executable="${ROBOT_ROOT}/build/turn_on_wheeltec_robot/wheeltec_robot_node"
base_build_dir="${ROBOT_ROOT}/build/turn_on_wheeltec_robot"
if [[ -e "${base_build_executable}" ]] \
    && { [[ ! -s "${base_build_executable}" ]] || [[ ! -x "${base_build_executable}" ]]; }; then
    echo "Invalid stale base executable detected; cleaning turn_on_wheeltec_robot build artifacts."
    cmake --build "${base_build_dir}" --target clean
fi

# The workspace also contains reference copies under src/agv_mec with the same
# ROS package names.  Limit discovery to the active package directories.
colcon build --symlink-install \
    --allow-overriding turn_on_wheeltec_robot wheeltec_nav2 \
    --base-paths \
        "${ROBOT_ROOT}/src/turn_on_wheeltec_robot" \
        "${ROBOT_ROOT}/src/wheeltec_robot_slam/slam_gmapping" \
        "${ROBOT_ROOT}/src/wheeltec_robot_slam/wheeltec_cartographer" \
        "${ROBOT_ROOT}/src/wheeltec_robot_nav2" \
    --packages-select \
        turn_on_wheeltec_robot \
        slam_gmapping \
        wheeltec_cartographer \
        wheeltec_nav2

set +u
source "${ROBOT_ROOT}/install/setup.bash"
set -u
required_executables=(
    "${ROBOT_ROOT}/install/turn_on_wheeltec_robot/lib/turn_on_wheeltec_robot/wheeltec_robot_node"
    "${ROBOT_ROOT}/install/rplidar_ros/lib/rplidar_ros/rplidar_node"
    "${ROBOT_ROOT}/install/slam_gmapping/lib/slam_gmapping/slam_gmapping"
)
for executable in "${required_executables[@]}"; do
    [[ -s "${executable}" && -x "${executable}" ]] \
        || { echo "ERROR: required ROS executable is invalid: ${executable}" >&2; exit 1; }
    echo "verified executable: ${executable}"
done

echo "Selected navigation packages built successfully."
