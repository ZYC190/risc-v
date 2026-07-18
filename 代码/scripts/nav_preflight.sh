#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
MODE="${1:-static}"
MAPPER="${2:-gmapping}"
CONFIG="${ROBOT_ROOT}/src/turn_on_wheeltec_robot/config/wheeltec_param.yaml"
MAP_YAML="${ROBOT_ROOT}/src/wheeltec_robot_nav2/map/WHEELTEC.yaml"
NAV_PARAMS="${ROBOT_ROOT}/src/wheeltec_robot_nav2/param/wheeltec_params/param_mini_mec.yaml"
MAPPING_SAFETY="${ROBOT_ROOT}/src/wheeltec_robot_nav2/param/wheeltec_params/mapping_safety.yaml"

source /opt/ros/humble/setup.bash
source "${ROBOT_ROOT}/install/setup.bash"
set -u

fail() {
    echo "ERROR: $*" >&2
    exit 1
}

case "${MODE}" in
    static|navigation|mapping) ;;
    *) fail "mode must be static, navigation, or mapping" ;;
esac

[[ -c /dev/wheeltec_controller ]] || fail "controller alias is not a character device"
[[ -r /dev/wheeltec_controller && -w /dev/wheeltec_controller ]] \
    || fail "controller device is not readable/writable"
[[ -c /dev/wheeltec_lidar ]] || fail "lidar alias is not a character device"
[[ -r /dev/wheeltec_lidar && -w /dev/wheeltec_lidar ]] \
    || fail "lidar device is not readable/writable"

lidar_target="$(readlink -f /dev/wheeltec_lidar)"
controller_target="$(readlink -f /dev/wheeltec_controller)"
echo "controller: /dev/wheeltec_controller -> ${controller_target}"
echo "lidar:      /dev/wheeltec_lidar -> ${lidar_target}"

lidar_properties="$(udevadm info --query=property --name=/dev/wheeltec_lidar)"
controller_properties="$(udevadm info --query=property --name=/dev/wheeltec_controller)"
grep -q '^ID_VENDOR_ID=10c4$' <<<"${lidar_properties}" \
    || fail "lidar USB vendor is not 10c4"
grep -q '^ID_MODEL_ID=ea60$' <<<"${lidar_properties}" \
    || fail "lidar USB product is not ea60"
grep -q '^ID_SERIAL_SHORT=ba668b80e173ef119897d08c8fcc3fa0$' \
    <<<"${lidar_properties}" || fail "lidar USB serial does not match this robot"
grep -q '^ID_VENDOR_ID=1a86$' <<<"${controller_properties}" \
    || fail "controller USB vendor is not 1a86"
grep -q '^ID_MODEL_ID=55d4$' <<<"${controller_properties}" \
    || fail "controller USB product is not 55d4"
grep -q '^ID_SERIAL_SHORT=5B0B027499$' <<<"${controller_properties}" \
    || fail "controller USB serial does not match this robot"

python3 - "${CONFIG}" "${MAP_YAML}" "${NAV_PARAMS}" "${MAPPING_SAFETY}" "${MODE}" <<'PY'
import os
import sys

import yaml

config_path, map_path, nav_params_path, mapping_safety_path, mode = sys.argv[1:]
with open(config_path, encoding="utf-8") as stream:
    config = yaml.safe_load(stream)

def require(condition, message):
    if not condition:
        raise SystemExit(f"hardware/map validation failed: {message}")


require(isinstance(config, dict), f"invalid YAML document: {config_path}")
require(config.get("car_mode") == "mini_mec",
        f"car_mode={config.get('car_mode')!r}, expected 'mini_mec'")
require(config.get("lidar_type") == "rplidar_c1",
        f"lidar_type={config.get('lidar_type')!r}, expected 'rplidar_c1'")
lidar = config.get("rplidar_c1")
require(isinstance(lidar, dict), "missing rplidar_c1 configuration")
require(lidar.get("serial_port") == "/dev/wheeltec_lidar",
        f"serial_port={lidar.get('serial_port')!r}")
require(int(lidar.get("serial_baudrate", 0)) == 460800,
        f"serial_baudrate={lidar.get('serial_baudrate')!r}")
require(lidar.get("frame_id") == "laser",
        f"frame_id={lidar.get('frame_id')!r}")
require(lidar.get("scan_mode") == "Standard",
        f"scan_mode={lidar.get('scan_mode')!r}")
require(lidar.get("inverted") is False,
        f"inverted={lidar.get('inverted')!r}")
require(lidar.get("angle_compensate") is True,
        f"angle_compensate={lidar.get('angle_compensate')!r}")
require(lidar.get("enable_angle_crop_func") is True,
        f"enable_angle_crop_func={lidar.get('enable_angle_crop_func')!r}")
require(float(lidar.get("angle_crop_min", -1.0)) == 90.0,
        f"angle_crop_min={lidar.get('angle_crop_min')!r}")
require(float(lidar.get("angle_crop_max", -1.0)) == 270.0,
        f"angle_crop_max={lidar.get('angle_crop_max')!r}")
require(str(lidar.get("usb_vendor_id", "")).lower() == "10c4",
        f"usb_vendor_id={lidar.get('usb_vendor_id')!r}")
require(str(lidar.get("usb_product_id", "")).lower() == "ea60",
        f"usb_product_id={lidar.get('usb_product_id')!r}")
require(lidar.get("usb_serial") == "ba668b80e173ef119897d08c8fcc3fa0",
        f"usb_serial={lidar.get('usb_serial')!r}")

print("hardware YAML: mini_mec + rplidar_c1 (460800, laser, /scan)")
if mode != "mapping":
    with open(map_path, encoding="utf-8") as stream:
        map_config = yaml.safe_load(stream)
    require(isinstance(map_config, dict), f"invalid map YAML: {map_path}")
    image_path = map_config.get("image")
    require(isinstance(image_path, str) and image_path,
            f"missing image entry in {map_path}")
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_path), image_path)
    require(os.path.isfile(image_path), f"map image not found: {image_path}")
    require(os.path.getsize(image_path) > 0, f"empty map image: {image_path}")
    print(f"map: {map_path} -> {image_path}")
else:
    print("map: not required for a new mapping session")

if mode != "mapping":
    with open(nav_params_path, encoding="utf-8") as stream:
        nav = yaml.safe_load(stream)
    require(isinstance(nav, dict), f"invalid navigation YAML: {nav_params_path}")
    velocity = nav.get("velocity_smoother", {}).get("ros__parameters", {})
    require(velocity.get("min_velocity") == [0.0, 0.0, -1.0],
            f"navigation min_velocity={velocity.get('min_velocity')!r}")
    behaviors = nav.get("behavior_server", {}).get("ros__parameters", {})
    require(behaviors.get("behavior_plugins") == ["wait"],
            f"behavior_plugins={behaviors.get('behavior_plugins')!r}")
    follow_path = nav.get("controller_server", {}).get("ros__parameters", {}).get("FollowPath", {})
    require(float(follow_path.get("vx_min", -1.0)) >= 0.0,
            f"controller vx_min={follow_path.get('vx_min')!r}")
    require(follow_path.get("allow_reversing") is False,
            f"allow_reversing={follow_path.get('allow_reversing')!r}")
    print("navigation safety: reverse disabled in controller, smoother and recoveries")
else:
    with open(mapping_safety_path, encoding="utf-8") as stream:
        mapping_safety = yaml.safe_load(stream)
    require(isinstance(mapping_safety, dict),
            f"invalid mapping safety YAML: {mapping_safety_path}")
    mapping_velocity = mapping_safety.get("velocity_smoother", {}).get("ros__parameters", {})
    require(mapping_velocity.get("max_velocity") == [0.15, 0.0, 0.35],
            f"mapping max_velocity={mapping_velocity.get('max_velocity')!r}")
    require(mapping_velocity.get("min_velocity") == [0.0, 0.0, -0.35],
            f"mapping min_velocity={mapping_velocity.get('min_velocity')!r}")
    mapping_collision = mapping_safety.get("collision_monitor", {}).get("ros__parameters", {})
    require(mapping_collision.get("scan", {}).get("topic") == "/scan",
            f"mapping collision scan={mapping_collision.get('scan', {}).get('topic')!r}")
    print("mapping safety: reverse/lateral disabled; forward 0.15 m/s, yaw 0.35 rad/s")
PY

packages=(turn_on_wheeltec_robot rplidar_ros wheeltec_nav2 nav2_map_server)
if [[ "${MODE}" == "mapping" ]]; then
    case "${MAPPER}" in
        gmapping) packages+=(slam_gmapping) ;;
        cartographer) packages+=(wheeltec_cartographer) ;;
        *) fail "mapper must be gmapping or cartographer" ;;
    esac
    packages+=(nav2_velocity_smoother nav2_collision_monitor nav2_lifecycle_manager wheeltec_robot_keyboard)
else
    packages+=(slam_gmapping wheeltec_cartographer)
    if [[ "${MODE}" == "navigation" ]]; then
        packages+=(nav2_velocity_smoother nav2_collision_monitor nav2_lifecycle_manager)
    fi
fi
for package in "${packages[@]}"; do
    prefix="$(ros2 pkg prefix "${package}" 2>/dev/null)" \
        || fail "ROS package not found: ${package}"
    echo "package ${package}: ${prefix}"
done

if [[ "${MODE}" == "mapping" || "${MODE}" == "navigation" ]]; then
    running="$(pgrep -af 'competition_system\.launch\.py|wheeltec_mapping\.launch\.py|wheeltec_nav2(_for_slam)?\.launch\.py|slam_gmapping\.launch\.py|cartographer\.launch\.py|component_container(_isolated)?.*nav2_container|/turn_on_wheeltec_robot/.*/wheeltec_robot_node|/rplidar_ros/.*/rplidar_node|/ydlidar_ros2_driver/.*/ydlidar_ros2_driver_node|/nav2_controller/.*/controller_server|/nav2_planner/.*/planner_server|/nav2_bt_navigator/.*/bt_navigator|/nav2_behaviors/.*/behavior_server|/nav2_waypoint_follower/.*/waypoint_follower|/nav2_waypoint_cycle/.*/nav2_waypoint_cycle|/nav2_velocity_smoother/.*/velocity_smoother|/nav2_collision_monitor/.*/collision_monitor|/nav2_lifecycle_manager/.*/lifecycle_manager|/nav2_map_server/.*/map_server|/nav2_amcl/.*/amcl|/slam_gmapping/.*/slam_gmapping|/cartographer_ros/.*/(cartographer_node|occupancy_grid_node)|/robot_state_publisher/.*/robot_state_publisher|/robot_localization/.*/ekf_node|/tf2_ros/.*/static_transform_publisher|/joint_state_publisher/.*/joint_state_publisher|/home_patrol/.*/(home_)?navigation_manager|/home_patrol/.*/patrol_node|/robot_mqtt_bridge/.*/unified_bridge|/wheeltec_ui_dashboard/.*/ui_dashboard|/jobot_mic/.*/myagv_mic_node|/jarvis_voice/.*/(jarvis_node|air_sensor_node)|/wheeltec_robot_keyboard/.*/wheeltec_keyboard' || true)"
    if [[ -n "${running}" ]]; then
        printf 'ERROR: %s requires an idle ROS hardware stack:\n%s\n' \
            "${MODE}" "${running}" >&2
        exit 1
    fi

    if ! topic_list="$(timeout 8 ros2 topic list 2>/dev/null)"; then
        fail "ROS graph query failed; refusing to assume the velocity topics are idle"
    fi
    for velocity_topic in /cmd_vel /cmd_vel_nav /cmd_vel_raw; do
        if ! grep -Fxq "${velocity_topic}" <<<"${topic_list}"; then
            continue
        fi
        if ! topic_info="$(timeout 5 ros2 topic info "${velocity_topic}" 2>/dev/null)"; then
            fail "unable to inspect existing topic ${velocity_topic}"
        fi
        publisher_count="$(awk '/Publisher count:/ {print $3}' <<<"${topic_info}")"
        [[ "${publisher_count}" =~ ^[0-9]+$ ]] \
            || fail "invalid publisher count for ${velocity_topic}: ${publisher_count:-missing}"
        [[ "${publisher_count}" -eq 0 ]] \
            || fail "${velocity_topic} still has ${publisher_count} publisher(s); stop all control sources first"
    done

    if grep -Fxq '/map' <<<"${topic_list}"; then
        if ! map_info="$(timeout 5 ros2 topic info /map 2>/dev/null)"; then
            fail "unable to inspect existing /map topic"
        fi
        map_publishers="$(awk '/Publisher count:/ {print $3}' <<<"${map_info}")"
        [[ "${map_publishers}" =~ ^[0-9]+$ ]] \
            || fail "invalid /map publisher count: ${map_publishers:-missing}"
        [[ "${map_publishers}" -eq 0 ]] \
            || fail "/map still has ${map_publishers} publisher(s); stop map_server/SLAM first"
    fi
fi

echo "Preflight passed (${MODE})."
