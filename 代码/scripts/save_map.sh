#!/usr/bin/env bash
set -eo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
MAP_DIR="${ROBOT_ROOT}/src/wheeltec_robot_nav2/map"
MAP_NAME="${MAP_NAME:-WHEELTEC}"
MAP_PREFIX="${MAP_DIR}/${MAP_NAME}"
BACKUP_ROOT="${ROBOT_ROOT}/map_backups"

[[ "${MAP_NAME}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] \
    && [[ ${#MAP_NAME} -le 64 ]] \
    || { echo "ERROR: MAP_NAME must be 1-64 safe filename characters." >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "${ROBOT_ROOT}/install/setup.bash"
set -u

if ! topic_info="$(timeout 8 ros2 topic info /map --verbose 2>/dev/null)"; then
    echo "ERROR: unable to inspect /map; keep SLAM running while saving." >&2
    exit 1
fi
publisher_count="$(awk '/Publisher count:/ {print $3}' <<<"${topic_info}")"
[[ "${publisher_count}" == "1" ]] \
    || { echo "ERROR: /map must have exactly one publisher, found ${publisher_count:-unknown}." >&2; exit 1; }
publisher_node="$(awk '/Node name:/ {node=$3} /Endpoint type: PUBLISHER/ {print node; exit}' <<<"${topic_info}")"
case "${publisher_node}" in
    slam_gmapping|occupancy_grid_node) ;;
    *)
        echo "ERROR: /map publisher ${publisher_node:-unknown} is not an active supported SLAM node." >&2
        exit 1
        ;;
esac

mkdir -p "${MAP_DIR}" "${BACKUP_ROOT}"
command -v flock >/dev/null 2>&1 \
    || { echo "ERROR: flock is required to serialize map saves." >&2; exit 1; }
exec 9>"${MAP_DIR}/.map-save.lock"
flock -n 9 \
    || { echo "ERROR: another map save is already in progress." >&2; exit 1; }

temporary_dir="$(mktemp -d "${MAP_DIR}/.map-save.XXXXXX")"
trap 'rm -rf -- "${temporary_dir}"' EXIT
temporary_prefix="${temporary_dir}/${MAP_NAME}"
timestamp="$(date +%Y%m%d_%H%M%S)"
temporary_token="$(basename "${temporary_dir}")"
temporary_token="${temporary_token##*.}"
generation_image="${MAP_NAME}.${timestamp}_${temporary_token}.pgm"
generation_image_path="${MAP_DIR}/${generation_image}"

timeout --signal=INT --kill-after=5s 45s \
    ros2 run nav2_map_server map_saver_cli \
    -f "${temporary_prefix}" \
    --ros-args \
    -p save_map_timeout:=20.0 \
    -p free_thresh_default:=0.196

python3 - "${temporary_prefix}.yaml" "${temporary_prefix}.pgm" \
    "${generation_image_path}" <<'PY'
import os
import sys

import yaml

yaml_path, image_path, generation_image_path = sys.argv[1:]
for path in (yaml_path, image_path):
    if not os.path.isfile(path):
        raise SystemExit(f"candidate file not found: {path}")
    if os.path.getsize(path) <= 0:
        raise SystemExit(f"candidate file is empty: {path}")
with open(yaml_path, encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
if not isinstance(config, dict):
    raise SystemExit(f"invalid candidate map YAML: {yaml_path}")
actual_image = config.get("image")
expected_image = os.path.basename(image_path)
if actual_image != expected_image:
    raise SystemExit(
        f"candidate image entry {actual_image!r} != {expected_image!r}"
    )
try:
    resolution = float(config.get("resolution", 0.0))
except (TypeError, ValueError) as exc:
    raise SystemExit(f"invalid map resolution: {config.get('resolution')!r}") from exc
if resolution <= 0.0:
    raise SystemExit(f"map resolution must be positive: {resolution}")
config["image"] = generation_image_path
with open(yaml_path, "w", encoding="utf-8") as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
print(f"validated candidate map: {yaml_path}")
PY

backup_dir="$(mktemp -d "${BACKUP_ROOT}/${timestamp}_before_${MAP_NAME}.XXXXXX")"
previous_image_path=""
if [[ -f "${MAP_PREFIX}.yaml" ]]; then
    previous_image_path="$(python3 - "${MAP_PREFIX}.yaml" "${MAP_DIR}" <<'PY'
import os
import sys

import yaml

yaml_path, map_dir = sys.argv[1:]
with open(yaml_path, encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
if not isinstance(config, dict) or not isinstance(config.get("image"), str):
    raise SystemExit(f"invalid current map YAML: {yaml_path}")
image_path = config["image"]
if not os.path.isabs(image_path):
    image_path = os.path.join(map_dir, image_path)
real_map_dir = os.path.realpath(map_dir)
real_image_path = os.path.realpath(image_path)
if os.path.commonpath((real_map_dir, real_image_path)) != real_map_dir:
    raise SystemExit(f"current map image escapes map directory: {image_path}")
print(real_image_path)
PY
)"
    cp -a -- "${MAP_PREFIX}.yaml" "${backup_dir}/"
fi
if [[ -n "${previous_image_path}" && -f "${previous_image_path}" ]]; then
    cp -a -- "${previous_image_path}" "${backup_dir}/"
elif [[ -f "${MAP_PREFIX}.pgm" ]]; then
    cp -a -- "${MAP_PREFIX}.pgm" "${backup_dir}/"
fi

# Publish an immutable, generation-specific image first.  Only then atomically
# replace WHEELTEC.yaml so an interruption always leaves the old YAML/image
# pair valid; the YAML rename is the single commit point consumed by Nav2.
chmod 0644 "${temporary_prefix}.pgm" "${temporary_prefix}.yaml"
[[ ! -e "${generation_image_path}" ]] \
    || { echo "ERROR: generation image already exists: ${generation_image_path}" >&2; exit 1; }
mv -- "${temporary_prefix}.pgm" "${generation_image_path}"
mv -f -- "${temporary_prefix}.yaml" "${MAP_PREFIX}.yaml"

echo "Map saved: ${MAP_PREFIX}.yaml -> ${generation_image_path}"
echo "Previous map backup: ${backup_dir}"
