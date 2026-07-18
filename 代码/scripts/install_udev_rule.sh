#!/usr/bin/env bash
set -euo pipefail

ROBOT_ROOT="${ROBOT_ROOT:-/home/zyc/robot2}"
RULE_SOURCE="${ROBOT_ROOT}/config/99-wheeltec-pro.rules"
RULE_TARGET="/etc/udev/rules.d/99-wheeltec-pro.rules"
EXPECTED_VENDOR="10c4"
EXPECTED_PRODUCT="ea60"
EXPECTED_SERIAL="ba668b80e173ef119897d08c8fcc3fa0"

fail() {
    echo "ERROR: $*" >&2
    exit 1
}

[[ -f "${RULE_SOURCE}" ]] || fail "missing ${RULE_SOURCE}"
id -nG | tr ' ' '\n' | grep -Fxq dialout \
    || fail "the invoking user must belong to the dialout group"

# Discover the physical CP2102N even when the wheeltec_lidar alias does not
# exist yet (first install or recovery from a broken rule).
lidar_device=""
for candidate in /dev/ttyUSB* /dev/ttyACM*; do
    [[ -e "${candidate}" ]] || continue
    properties="$(udevadm info --query=property --name="${candidate}" 2>/dev/null || true)"
    if grep -q "^ID_VENDOR_ID=${EXPECTED_VENDOR}$" <<<"${properties}" \
        && grep -q "^ID_MODEL_ID=${EXPECTED_PRODUCT}$" <<<"${properties}" \
        && grep -q "^ID_SERIAL_SHORT=${EXPECTED_SERIAL}$" <<<"${properties}"; then
        lidar_device="$(readlink -f "${candidate}")"
        break
    fi
done
[[ -n "${lidar_device}" ]] || fail "the expected RPLIDAR CP2102N is not connected"

timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -f "${RULE_TARGET}" ]]; then
    sudo cp -a "${RULE_TARGET}" "${RULE_TARGET}.bak_${timestamp}"
fi
sudo install -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sudo udevadm settle --timeout=10

[[ -e /dev/wheeltec_lidar ]] || fail "udev reload did not create /dev/wheeltec_lidar"
alias_target="$(readlink -f /dev/wheeltec_lidar)"
[[ "${alias_target}" == "${lidar_device}" ]] \
    || fail "lidar alias points to ${alias_target}, expected ${lidar_device}"
properties="$(udevadm info --query=property --name=/dev/wheeltec_lidar)"
grep -q "^ID_SERIAL_SHORT=${EXPECTED_SERIAL}$" <<<"${properties}" \
    || fail "installed alias does not match the expected serial"
[[ "$(stat -c '%a' "${alias_target}")" == "660" ]] \
    || fail "lidar device mode is not 0660"
[[ "$(stat -c '%G' "${alias_target}")" == "dialout" ]] \
    || fail "lidar device group is not dialout"

echo "Installed ${RULE_TARGET}"
echo "/dev/wheeltec_lidar -> ${alias_target} (0660, dialout)"
