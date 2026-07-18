#!/usr/bin/env python3
"""Atomically enforce the no-reverse contract in the active Nav2 YAML."""

import os
from pathlib import Path
import re
import sys
import tempfile

import yaml


def fail(message: str) -> None:
    raise SystemExit(f"navigation parameter hardening failed: {message}")


check_only = False
arguments = sys.argv[1:]
if arguments[:1] == ["--check"]:
    check_only = True
    arguments = arguments[1:]
if len(arguments) != 1:
    fail("usage: harden_navigation_params.py [--check] PARAM_FILE")

path = Path(arguments[0]).resolve()
if path != Path("/home/zyc/robot2/src/wheeltec_robot_nav2/param/wheeltec_params/param_mini_mec.yaml"):
    fail(f"unexpected target: {path}")

text = path.read_text(encoding="utf-8")
text = text.replace(
    "vx_min: 0.0       #导航不主动倒车；倒车仅交给恢复行为",
    "vx_min: 0.0       #后向激光被屏蔽，导航和恢复均禁止倒车",
)
text = text.replace(
    "远离障碍的旋转或后退仍可执行，避免窄处被全方向锁死。",
    "远离障碍的低速转向仍可执行；后退由控制器和速度平滑器共同禁用。",
)

# Limit the replacement to the velocity_smoother section.  Preserve the
# hand-tuned competition YAML and its comments instead of serializing it anew.
velocity_match = re.search(
    r"(?ms)^velocity_smoother:\n(?P<body>.*?)(?=^collision_monitor:)", text
)
if velocity_match is None:
    fail("velocity_smoother section not found")
velocity_body = velocity_match.group("body")
velocity_body, replacement_count = re.subn(
    r"(?m)^(\s*min_velocity:)\s*\[[^\n]*\]",
    r"\1 [0.0, 0.0, -1.0]  # RPLIDAR rear half-plane is masked: never reverse",
    velocity_body,
    count=1,
)
if replacement_count != 1:
    fail("velocity_smoother min_velocity was not uniquely found")
text = text[: velocity_match.start("body")] + velocity_body + text[velocity_match.end("body") :]

behavior_match = re.search(
    r"(?ms)^behavior_server:\n(?P<body>.*?)(?=^# behavior_server reference:)", text
)
if behavior_match is None:
    fail("behavior_server section not found")
behavior_body = behavior_match.group("body")
behavior_body = re.sub(
    r"(?ms)^\s{4}# BEGIN CODEX NO-REVERSE BEHAVIORS\n.*?^\s{4}# END CODEX NO-REVERSE BEHAVIORS\n",
    "",
    behavior_body,
)
anchor = "    cycle_frequency: 10.0\n"
if behavior_body.count(anchor) != 1:
    fail("behavior_server cycle_frequency anchor is not unique")
safety_block = (
    "    # BEGIN CODEX NO-REVERSE BEHAVIORS\n"
    "    # Rear lidar coverage is masked; do not load BackUp, DriveOnHeading,\n"
    "    # or Spin recovery actions. Normal controller heading changes remain.\n"
    "    behavior_plugins: [\"wait\"]\n"
    "    wait:\n"
    "      plugin: \"nav2_behaviors/Wait\"\n"
    "    # END CODEX NO-REVERSE BEHAVIORS\n"
)
behavior_body = behavior_body.replace(anchor, anchor + safety_block, 1)
text = text[: behavior_match.start("body")] + behavior_body + text[behavior_match.end("body") :]

document = yaml.safe_load(text)
if not isinstance(document, dict):
    fail("result is not a YAML mapping")
velocity = document.get("velocity_smoother", {}).get("ros__parameters", {})
if velocity.get("min_velocity") != [0.0, 0.0, -1.0]:
    fail(f"unexpected min_velocity: {velocity.get('min_velocity')!r}")
behaviors = document.get("behavior_server", {}).get("ros__parameters", {})
if behaviors.get("behavior_plugins") != ["wait"]:
    fail(f"unexpected behavior_plugins: {behaviors.get('behavior_plugins')!r}")
if behaviors.get("wait", {}).get("plugin") != "nav2_behaviors/Wait":
    fail("Wait behavior plugin is missing")
follow_path = document.get("controller_server", {}).get("ros__parameters", {}).get("FollowPath", {})
if float(follow_path.get("vx_min", -1.0)) < 0.0:
    fail(f"controller vx_min permits reverse: {follow_path.get('vx_min')!r}")
if follow_path.get("allow_reversing") is not False:
    fail(f"allow_reversing is not false: {follow_path.get('allow_reversing')!r}")

if check_only:
    print(f"Navigation no-reverse transformation check passed: {path}")
    raise SystemExit(0)

mode = path.stat().st_mode & 0o777
fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
try:
    with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(temporary_name, mode)
    os.replace(temporary_name, path)
    directory_fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
finally:
    if os.path.exists(temporary_name):
        os.unlink(temporary_name)

print(f"Navigation no-reverse contract verified: {path}")
