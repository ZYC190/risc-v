#!/usr/bin/env python3
"""Replace the 32-bit type-punning InvSqrt with portable floating math."""

import os
from pathlib import Path
import re
import sys
import tempfile


def fail(message: str) -> None:
    raise SystemExit(f"quaternion source hardening failed: {message}")


check_only = False
arguments = sys.argv[1:]
if arguments[:1] == ["--check"]:
    check_only = True
    arguments = arguments[1:]
if len(arguments) != 1:
    fail("usage: harden_quaternion_source.py [--check] SOURCE_FILE")

path = Path(arguments[0]).resolve()
expected = Path(
    "/home/zyc/robot2/src/turn_on_wheeltec_robot/src/Quaternion_Solution.cpp"
)
if path != expected:
    fail(f"unexpected target: {path}")

text = path.read_text(encoding="utf-8")
if "#include <cmath>" not in text:
    include_anchor = '#include "turn_on_wheeltec_robot/Quaternion_Solution.h"\n'
    if text.count(include_anchor) != 1:
        fail("header include anchor is not unique")
    text = text.replace(include_anchor, include_anchor + "#include <cmath>\n", 1)

replacement = """float InvSqrt(float number)
{
  // The upstream long-based bit hack reads 8 bytes on riscv64 and violates
  // strict aliasing. At 20 Hz, the portable square root is both safe and fast.
  if (!(number > 0.0f)) {
    return 0.0f;
  }
  return 1.0f / std::sqrt(number);
}"""
pattern = re.compile(r"(?ms)^float InvSqrt\(float number\)\n\{.*?^\}")
text, count = pattern.subn(replacement, text, count=1)
if count != 1:
    fail("InvSqrt function was not uniquely found")
if "volatile long i" in text or "* (( long * ) &y)" in text:
    fail("unsafe long type-punning remains")
if "return 1.0f / std::sqrt(number);" not in text:
    fail("portable InvSqrt replacement is missing")

if check_only:
    print(f"Portable riscv64 InvSqrt transformation check passed: {path}")
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

print(f"Portable riscv64 InvSqrt installed: {path}")
