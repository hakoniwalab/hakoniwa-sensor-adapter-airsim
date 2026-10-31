#!/usr/bin/env python3
"""Pinned Project AirSim core_sim sources for the native adapters.

The native adapters compile part of Project AirSim's core_sim unchanged. This repository does
not contain those files. It commits only a lock file (third_party/projectairsim/lock.json):
the upstream repository, the commit SHA, and the path and sha256 of every file. The files are
fetched from that exact commit into a cache (.cache/projectairsim/<sha>/), which is not committed.

  python tools/projectairsim_lock.py fetch           download the locked files into the cache, check every sha256
  python tools/projectairsim_lock.py verify          check the cache against the lock file
  python tools/projectairsim_lock.py lock --clone DIR
                                                     write the lock file from a local Project AirSim checkout
                                                     (its HEAD commit): the sources below and every file they
                                                     include. Run only to move to another upstream commit.
"""

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import urllib.request
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
LOCK = REPO_ROOT / "third_party" / "projectairsim" / "lock.json"
CACHE_ROOT = REPO_ROOT / ".cache" / "projectairsim"
UPSTREAM = "iamaisim/ProjectAirSim"

# core_sim sources compiled by the native adapters: the sensors, the managers they need, every
# message (the managers reference them) and the utilities. Paths are relative to core_sim/.
SOURCES = [
    "src/geodetic_converter.cpp", "src/json_utils.cpp", "src/logger.cpp", "src/math_utils.cpp",
    "src/message/airspeed_message.cpp", "src/message/barometer_message.cpp", "src/message/battery_message.cpp",
    "src/message/camera_info_message.cpp", "src/message/clock_message.cpp",
    "src/message/collision_info_message.cpp", "src/message/distance_sensor_message.cpp",
    "src/message/flight_control_rc_input_message.cpp", "src/message/flight_control_setpoint_message.cpp",
    "src/message/float_message.cpp", "src/message/gps_message.cpp", "src/message/image_message.cpp",
    "src/message/imu_message.cpp", "src/message/int8_message.cpp", "src/message/int_list_message.cpp",
    "src/message/joint_state_message.cpp", "src/message/kinematics_message.cpp", "src/message/lidar_message.cpp",
    "src/message/magnetometer_message.cpp", "src/message/message.cpp", "src/message/message_utils.cpp",
    "src/message/physics_state_message.cpp", "src/message/pose_message.cpp",
    "src/message/pose_stamped_message.cpp", "src/message/radar_detection_message.cpp",
    "src/message/radar_track_message.cpp", "src/message/ready_state_message.cpp",
    "src/message/request_message.cpp", "src/message/response_failure_message.cpp",
    "src/message/response_success_message.cpp", "src/message/rotor_message.cpp",
    "src/message/transform_message.cpp", "src/sensors/barometer.cpp", "src/sensors/gps.cpp",
    "src/sensors/imu.cpp", "src/sensors/magnetometer.cpp", "src/sensors/sensor.cpp", "src/service.cpp",
    "src/service_manager.cpp", "src/state_manager.cpp", "src/string_utils.cpp", "src/topic.cpp",
    "src/topic_manager.cpp", "src/transforms/transform_utils.cpp",
]
# Headers declaring the functions that src/native/core_sim_stubs.cpp stands in for.
STUB_HEADERS = [
    "include/core_sim/actor.hpp", "include/core_sim/actor/robot.hpp", "include/core_sim/actuators/actuator.hpp",
    "include/core_sim/actuators/rotor.hpp", "include/core_sim/client_authorization.hpp",
    "include/core_sim/clock.hpp", "include/core_sim/scene.hpp",
]
# Repository files kept with the sources (licence and notices).
NOTICES = ["LICENSE", "NOTICE.txt"]
INCLUDE_DIRS = ["include", "src"]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def trace(core_sim: Path) -> list[str]:
    seen: set[str] = set()
    stack = SOURCES + STUB_HEADERS
    while stack:
        rel = stack.pop()
        if rel in seen:
            continue
        if not (core_sim / rel).is_file():
            raise SystemExit(f"missing in the checkout: core_sim/{rel}")
        seen.add(rel)
        text = (core_sim / rel).read_text(encoding="utf-8", errors="replace")
        for name in re.findall(r'#include\s+"([^"]+)"', text):
            for base in [os.path.dirname(rel)] + INCLUDE_DIRS:
                candidate = os.path.normpath(os.path.join(base, name))
                if (core_sim / candidate).is_file():
                    stack.append(candidate)
                    break
    return sorted(seen)


def write_lock(clone: Path) -> None:
    commit = subprocess.run(["git", "-C", str(clone), "rev-parse", "HEAD"], check=True, capture_output=True,
                            text=True).stdout.strip()
    status = subprocess.run(["git", "-C", str(clone), "status", "--porcelain", "--", "core_sim", *NOTICES],
                            check=True, capture_output=True, text=True).stdout.strip()
    if status:
        raise SystemExit(f"the checkout has local changes; lock only an unmodified commit:\n{status}")
    files = [{"path": f"core_sim/{rel}", "sha256": sha256(clone / "core_sim" / rel)} for rel in trace(clone / "core_sim")]
    files += [{"path": name, "sha256": sha256(clone / name)} for name in NOTICES]
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    LOCK.write_text(json.dumps({
        "upstream": UPSTREAM,
        "commit": commit,
        "license": "MIT (LICENSE, NOTICE.txt)",
        "sources": [f"core_sim/{rel}" for rel in SOURCES],
        "files": files,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"locked {len(files)} files at {UPSTREAM}@{commit} -> {LOCK.relative_to(REPO_ROOT)}")


def read_lock() -> dict:
    if not LOCK.is_file():
        raise SystemExit(f"no lock file: {LOCK.relative_to(REPO_ROOT)}")
    return json.loads(LOCK.read_text(encoding="utf-8"))


def cache_dir(lock: dict) -> Path:
    return CACHE_ROOT / lock["commit"]


def fetch() -> None:
    lock = read_lock()
    root = cache_dir(lock)
    fetched = 0
    for entry in lock["files"]:
        target = root / entry["path"]
        if target.is_file() and sha256(target) == entry["sha256"]:
            continue
        url = f"https://raw.githubusercontent.com/{lock['upstream']}/{lock['commit']}/{entry['path']}"
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
        digest = hashlib.sha256(data).hexdigest()
        if digest != entry["sha256"]:
            raise SystemExit(f"sha256 mismatch for {entry['path']}: expected {entry['sha256']}, got {digest}")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        fetched += 1
    print(f"{len(lock['files'])} files of {lock['upstream']}@{lock['commit']} in {root.relative_to(REPO_ROOT)}"
          f" ({fetched} downloaded)")


def verify() -> None:
    lock = read_lock()
    root = cache_dir(lock)
    bad = [e["path"] for e in lock["files"]
           if not (root / e["path"]).is_file() or sha256(root / e["path"]) != e["sha256"]]
    if bad:
        print("missing or changed in the cache:", *bad, sep="\n  ")
        sys.exit(1)
    print(f"cache matches the lock: {len(lock['files'])} files at {lock['commit']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("fetch")
    sub.add_parser("verify")
    lock_parser = sub.add_parser("lock")
    lock_parser.add_argument("--clone", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "fetch":
        fetch()
    elif args.command == "verify":
        verify()
    else:
        write_lock(args.clone.resolve())


if __name__ == "__main__":
    main()
