# so101_play_from_json_slow.py
# Usage example:
#   python so101_play_from_json_slow.py --file follower_test.json --id my_follower \
#     --rate_hz 20 --speed_deg_s 8 --pause_s 1.0
#
# Notes:
# - Joint units are DEGREES internally (gripper 0..100). If your file has radians,
#   put "units": "radians" at top-level or pass --units radians.
# - We do one read at the beginning for a smooth first move. If that read fails
#   (weak power), we fall back to using the first pose as the starting state.
# - Motion uses cosine easing + a per-joint speed cap for clean, controlled moves.

import argparse
import json
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

# Joint order used throughout (gripper treated as 0..100 "units")
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


# ----------------------------- JSON parsing -----------------------------

def _norm_units(u: Optional[str]) -> str:
    if not u:
        return "degrees"
    u = u.strip().lower()
    return "radians" if u in ("rad", "radian", "radians") else "degrees"


@dataclass
class Pose:
    name: str
    timestamp: Optional[str]
    joints: Dict[str, float]  # degrees for arm joints; gripper 0..100


@dataclass
class PoseFile:
    robot: Optional[str]
    port: Optional[str]
    units: str
    poses: List[Pose]


def parse_pose_obj(obj: dict, default_units: str, idx: int, last: Optional[Pose]) -> Pose:
    """Accept two shapes:
    (A) flat:   { "shoulder_pan": ..., "name": "...", "timestamp": "..." }
    (B) nested: { "joints": { ... }, "name": "...", "timestamp": "..." }
    Missing joints are carried forward from the previous pose (or set to 0).
    """
    units = _norm_units(obj.get("units", default_units))
    src = obj.get("joints", obj)

    joints: Dict[str, float] = {}
    for j in JOINTS:
        if j in src and src[j] is not None:
            val = float(src[j])
        else:
            val = float(last.joints[j]) if last else 0.0
        if j != "gripper" and units == "radians":
            val = math.degrees(val)
        joints[j] = val

    name = obj.get("name")
    ts = obj.get("timestamp")
    if not name:
        if ts:
            name = f"pose_{idx:03d}_{ts.replace(':','-')}"
        else:
            name = f"pose_{idx:03d}"
    return Pose(name=name, timestamp=ts, joints=joints)


def load_pose_file(path: str, units_override: Optional[str] = None) -> PoseFile:
    with open(path, "r") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Top-level JSON must be an object.")
    robot = data.get("robot")
    port = data.get("port")
    units = _norm_units(units_override) if units_override else _norm_units(data.get("units"))

    raw = data.get("poses")
    if not isinstance(raw, list) or not raw:
        raise ValueError("JSON must contain a non-empty 'poses' array.")

    poses: List[Pose] = []
    last = None
    for i, p in enumerate(raw, start=1):
        if not isinstance(p, dict):
            raise ValueError(f"poses[{i}] must be an object.")
        pose = parse_pose_obj(p, units, i, last)
        poses.append(pose)
        last = pose

    return PoseFile(robot=robot, port=port, units=units or "degrees", poses=poses)


# ----------------------------- Motion helpers -----------------------------

def goto_slow(robot: SO101Follower,
              cur: Dict[str, float],
              goal: Dict[str, float],
              rate_hz: float,
              speed_deg_s: float,
              ease: bool = True) -> Dict[str, float]:
    """
    Interpolate from 'cur' to 'goal' with a per-joint speed cap and optional cosine easing.
    Returns the commanded 'goal' dict (your new 'current').
    """
    dt = 1.0 / rate_hz
    per_tick = max(0.1, float(speed_deg_s) * dt)

    diffs = {j: float(goal[j]) - float(cur[j]) for j in JOINTS}
    max_span = max(abs(d) for d in diffs.values()) if diffs else 0.0
    steps = max(1, int(math.ceil(max_span / per_tick)))

    for k in range(1, steps + 1):
        t = k / steps  # 0..1
        if ease:
            # cosine ease-in/out -> smooth start/stop
            t = 0.5 - 0.5 * math.cos(math.pi * t)
        action = {}
        for j in JOINTS:
            val = cur[j] + diffs[j] * t
            action[f"{j}.pos"] = float(val)
        robot.send_action(action)
        time.sleep(dt)

    return {j: float(goal[j]) for j in JOINTS}


def main(path: str,
         robot_id: str,
         port_override: Optional[str],
         rate_hz: float,
         speed_deg_s: float,
         pause_s: float,
         units_override: Optional[str],
         check_only: bool,
         ease: bool):
    # Parse file
    pf = load_pose_file(path, units_override=units_override)
    port = port_override or pf.port or "/dev/ttyACM0"

    print(f"Loaded: {path}")
    print(f" Robot: {pf.robot or '(unspecified)'}")
    print(f" Port:  {port}")
    print(f" Units: {pf.units} (converted to degrees internally)")
    print(f" Poses: {len(pf.poses)}")
    for i, pose in enumerate(pf.poses, start=1):
        t = f" @ {pose.timestamp}" if pose.timestamp else ""
        print(f"  {i:>2}. {pose.name}{t}")

    if check_only:
        return

    cfg = SO101FollowerConfig(
        port=port,
        id=robot_id,
        use_degrees=True,                 # we keep everything in degrees internally
        disable_torque_on_disconnect=True,
        # Optional: uncomment if you want LeRobot to clip per-tick jumps additionally
        # max_relative_target=5.0,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)

    def cleanup(*_):
        try:
            robot.disconnect()
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # Starting pose
    start_pose = pf.poses[0]
    start_goal = {j: float(start_pose.joints[j]) for j in JOINTS}

    # Try to read current once for a smooth first move; fall back to start if read fails
    try:
        obs = robot.get_observation()
        current = {j: float(obs.get(f"{j}.pos", start_goal[j])) for j in JOINTS}
    except Exception:
        current = dict(start_goal)

    print(f"\nGoing to START: {start_pose.name}" + (f" @ {start_pose.timestamp}" if start_pose.timestamp else ""))
    current = goto_slow(robot, current, start_goal, rate_hz=rate_hz, speed_deg_s=speed_deg_s, ease=ease)
    print(f"Reached: {start_pose.name}")
    time.sleep(pause_s)

    # Remaining poses
    for pose in pf.poses[1:]:
        goal = {j: float(pose.joints[j]) for j in JOINTS}
        print(f"Going to: {pose.name}" + (f" @ {pose.timestamp}" if pose.timestamp else ""))
        current = goto_slow(robot, current, goal, rate_hz=rate_hz, speed_deg_s=speed_deg_s, ease=ease)
        print(f"Reached: {pose.name}")
        time.sleep(pause_s)

    cleanup()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Play SO-101 follower poses from JSON with smooth, speed-limited motion.")
    ap.add_argument("--file", required=True, help="Path to JSON pose file")
    ap.add_argument("--id", default="my_follower", help="Follower calibration id (from your calibration step)")
    ap.add_argument("--port", default="", help="Override serial port (else JSON->port or /dev/ttyACM0)")
    ap.add_argument("--rate_hz", type=float, default=20.0, help="Command rate (Hz)")
    ap.add_argument("--speed_deg_s", type=float, default=10.0, help="Max joint speed (deg/s); gripper uses same units/s")
    ap.add_argument("--pause_s", type=float, default=1.0, help="Pause between poses (s)")
    ap.add_argument("--units", default="", help="Override file units: 'degrees' or 'radians'")
    ap.add_argument("--check_only", action="store_true", help="Parse & print summary; do not move the robot")
    ap.add_argument("--no_ease", action="store_true", help="Disable cosine ease-in/out (use linear interpolation)")
    args = ap.parse_args()

    main(
        path=args.file,
        robot_id=args.id,
        port_override=args.port or None,
        rate_hz=args.rate_hz,
        speed_deg_s=args.speed_deg_s,
        pause_s=args.pause_s,
        units_override=(args.units or None),
        check_only=args.check_only,
        ease=(not args.no_ease),
    )
