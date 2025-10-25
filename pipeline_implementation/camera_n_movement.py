# so101_sequence_capture.py
# Usage example:
#   python so101_sequence_capture.py --file follower_test.json --id my_follower \
#       --loops 3 --rate_hz 20 --speed_deg_s 8 --settle_s 0.7 \
#       --cam_device /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
#
# Notes:
# - Creates an output folder named with current datetime (YYYYmmdd_HHMMSS) next to where you run it.
# - Joint units are DEGREES internally (gripper 0..100). If your file has radians,
#   put "units": "radians" at top-level or pass --units radians.
# - Motion uses cosine easing + per-joint speed cap for controlled moves.
# - Camera is opened once; we warm up, then capture one frame per pose.

import argparse
import json
import math
import os
import re
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import cv2
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

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
    # Accept two shapes:
    # (A) flat:   { "shoulder_pan":..., "name":"...", "timestamp":"..." }
    # (B) nested: { "joints": {...}, "name":"...", "timestamp":"..." }
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
        name = f"pose_{idx:03d}" if not ts else f"pose_{idx:03d}_{ts.replace(':','-')}"
    return Pose(name=name, timestamp=ts, joints=joints)

def load_pose_file(path: str, units_override: Optional[str] = None) -> PoseFile:
    with open(path, "r") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("Top-level JSON must be an object.")

    robot = data.get("robot")
    port  = data.get("port")
    units = _norm_units(units_override) if units_override else _norm_units(data.get("units"))

    raw = data.get("poses")
    if not isinstance(raw, list) or not raw:
        raise ValueError("JSON must contain a non-empty 'poses' array.")

    poses: List[Pose] = []
    last: Optional[Pose] = None
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
        t = k / steps
        if ease:
            t = 0.5 - 0.5 * math.cos(math.pi * t)  # cosine ease-in/out
        action = {f"{j}.pos": float(cur[j] + diffs[j] * t) for j in JOINTS}
        robot.send_action(action)
        time.sleep(dt)

    return {j: float(goal[j]) for j in JOINTS}

# ----------------------------- Camera helpers -----------------------------

def open_camera(device: str, width: int, height: int, fps: int, warmup_s: float, warmup_frames: int):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {device}")
    # Request MJPG @ WxH / FPS
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    # Warm up
    if warmup_s > 0:
        time.sleep(warmup_s)
    for _ in range(max(0, warmup_frames)):
        cap.read()
    return cap

def snap_image(cap, out_path: Path) -> Path:
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Failed to grab frame")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), frame)
    return out_path

def sanitize(name: str) -> str:
    # safe for filenames
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("_")

# ----------------------------- Main -----------------------------

def main(args):
    # 0) Create timestamped output folder
    out_dir = Path(args.out_dir or datetime.now().strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"Output dir: {out_dir.resolve()}")

    # 1) Load poses
    pf = load_pose_file(args.file, units_override=(args.units or None))
    port = args.port or pf.port or "/dev/ttyACM0"
    print(f"Loaded poses from: {args.file}")
    print(f"  Robot: {pf.robot or '(unspecified)'}  Port: {port}  Units: {pf.units}")
    print(f"  Count: {len(pf.poses)} poses")

    # (Optional) store a copy of the JSON next to images for traceability
    try:
        backup_path = out_dir / Path(args.file).name
        if not backup_path.exists():
            backup_path.write_text(Path(args.file).read_text())
    except Exception as e:
        print(f"Warning: could not copy JSON into output folder: {e}", file=sys.stderr)

    # 2) Connect robot
    cfg = SO101FollowerConfig(
        port=port,
        id=args.id,
        use_degrees=True,
        disable_torque_on_disconnect=True,
        # You can uncomment if you also want device-side clipping per tick:
        # max_relative_target=5.0,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)

    # SIGINT/SIGTERM clean-up
    def cleanup(*_):
        try:
            if 'cap' in locals() and cap is not None:
                cap.release()
            robot.disconnect()
        finally:
            sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # 3) Open camera once
    cap = open_camera(
        device=args.cam_device,
        width=args.cam_width,
        height=args.cam_height,
        fps=args.cam_fps,
        warmup_s=args.cam_warmup_s,
        warmup_frames=args.cam_warmup_frames,
    )
    print(f"Camera ready: {args.cam_device} ({args.cam_width}x{args.cam_height}@{args.cam_fps})")

    # 4) Determine current joint angles once for smooth first move
    start_goal = {j: float(pf.poses[0].joints[j]) for j in JOINTS}
    try:
        obs = robot.get_observation()
        current = {j: float(obs.get(f"{j}.pos", start_goal[j])) for j in JOINTS}
    except Exception:
        current = dict(start_goal)

    # 5) Loops over the sequence
    for loop_idx in range(1, args.loops + 1):
        print(f"\n=== Loop {loop_idx}/{args.loops} ===")
        for pose_idx, pose in enumerate(pf.poses, start=1):
            goal = {j: float(pose.joints[j]) for j in JOINTS}
            label = f"{pose_idx:03d}_{sanitize(pose.name)}"
            print(f"Moving to: {pose.name} ({pose_idx}/{len(pf.poses)})")
            current = goto_slow(
                robot, current, goal,
                rate_hz=args.rate_hz,
                speed_deg_s=args.speed_deg_s,
                ease=(not args.no_ease),
            )
            # settle then snapshot
            time.sleep(args.settle_s)
            img_name = f"loop{loop_idx:03d}_{label}.jpg"
            img_path = out_dir / img_name
            snap_image(cap, img_path)
            print(f"Reached: {pose.name} → saved {img_path.name}")

    # Clean exit
    cap.release()
    robot.disconnect()
    print("\nDone.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Play SO-101 poses from JSON, loop N times, and capture a photo at each pose.")
    parser.add_argument("--file", required=True, help="Path to JSON pose file")
    parser.add_argument("--id", default="my_follower", help="Follower calibration id")
    parser.add_argument("--port", default="", help="Override serial port (else JSON->port or /dev/ttyACM0)")
    parser.add_argument("--units", default="", help="Override file units: 'degrees' or 'radians'")
    parser.add_argument("--loops", type=int, default=1, help="How many times to run the full pose sequence")
    parser.add_argument("--rate_hz", type=float, default=20.0, help="Command rate (Hz)")
    parser.add_argument("--speed_deg_s", type=float, default=8.0, help="Max joint speed (deg/s); gripper shares units/s")
    parser.add_argument("--settle_s", type=float, default=0.6, help="Pause at each pose before taking a picture (s)")
    parser.add_argument("--no_ease", action="store_true", help="Disable cosine easing (use linear interpolation)")
    parser.add_argument("--out_dir", default="", help="Output directory (default: current datetime stamp)")
    # Camera args
    parser.add_argument("--cam_device", default="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0")
    parser.add_argument("--cam_width", type=int, default=1280)
    parser.add_argument("--cam_height", type=int, default=720)
    parser.add_argument("--cam_fps", type=int, default=30)
    parser.add_argument("--cam_warmup_s", type=float, default=0.2)
    parser.add_argument("--cam_warmup_frames", type=int, default=5)
    args = parser.parse_args()
    main(args)
