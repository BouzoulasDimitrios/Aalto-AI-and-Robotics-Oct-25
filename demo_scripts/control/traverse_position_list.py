# so101_play_poses.py
# Run: python so101_play_poses.py --file /path/to/your.json --id my_follower --port /dev/ttyACM0
# Notes:
# - Positions are interpreted in DEGREES (gripper 0..100).
# - We move smoothly: each tick we step toward the goal with a max speed (deg/s).

import json, time, sys, signal, argparse, math, os
from typing import Dict, List
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]

def clamp_step(cur, goal, max_step):
    d = goal - cur
    if abs(d) <= max_step:
        return goal
    return cur + (max_step if d > 0 else -max_step)

def goto(robot: SO101Follower, goal: Dict[str, float], rate_hz: float, speed_deg_s: float, tol_deg: float, timeout_s: float = 3.0):
    """Incrementally walk from current obs toward goal at 'speed_deg_s', until within 'tol_deg' on all joints."""
    dt = 1.0 / rate_hz
    per_tick = speed_deg_s * dt
    t0 = time.time()
    while True:
        obs = robot.get_observation()  # observation keys like "shoulder_pan.pos", ...
        cur = {j: float(obs.get(f"{j}.pos", 0.0)) for j in JOINTS}
        # Build the next small step toward goal
        action = {}
        done = True
        for j in JOINTS:
            g = float(goal.get(j, cur[j]))  # if missing, hold current
            if math.isnan(g): g = cur[j]
            if abs(g - cur[j]) > tol_deg:
                done = False
            nxt = clamp_step(cur[j], g, per_tick)
            action[f"{j}.pos"] = float(nxt)
        robot.send_action(action)
        if done:
            return True
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(dt)

def main(file_path: str, port_cli: str, robot_id: str, rate_hz: float, speed_deg_s: float, tol_deg: float, pause_s: float):
    # Load JSON
    with open(file_path, "r") as f:
        data = json.load(f)
    poses: List[Dict[str, float]] = data.get("poses", [])
    if not poses:
        print("No poses found in JSON.")
        sys.exit(1)

    # Port selection: CLI overrides JSON-> 'port', else default /dev/ttyACM0
    port = port_cli or data.get("port") or "/dev/ttyACM0"

    cfg = SO101FollowerConfig(
        port=port,
        id=robot_id,
        use_degrees=True,                   # read/send degrees for joints; gripper in 0..100
        disable_torque_on_disconnect=True,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)

    def cleanup(*_):
        try: robot.disconnect()
        finally: sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # 1) Go to the starting pose first (from whatever random pose)
    start = poses[0]
    print("Going to starting pose (pose #1)...")
    ok = goto(robot, start, rate_hz, speed_deg_s, tol_deg, timeout_s=3.0)
    print(f"{'Reached' if ok else 'Timed out at'} pose #1 (timestamp {start.get('timestamp','n/a')}).")
    time.sleep(pause_s)

    # 2) Walk through the rest
    for idx, pose in enumerate(poses[1:], start=2):
        print(f"Going to pose #{idx} ...")
        ok = goto(robot, pose, rate_hz, speed_deg_s, tol_deg, timeout_s=3.0)
        print(f"{'Reached' if ok else 'Timed out at'} pose #{idx} (timestamp {pose.get('timestamp','n/a')}).")
        time.sleep(pause_s)

    cleanup()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default="follower_test.json", help="JSON with a 'poses' list")
    ap.add_argument("--port", default="", help="Override serial port (else use JSON->port or /dev/ttyACM0)")
    ap.add_argument("--id",   default="my_follower", help="Follower calibration id")
    ap.add_argument("--rate_hz", type=float, default=20.0)
    ap.add_argument("--speed_deg_s", type=float, default=30.0, help="Max joint speed (deg/s)")
    ap.add_argument("--tol_deg", type=float, default=1.5, help="Convergence tolerance (deg)")
    ap.add_argument("--pause_s", type=float, default=1.0, help="Pause between poses (s)")
    args = ap.parse_args()
    main(args.file, args.port, args.id, args.rate_hz, args.speed_deg_s, args.tol_deg, args.pause_s)
