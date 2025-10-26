# so101_joint_control.py
import time, sys, signal, argparse
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]


movements = {
    1: {"shoulder_pan": 0,  "shoulder_lift": -20, "elbow_flex": 40, "wrist_flex": -20, "wrist_roll": 0,  "gripper": 60},
    2: {"shoulder_pan": 15, "shoulder_lift": -35, "elbow_flex": 60, "wrist_flex": -25, "wrist_roll": 0,  "gripper": 60},
    3: {"shoulder_pan": 20, "shoulder_lift": -35, "elbow_flex": 60, "wrist_flex": -25, "wrist_roll": 0,  "gripper": 60},
    4: {"shoulder_pan": 25, "shoulder_lift": -35, "elbow_flex": 60, "wrist_flex": -25, "wrist_roll": 0,  "gripper": 60},
    5: {"shoulder_pan": 30, "shoulder_lift": -35, "elbow_flex": 60, "wrist_flex": -25, "wrist_roll": 0,  "gripper": 60},
    6: {"shoulder_pan": 35, "shoulder_lift": -35, "elbow_flex": 60, "wrist_flex": -25, "wrist_roll": 0,  "gripper": 60},
    7: {"shoulder_pan": 40, "shoulder_lift": -35, "elbow_flex": 60, "wrist_flex": -25, "wrist_roll": 0,  "gripper": 60}
    
}

at_rest = {"shoulder_pan": 0, "shoulder_lift": -5, "elbow_flex": 10, "wrist_flex": -5, "wrist_roll": 0,  "gripper": 60}

def main(port, robot_id, rate_hz):
    cfg = SO101FollowerConfig(
        port=port,
        id=robot_id,
        use_degrees=True,           # degrees for joints; gripper in 0..100
        disable_torque_on_disconnect=True,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)

    def cleanup(*_):
        try: robot.disconnect()
        finally: sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    def hold(goal_deg: dict, seconds=2.0):
        # Re-send the same action at a fixed rate so motors track smoothly.
        action = {f"{j}.pos": float(goal_deg[j]) for j in goal_deg}
        steps = max(1, int(seconds * rate_hz))
        for _ in range(steps):
            robot.send_action(action)
            time.sleep(1.0 / rate_hz)

    # --- Example poses (tweak as you like) ---
    # hold(neutral, 3.0)
    # hold(reach,   3.0)
    # hold(grasp,   1.0)
    # hold(open_,   1.0)
    for i in movements:
        hold(movements[i])
    hold(at_rest, 1)
    cleanup()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id",   default="my_follower")  # your calibration id
    ap.add_argument("--rate_hz", type=float, default=20.0)
    args = ap.parse_args()
    main(args.port, args.id, args.rate_hz)
