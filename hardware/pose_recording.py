# record_arm_positions.py
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from pathlib import Path
import draccus
import time
import json
from datetime import datetime

# ==== CONFIG ====
PORT_ID = "/dev/ttyACM0"        # <-- set your port if different
ROBOT_NAME = "follower-1"       # unused except for file naming
CALIB_PATH = Path("/home/dimitrios/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_follower.json")
SAVE_DIR = Path("./captures")   # where the JSON will be saved
# ================

def load_calibration() -> dict[str, MotorCalibration]:
    with open(CALIB_PATH) as f, draccus.config_type("json"):
        return draccus.load(dict[str, MotorCalibration], f)

def setup_motors(calibration, port_id: str) -> FeetechMotorsBus:
    norm_mode_body = MotorNormMode.DEGREES
    bus = FeetechMotorsBus(
        port=port_id,
        motors={
            "shoulder_pan":  Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex":    Motor(3, "sts3215", norm_mode_body),
            "wrist_flex":    Motor(4, "sts3215", norm_mode_body),
            "wrist_roll":    Motor(5, "sts3215", norm_mode_body),
            "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
        calibration=calibration,
    )
    bus.connect(True)

    # Configure once with torque temporarily off to avoid twitch
    with bus.torque_disabled():
        bus.configure_motors()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            bus.write("P_Coefficient", motor, 16)  # calmer
            bus.write("I_Coefficient", motor, 0)
            bus.write("D_Coefficient", motor, 32)
    return bus

def snapshot(bus: FeetechMotorsBus) -> dict:
    """Read the current joint states into a clean dict with a timestamp."""
    present = bus.sync_read("Present_Position")  # typically returns a dict
    # ensure we only serialize plain numbers
    clean = {k: float(v) for k, v in present.items()}
    clean["timestamp"] = datetime.now().isoformat(timespec="milliseconds")
    return clean

def main():
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    calibration = load_calibration()
    bus = setup_motors(calibration, PORT_ID)

    # Free-move the arm
    bus.disable_torque()
    print("\nTorque disabled — you can move the arm by hand.")
    print("Press ENTER to record a pose, type 'q' then ENTER to finish.\n")

    recordings: list[dict] = []
    count = 0

    try:
        while True:
            user = input("[ENTER]=record  |  q=quit > ").strip().lower()
            if user == "q":
                break

            pose = snapshot(bus)
            recordings.append(pose)
            count += 1

            # Pretty, short feedback
            joints_preview = {k: round(v, 2) for k, v in pose.items() if k != "timestamp"}
            print(f"  ✓ Pose {count} @ {pose['timestamp']}")
            print(f"    {joints_preview}")

    except KeyboardInterrupt:
        print("\nInterrupted — saving what we have…")

    finally:
        # Always keep torque disabled for safety; just disconnect the bus.
        try:
            bus.disconnect()
        except Exception:
            pass

    # Save JSON if we captured anything
    if recordings:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = SAVE_DIR / f"{ROBOT_NAME}_poses_{stamp}.json"
        with open(out_path, "w") as f:
            json.dump(
                {
                    "robot": ROBOT_NAME,
                    "port": PORT_ID,
                    "created_at": datetime.now().isoformat(timespec="seconds"),
                    "count": len(recordings),
                    "poses": recordings,
                },
                f,
                indent=2
            )
        print(f"\nSaved {len(recordings)} pose(s) to: {out_path.resolve()}")
    else:
        print("\nNo poses recorded. Nothing saved.")

if __name__ == "__main__":
    main()