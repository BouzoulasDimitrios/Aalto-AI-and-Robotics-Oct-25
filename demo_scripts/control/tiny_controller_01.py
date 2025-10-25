# so101_quicktest.py
from pathlib import Path
import time, draccus
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

PORT = "/dev/ttyACM0"  # you already confirmed this
CAL = Path("/home/dimitrios/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_follower.json")

# load calibration
with open(CAL) as f, draccus.config_type("json"):
    calib = draccus.load(dict[str, MotorCalibration], f)

# define bus + motors (IDs 1..6 are the SO-101 follower default)
bus = FeetechMotorsBus(
    port=PORT,
    motors={
        "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex":    Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex":    Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll":    Motor(5, "sts3215", MotorNormMode.DEGREES),
        "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    },
    calibration=calib,
)

bus.connect(True)
with bus.torque_disabled():
    bus.configure_motors()
    for m in bus.motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)

# read start pose
start = bus.sync_read("Present_Position")

# small, safe target (adjust if needed)
target = start | {"shoulder_lift": start["shoulder_lift"] + 10.0, "gripper": 10.0}

# go to target over ~2s
t0 = time.time()
while time.time() - t0 < 2.0:
    # simple lerp
    alpha = (time.time() - t0) / 2.0
    cmd = {k: (1-alpha)*start[k] + alpha*target[k] for k in target}
    bus.sync_write("Goal_Position", cmd, normalize=True)
    time.sleep(0.02)

# hold briefly, then return
time.sleep(1.0)
bus.sync_write("Goal_Position", start, normalize=True)
time.sleep(1.0)

bus.disable_torque()
print("Done.")
