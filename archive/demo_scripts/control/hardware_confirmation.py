# scan_bus.py
import argparse

# New-style imports (no .configs)
from lerobot.motors.feetech import FeetechMotorsBus

def main(port: str):
    # Empty 'motors' is fine for a raw scan; we bypass the handshake check.
    bus = FeetechMotorsBus(port=port, motors={})
    bus.connect(handshake=False)  # keep it simple: open port, don't assert expected motors
    found = {}
    for mid in range(1, 21):
        try:
            model = bus.read("Model_Number", mid)
            val = int(model[0]) if hasattr(model, "__len__") else int(model)
            found[mid] = val
        except Exception:
            pass
    bus.disconnect()
    print("Found motors (id: model_number):", found)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    args = ap.parse_args()
    main(args.port)
