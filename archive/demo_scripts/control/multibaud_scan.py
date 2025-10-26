# deep_scan_bus.py
import glob

from lerobot.motors.feetech import FeetechMotorsBus

ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/serial/by-id/*")))
if not ports:
    ports = ["/dev/ttyACM0"]

bauds = [1_000_000, 500_000, 250_000, 115200]

def try_scan(port, baud):
    try:
        bus = FeetechMotorsBus(port=port, motors={}, baudrate=baud)  # if your version doesn’t accept baudrate, it will raise TypeError
    except TypeError:
        bus = FeetechMotorsBus(port=port, motors={})  # fallback; uses default 1_000_000
    try:
        bus.connect(handshake=False)
        found = {}
        for mid in range(1, 21):
            try:
                model = bus.read("Model_Number", mid)
                val = int(model[0]) if hasattr(model, "__len__") else int(model)
                found[mid] = val
            except Exception:
                pass
        bus.disconnect()
        return found
    except Exception:
        return {}

for p in ports:
    for b in bauds:
        found = try_scan(p, b)
        print(f"Port {p} @ {b} -> {found}")
