import cv2, time
from pathlib import Path

DEVICE = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera: {DEVICE}")

# Ask for MJPG @ 1280x720/30 (matches your v4l2-ctl output)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# Warm up & flush a few frames (some cams need this)
time.sleep(0.2)
for _ in range(5):
    cap.read()

ok, frame = cap.read()
cap.release()
if not ok:
    raise RuntimeError("Failed to grab frame (after MJPG request)")

out = Path("usb_snapshot.jpg")
cv2.imwrite(str(out), frame)
print(f"Saved {out.resolve()}")
