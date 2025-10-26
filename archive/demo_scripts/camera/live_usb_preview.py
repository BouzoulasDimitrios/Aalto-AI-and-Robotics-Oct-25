#!/usr/bin/env python3
import cv2, time
from pathlib import Path
from datetime import datetime

DEVICE = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera: {DEVICE}")

# Ask for MJPG @ 1280x720/30 (same as your script)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

# Warm up & flush a few frames
time.sleep(0.2)
for _ in range(5):
    cap.read()

# Report negotiated settings
w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
fourcc = "".join([chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4)])
print(f"Opened {DEVICE} with {w}x{h} @ ~{fps:.1f} FPS, FOURCC={fourcc}")
print("Controls: q/ESC = quit, s = save snapshot")

cv2.namedWindow("USB Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("USB Camera", w, h)

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("⚠️  Failed to grab frame — stopping.")
            break

        # (Optional) overlay tiny help text
        cv2.putText(frame, "q/ESC: quit   s: snapshot",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

        cv2.imshow("USB Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('s'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            out = Path(f"usb_snapshot_{ts}.jpg")
            cv2.imwrite(str(out), frame)
            print(f"Saved {out.resolve()}")
finally:
    cap.release()
    cv2.destroyAllWindows()
