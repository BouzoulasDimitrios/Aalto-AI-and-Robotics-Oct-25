#!/usr/bin/env python3
"""
Extract 80 evenly spaced frames from an MP4 and save them as images.

Usage:
  python extract_80_frames.py /path/to/video.mp4 -o out_frames -f jpg
"""

import cv2
import os
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video", help="Path to input .mp4")
    ap.add_argument("-o", "--out", default="frames_out", help="Output folder")
    ap.add_argument("-k", "--count", type=int, default=60, help="How many frames to save (default: 80)")
    ap.add_argument("-f", "--format", choices=["jpg", "png"], default="jpg", help="Image format")
    ap.add_argument("--quality", type=int, default=95, help="JPEG quality (1–100) / PNG compression (0–9)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Could not open video: {args.video}")

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        raise SystemExit("Could not read frame count from video.")

    k = min(args.count, total)  # can't extract more unique frames than exist

    # Compute k unique, increasing frame indices across the video
    if k == 1:
        frame_indices = [0]
    else:
        step = (total - 1) / (k - 1)
        frame_indices = []
        prev = -1
        for i in range(k):
            idx = int(round(i * step))
            if idx <= prev:
                idx = prev + 1
            if idx >= total:
                idx = total - 1
            frame_indices.append(idx)
            prev = idx

    # Save frames
    saved = 0
    for n, idx in enumerate(frame_indices, start=1):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok or frame is None:
            print(f"Warning: unable to read frame {idx}")
            continue

        fname = os.path.join(args.out, f"frame_{n:04d}.{args.format}")
        if args.format == "jpg":
            cv2.imwrite(fname, frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
        else:  # png
            comp = max(0, min(9, args.quality // 10))  # map ~0–100 → 0–9
            cv2.imwrite(fname, frame, [int(cv2.IMWRITE_PNG_COMPRESSION), comp])
        saved += 1

    cap.release()

    print(f"Requested: {args.count} | Available frames: {total} | Saved: {saved} → {os.path.abspath(args.out)}")
    if saved < args.count:
        print("Note: Video didn't have enough unique frames to save all requested images.")

if __name__ == "__main__":
    main()

