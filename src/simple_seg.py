#!/usr/bin/env python3
import argparse
import os
from pathlib import Path
import cv2
import numpy as np

EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}

def imread_any(p: Path):
    img = cv2.imdecode(np.fromfile(str(p), dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Failed to read image: {p}")
    return img

def imwrite_any(p: Path, img):
    p.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imencode(p.suffix, img)[1]
    ok.tofile(str(p))

def clean_mask(mask, min_area=0, close_kernel=3, open_kernel=3):
    """mask: uint8 {0,255}"""
    mask_bin = (mask > 0).astype(np.uint8) * 255

    if close_kernel and close_kernel > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel, close_kernel))
        mask_bin = cv2.morphologyEx(mask_bin, cv2.MORPH_CLOSE, k)

    if open_kernel and open_kernel > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_kernel, open_kernel))
        mask_bin = cv2.morphologyEx(mask_bin, cv2.MORPH_OPEN, k)

    if min_area and min_area > 0:
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats((mask_bin > 0).astype(np.uint8), 8)
        keep = np.zeros_like(mask_bin)
        for lbl in range(1, num_labels):
            if stats[lbl, cv2.CC_STAT_AREA] >= min_area:
                keep[labels == lbl] = 255
        mask_bin = keep
    return mask_bin

def segment_otsu(img, blur=3):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if blur and blur > 1:
        gray = cv2.GaussianBlur(gray, (blur if blur % 2 else blur+1,)*2, 0)
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # Choose foreground polarity automatically (assume larger area is background)
    if (th > 0).mean() > 0.5:  # white covers majority
        th = cv2.bitwise_not(th)
    return th

def segment_adaptive(img, block_size=35, C=5):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if block_size % 2 == 0:
        block_size += 1
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                               cv2.THRESH_BINARY, block_size, C)
    if (th > 0).mean() > 0.5:
        th = cv2.bitwise_not(th)
    return th

def segment_kmeans(img, K=2, attempts=3):
    Z = img.reshape((-1,3)).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5)
    _, labels, centers = cv2.kmeans(Z, K, None, criteria, attempts, cv2.KMEANS_PP_CENTERS)
    labels = labels.reshape(img.shape[:2])
    # Pick cluster with lowest average intensity as foreground
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    fg_label = np.argmin([gray[labels==k].mean() for k in range(K)])
    mask = (labels == fg_label).astype(np.uint8)*255
    return mask

def segment_watershed(img, blur=3):
    """Rough instance-friendly FG mask via watershed over distance transform."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if blur and blur > 1:
        gray = cv2.GaussianBlur(gray, (blur if blur % 2 else blur+1,)*2, 0)
    # binary seed
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    th = 255 - th if (th > 0).mean() > 0.5 else th

    # sure background
    kernel = np.ones((3,3), np.uint8)
    sure_bg = cv2.dilate(th, kernel, iterations=3)

    # sure foreground via distance transform
    dist = cv2.distanceTransform(th, cv2.DIST_L2, 5)
    _, sure_fg = cv2.threshold(dist, 0.4*dist.max(), 255, 0)
    sure_fg = sure_fg.astype(np.uint8)

    unknown = cv2.subtract(sure_bg, sure_fg)
    # markers
    num_markers, markers = cv2.connectedComponents(sure_fg)
    markers = markers + 1
    markers[unknown==255] = 0

    markers = cv2.watershed(img, markers)
    mask = (markers > 1).astype(np.uint8) * 255
    return mask

def overlay(img, mask, alpha=0.45, color=(0,255,0)):
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)
    if mask.max() == 1:
        mask = mask*255
    color_mask = np.zeros_like(img)
    color_mask[:] = color
    mask3 = cv2.merge([mask, mask, mask])
    blended = img.copy()
    blended = cv2.addWeighted(blended, 1.0, (color_mask & mask3), alpha, 0)
    return blended

def process_image(img, method, args):
    if method == "otsu":
        m = segment_otsu(img, blur=args.blur)
    elif method == "adaptive":
        m = segment_adaptive(img, block_size=args.block_size, C=args.C)
    elif method == "kmeans":
        m = segment_kmeans(img, K=args.k)
    elif method == "watershed":
        m = segment_watershed(img, blur=args.blur)
    else:
        raise ValueError(f"Unknown method: {method}")

    m = clean_mask(m, min_area=args.min_area, close_kernel=args.close_kernel, open_kernel=args.open_kernel)
    return m

def main():
    parser = argparse.ArgumentParser(description="Lightweight CPU-based image segmentation (folder in -> masks out).")
    parser.add_argument("--input", required=True, type=str, help="Input folder with images")
    parser.add_argument("--output", required=True, type=str, help="Output folder for masks")
    parser.add_argument("--method", default="otsu", choices=["otsu","adaptive","kmeans","watershed"], help="Segmentation method")
    parser.add_argument("--min-area", type=int, default=200, help="Remove blobs smaller than this area (px)")
    parser.add_argument("--blur", type=int, default=3, help="Gaussian blur kernel size (odd, >=1) for otsu/watershed")
    parser.add_argument("--block-size", type=int, default=35, help="Adaptive threshold block size (odd)")
    parser.add_argument("--C", type=int, default=5, help="Adaptive threshold constant")
    parser.add_argument("--k", type=int, default=2, help="K-means K")
    parser.add_argument("--close-kernel", type=int, default=3, help="Morph close kernel size")
    parser.add_argument("--open-kernel", type=int, default=3, help="Morph open kernel size")
    parser.add_argument("--save-overlay", action="store_true", help="Also save color overlays")
    parser.add_argument("--overlay-alpha", type=float, default=0.45, help="Overlay opacity")
    args = parser.parse_args()

    in_dir = Path(args.input)
    out_dir = Path(args.output)
    out_masks = out_dir
    out_over = out_dir / "overlays"

    paths = [p for p in in_dir.rglob("*") if p.suffix.lower() in EXTS]
    if not paths:
        print("No images found.")
        return

    for p in paths:
        rel = p.relative_to(in_dir)
        mask_path = (out_masks / rel).with_suffix(".png")
        img = imread_any(p)
        mask = process_image(img, args.method, args)
        imwrite_any(mask_path, mask)
        if args.save_overlay:
            over = overlay(img, mask, alpha=args.overlay_alpha)
            over_path = (out_over / rel).with_suffix(".jpg")
            imwrite_any(over_path, over)

    print(f"Done. Masks -> {out_masks}")
    if args.save_overlay:
        print(f"Overlays -> {out_over}")

if __name__ == "__main__":
    main()
