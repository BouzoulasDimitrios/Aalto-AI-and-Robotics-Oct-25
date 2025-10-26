import argparse
import os
import random
from pathlib import Path

import cv2
import numpy as np
import torch
from tqdm import tqdm
import matplotlib.pyplot as plt

from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

# ---------------------------
# Utilities
# ---------------------------

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}

def list_images(folder: Path):
    return sorted([p for p in folder.rglob("*") if p.suffix.lower() in IMG_EXTS])

def colorize_masks(image_bgr: np.ndarray, masks: list, alpha: float = 0.55):
    """
    Overlay random colors and draw contours for each mask.
    Returns an RGB visualization and a 16-bit instance map.
    """
    h, w = image_bgr.shape[:2]
    overlay = np.zeros((h, w, 3), dtype=np.uint8)
    edges = np.zeros((h, w), dtype=np.uint8)
    instance_map = np.zeros((h, w), dtype=np.uint16)  # up to 65535 instances

    # Sort masks by area descending for nicer layering
    masks_sorted = sorted(masks, key=lambda m: m.get("area", 0), reverse=True)

    for idx, m in enumerate(masks_sorted, start=1):
        seg = m["segmentation"].astype(np.uint8)  # HxW bool
        if seg.sum() == 0:
            continue

        # Random color (but deterministic per run for reproducibility could be added)
        color = [random.randint(0, 255) for _ in range(3)]

        # Paint overlay
        for c in range(3):
            overlay[..., c] = np.where(seg == 1, color[c], overlay[..., c])

        # Draw contours
        cnts, _ = cv2.findContours(seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(edges, cnts, -1, 255, thickness=1)

        # Instance IDs (idx)
        instance_map[seg == 1] = idx

    # Alpha blend overlay with original
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    vis = cv2.addWeighted(image_rgb, 1 - alpha, overlay, alpha, 0)

    # Put white contours on top
    vis[edges == 255] = [255, 255, 255]

    return vis, instance_map

def save_visualization(out_path: Path, vis_rgb: np.ndarray):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Save with matplotlib to avoid color issues and allow tight bbox
    plt.figure(figsize=(10, 10))
    plt.imshow(vis_rgb)
    plt.axis("off")
    plt.tight_layout(pad=0)
    plt.savefig(str(out_path), dpi=200, bbox_inches="tight", pad_inches=0)
    plt.close()

def save_instance_png(out_path: Path, instance_map: np.ndarray):
    """
    Save a 16-bit PNG where each instance has a unique integer ID.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Ensure 16-bit
    instance_map = instance_map.astype(np.uint16)
    cv2.imwrite(str(out_path), instance_map)

# ---------------------------
# Main
# ---------------------------

def main():
    parser = argparse.ArgumentParser(description="Batch-segment images with SAM and save visualizations.")
    parser.add_argument("--input_dir", required=True, type=str, help="Folder with input images.")
    parser.add_argument("--output_dir", required=True, type=str, help="Folder to write results.")
    parser.add_argument("--checkpoint", required=True, type=str, help="Path to SAM checkpoint .pth")
    parser.add_argument("--model_type", default="vit_h", choices=["vit_h", "vit_l", "vit_b"], help="SAM model variant.")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu", help="Device: 'cuda' or 'cpu'")
    # Mask generator knobs (tune for your data)
    parser.add_argument("--points_per_side", type=int, default=32, help="Grid density; higher = more masks.")
    parser.add_argument("--pred_iou_thresh", type=float, default=0.86)
    parser.add_argument("--stability_score_thresh", type=float, default=0.92)
    parser.add_argument("--crop_n_layers", type=int, default=1)
    parser.add_argument("--crop_n_points_downscale_factor", type=int, default=2)
    parser.add_argument("--min_mask_region_area", type=int, default=0, help="Filter small regions in postprocessing (px).")
    parser.add_argument("--alpha", type=float, default=0.55, help="Overlay transparency (0..1).")
    parser.add_argument("--max_size", type=int, default=0, help="Optional longest side resize before SAM; 0 = no resize")
    args = parser.parse_args()

    in_dir = Path(args.input_dir)
    out_dir = Path(args.output_dir)
    vis_dir = out_dir / "visualizations"
    inst_dir = out_dir / "instance_masks"

    if not in_dir.exists():
        raise FileNotFoundError(f"Input folder not found: {in_dir}")

    # Load SAM
    sam = sam_model_registry[args.model_type](checkpoint=args.checkpoint)
    sam.to(device=args.device)
    mask_generator = SamAutomaticMaskGenerator(
        sam,
        points_per_side=args.points_per_side,
        pred_iou_thresh=args.pred_iou_thresh,
        stability_score_thresh=args.stability_score_thresh,
        crop_n_layers=args.crop_n_layers,
        crop_n_points_downscale_factor=args.crop_n_points_downscale_factor,
        min_mask_region_area=args.min_mask_region_area,
    )

    images = list_images(in_dir)
    if not images:
        print(f"No images found in {in_dir}. Supported: {sorted(IMG_EXTS)}")
        return

    for img_path in tqdm(images, desc="Segmenting"):
        try:
            img_bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
            if img_bgr is None:
                print(f"Warning: failed to read {img_path}")
                continue

            # Optional pre-resize for very large images to speed up
            if args.max_size and max(img_bgr.shape[:2]) > args.max_size:
                h, w = img_bgr.shape[:2]
                scale = args.max_size / float(max(h, w))
                new_wh = (int(w * scale), int(h * scale))
                img_bgr = cv2.resize(img_bgr, new_wh, interpolation=cv2.INTER_AREA)

            masks = mask_generator.generate(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
            vis_rgb, instance_map = colorize_masks(img_bgr, masks, alpha=args.alpha)

            rel = img_path.relative_to(in_dir)
            base = rel.with_suffix("")  # keep subfolders
            save_visualization(vis_dir / f"{base}.png", vis_rgb)
            save_instance_png(inst_dir / f"{base}.png", instance_map)

        except Exception as e:
            print(f"Error on {img_path}: {e}")

    print(f"\nDone! Visualizations -> {vis_dir}\nInstance masks (16-bit PNG) -> {inst_dir}")

if __name__ == "__main__":
    main()
