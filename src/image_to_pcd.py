import os
import shutil
import subprocess
from pathlib import Path
import open3d as o3d

# ---- Config ----
IMAGE_DIR = Path("./images")
WORK_DIR  = Path("./work")
OUT_DIR   = Path("./outputs")
DB_PATH   = WORK_DIR / "colmap.db"
SPARSE_DIR = WORK_DIR / "sparse"        # mapper output
MODEL0_DIR = SPARSE_DIR / "0"           # first model
PLY_PATH   = OUT_DIR / "sparse_model.ply"
PCD_PATH   = OUT_DIR / "sparse_model.pcd"

# Downscale to reduce CPU time (longest side set here)
MAX_IMAGE_SIZE = 1920

def run(cmd):
    print(">>", " ".join(map(str, cmd)))
    subprocess.run(cmd, check=True)

def ensure_dirs():
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    SPARSE_DIR.mkdir(parents=True, exist_ok=True)

def main():
    ensure_dirs()
    if DB_PATH.exists():
        DB_PATH.unlink()

    # 1) OPTIONAL: image undistortion isn’t required up front; COLMAP handles it.
    # 2) Feature extraction (CPU SIFT)
    run([
        "colmap", "feature_extractor",
        "--database_path", str(DB_PATH),
        "--image_path", str(IMAGE_DIR),
        "--ImageReader.single_camera", "1",                 # small-obj capture: often same intrinsics
        "--ImageReader.camera_model", "PINHOLE",            # robust default
        "--SiftExtraction.use_gpu", "0",
        "--SiftExtraction.num_threads", "8",
        "--SiftExtraction.max_image_size", str(MAX_IMAGE_SIZE),
    ])

    # 3) Exhaustive matching (good for small N)
    run([
        "colmap", "exhaustive_matcher",
        "--database_path", str(DB_PATH),
        "--SiftMatching.use_gpu", "0",
        "--SiftMatching.num_threads", "8",
    ])

    # 4) Incremental mapping (sparse reconstruction)
    run([
        "colmap", "mapper",
        "--database_path", str(DB_PATH),
        "--image_path", str(IMAGE_DIR),
        "--output_path", str(SPARSE_DIR),
        "--Mapper.min_num_matches", "15",
        "--Mapper.tri_min_angle", "2.0",
    ])

    # Pick first model folder (0)
    if not MODEL0_DIR.exists():
        # Fallback: find any model folder
        candidates = sorted(SPARSE_DIR.glob("*"))
        if not candidates:
            raise RuntimeError("No sparse model produced. Check images/texture/overlap.")
        target = candidates[0]
    else:
        target = MODEL0_DIR

    # 5) Convert sparse model to PLY (contains colored 3D points)
    run([
        "colmap", "model_converter",
        "--input_path", str(target),
        "--output_path", str(PLY_PATH),
        "--output_type", "PLY"
    ])

    # 6) Load with Open3D, lightly clean, and also export .pcd
    pcd = o3d.io.read_point_cloud(str(PLY_PATH))
    # optional: statistical outlier removal for a cleaner cloud
    pcd, _ = o3d.geometry.PointCloud.remove_statistical_outlier(
        pcd, nb_neighbors=10, std_ratio=5.0
    )
    # optional: voxel downsample if you want fewer points
    # pcd = pcd.voxel_down_sample(voxel_size=0.002)  # ~2mm at your scene scale

    # Save cleaned PLY (overwrite) and PCD
    o3d.io.write_point_cloud(str(PLY_PATH), pcd, write_ascii=False, compressed=True)
    o3d.io.write_point_cloud(str(PCD_PATH), pcd, write_ascii=False, compressed=True)

    print(f"Done!\nPLY: {PLY_PATH}\nPCD: {PCD_PATH}")

if __name__ == "__main__":
    main()
