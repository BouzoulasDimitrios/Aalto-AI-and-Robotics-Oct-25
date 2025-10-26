#!/usr/bin/env python3
import signal
import subprocess
from pathlib import Path
import numpy as np
import open3d as o3d
import cv2

# =======================
# Config (CPU/GPU)
# =======================
IMAGE_DIR   = Path(f"PATH_TO_YOU_IMAGE_FOLDER")
WORK_DIR    = Path("./work")
OUT_DIR     = Path("./outputs")

DB_PATH     = WORK_DIR / "colmap.db"
SPARSE_DIR  = WORK_DIR / "sparse"        # mapper output
MODEL0_DIR  = SPARSE_DIR / "0"           # first model

# Sparse outputs
SPARSE_PLY  = OUT_DIR / "sparse_model.ply"
SPARSE_PCD  = OUT_DIR / "sparse_model.pcd"

# Dense (optional; on by default here)
DENSE_DIR   = WORK_DIR / "dense"
UNDIST_DIR  = DENSE_DIR / "undistorted"
DENSE_PLY   = OUT_DIR / "dense_fused.ply"
DENSE_PCD   = OUT_DIR / "dense_fused.pcd"

# Image size cap (reduce if you still hit OOM)
# Your images are 1020x740; keep a cap anyway for safety.
MAX_IMAGE_SIZE = 2000  # try 1280 on very low-RAM systems

# SIFT feature budget (lower => less RAM; higher => more points)
MAX_FEATURES    = 12000   # try 8000 if OOM; 20000+ if plenty RAM
# We will override PEAK_THRESHOLD below inside run_feature_extractor()
PEAK_THRESHOLD  = 0.006   # lower -> more features (more RAM). Was 0.01

# Threads
SIFT_THREADS    = "10"     # OK to increase on beefy CPUs
MATCH_THREADS_1 = "8"     # first attempt
MATCH_THREADS_2 = "4"     # fallback attempt(s)

# GPU flags — "0" to force CPU, "1" to allow GPU (as strings for COLMAP)
USE_GPU_SIFT    = "1"
USE_GPU_MATCH   = "1"

# Open3D point-cloud cleaning (keep False to retain more points)
APPLY_CLEANING        = False
CLEAN_nb_neighbors    = 8
CLEAN_std_ratio       = 20.0

# Dense reconstruction (PatchMatch) — uses more RAM/time
RUN_DENSE = True

# =======================
# Image preprocessing (helps planes)
# =======================
PREPROCESS = True
PREP_DIR = WORK_DIR / "images_preprocessed"
CLAHE_CLIP = 3.0          # 2.0–3.5 is a good range
CLAHE_TILE = 8            # tiles per dimension for CLAHE
UNSHARP_AMOUNT = 0.6      # 0.4–0.8 gentle, avoids halos
UNSHARP_RADIUS = 1.2      # in pixels (Gaussian sigma)
DITHER_STD = 0.0          # 0.0–0.5; add micro-noise if surfaces are very flat

# Point COLMAP to the effective directory (original or preprocessed)
EFFECTIVE_IMAGE_DIR = PREP_DIR if PREPROCESS else IMAGE_DIR

# =======================
# Ground removal + meshing settings
# =======================
# RANSAC plane fit to remove ground
GROUND_DISTANCE_THRESHOLD = 0.02  # adjust to your scene scale (meters)
GROUND_RANSAC_N           = 3
GROUND_RANSAC_ITERS       = 1000
# Require at least this fraction of points to be on the plane to accept it as "ground"
GROUND_MIN_INLIER_FRACTION = 0.05

# Mesh method: "poisson" (smooth, needs normals) or "alpha" (sharper, thin parts)
MESH_METHOD = "poisson"   # options: "poisson", "alpha"
POISSON_DEPTH = 10        # larger -> more detail & memory
POISSON_DENSITY_TRIM_PERCENT = 5.0  # remove lowest X% density verts (artifacts)
ALPHA = 0.05              # alpha-shape parameter (scene-scale dependent)

# Save mesh as ASCII STL (human-readable) or binary (smaller)
MESH_STL_ASCII = False

# =======================
# Densification (fill empty spots in PCD)
# =======================
DENSIFY = True
# Generate extra points by sampling the mesh and merging with the point cloud.
# Set target points ~ multiplier * original non-ground points
DENSIFY_MULTIPLIER = 2.0          # e.g., 2.0 => doubles point count (approx.)
DENSIFY_MIN_ADD    = 20000        # ensure at least this many new points (if feasible)
DENSIFY_VOXEL_MERGE = 0.003       # meters; 0.0 to disable merge (use scene scale!)
# If you prefer a fixed total, set DENSIFY_TARGET_ABS > 0 to override multiplier:
DENSIFY_TARGET_ABS = 0            # e.g., 2_000_000 for a big uniform cloud

# =======================
# Helpers
# =======================
def run(cmd):
    """Run a subprocess and turn SIGKILL into a friendlier message."""
    print(">>", " ".join(map(str, cmd)))
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        code = e.returncode
        if code in (-signal.SIGKILL, signal.SIGKILL, -9, 9):
            raise RuntimeError(
                "COLMAP process was killed (likely out of memory). "
                "Lower MAX_FEATURES and/or MAX_IMAGE_SIZE, reduce threads, "
                "or try the sequential matcher fallback."
            ) from e
        raise

def ensure_dirs():
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    SPARSE_DIR.mkdir(parents=True, exist_ok=True)
    DENSE_DIR.mkdir(parents=True, exist_ok=True)
    if PREPROCESS:
        PREP_DIR.mkdir(parents=True, exist_ok=True)


def pick_model_folder():
    if MODEL0_DIR.exists():
        return MODEL0_DIR
    candidates = sorted([p for p in SPARSE_DIR.iterdir() if p.is_dir()])
    if not candidates:
        raise RuntimeError("No sparse model produced. Check image overlap/texture.")
    return candidates[0]

# -----------------------
# Image preprocessing
# -----------------------
def preprocess_images(in_dir: Path, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=(CLAHE_TILE, CLAHE_TILE))
    for p in sorted(in_dir.glob("*")):
        if not p.suffix.lower() in [".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"]:
            continue
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            continue
        # CLAHE on L channel (LAB)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        L, A, B = cv2.split(lab)
        L = clahe.apply(L)
        lab = cv2.merge([L, A, B])
        img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        # Unsharp mask
        blur = cv2.GaussianBlur(img, (0, 0), UNSHARP_RADIUS)
        img = cv2.addWeighted(img, 1 + UNSHARP_AMOUNT, blur, -UNSHARP_AMOUNT, 0)
        # Optional tiny dithering to create micro-gradients on blank paint/glass
        if DITHER_STD > 0:
            noise = np.random.normal(0, DITHER_STD, img.shape).astype(np.float32)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        cv2.imwrite(str(out_dir / p.name), img, [cv2.IMWRITE_JPEG_QUALITY, 95])

# -----------------------
# Point cloud -> no-ground -> mesh -> files (+ densification)
# -----------------------

def segment_ground(pcd: o3d.geometry.PointCloud):
    """Detect and remove ground via RANSAC plane fitting. Returns (non_ground_pcd, plane_model, inlier_count)."""
    if len(pcd.points) == 0:
        print("[warn] Empty point cloud given to segment_ground.")
        return pcd, None, 0

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=GROUND_DISTANCE_THRESHOLD,
        ransac_n=GROUND_RANSAC_N,
        num_iterations=GROUND_RANSAC_ITERS
    )
    inlier_count = len(inliers)
    frac = inlier_count / max(1, len(pcd.points))
    print(f"Plane inliers: {inlier_count} ({frac:.1%} of points)")
    if frac < GROUND_MIN_INLIER_FRACTION:
        print("[info] Largest plane too small to be ground; skipping removal.")
        return pcd, None, 0

    a, b, c, d = plane_model
    print(f"Detected plane: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0 (removing inliers as ground)")
    non_ground_pcd = pcd.select_by_index(inliers, invert=True)
    return non_ground_pcd, plane_model, inlier_count


def estimate_normals_if_needed(pcd: o3d.geometry.PointCloud, k: int = 30):
    """Estimate and orient normals if missing (needed for Poisson)."""
    if not pcd.has_normals():
        print("Estimating normals...")
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k))
    # Oriented normals lead to better Poisson results
    pcd.orient_normals_consistent_tangent_plane(50)
    return pcd


def reconstruct_mesh_from_pcd(pcd: o3d.geometry.PointCloud):
    """Create TriangleMesh from point cloud using selected method."""
    if MESH_METHOD.lower() == "alpha":
        print(f"Meshing via Alpha Shape (alpha={ALPHA})...")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, ALPHA)
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()
        mesh.remove_non_manifold_edges()
        mesh.compute_vertex_normals()
        return mesh

    # Default: Poisson
    print(f"Meshing via Poisson (depth={POISSON_DEPTH})...")
    pcd = estimate_normals_if_needed(pcd)
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=POISSON_DEPTH
    )
    densities = np.asarray(densities)
    keep_threshold = np.percentile(densities, POISSON_DENSITY_TRIM_PERCENT)
    keep_idx = np.where(densities > keep_threshold)[0]
    mesh = mesh.select_by_index(keep_idx)
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    return mesh


def densify_from_mesh(original_pcd: o3d.geometry.PointCloud,
                      mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.PointCloud:
    """
    Create extra points by sampling the mesh and merge with the original non-ground cloud.
    Returns a densified point cloud.
    """
    n0 = len(original_pcd.points)
    if n0 == 0:
        print("[warn] No points to densify; returning original.")
        return original_pcd

    if DENSIFY_TARGET_ABS > 0:
        target_total = DENSIFY_TARGET_ABS
    else:
        target_total = max(int(n0 * DENSIFY_MULTIPLIER), n0 + DENSIFY_MIN_ADD)

    add_count = max(0, target_total - n0)
    if add_count == 0:
        print("[info] Densification not needed based on target settings.")
        return original_pcd

    # Prefer Poisson-disk sampling for good coverage and spacing
    print(f"Sampling {add_count} new points from mesh (Poisson-disk)...")
    sampled = mesh.sample_points_poisson_disk(
        number_of_points=add_count,
        init_factor=5  # oversampling factor to improve coverage
    )

    # If mesh had colors, they propagate; otherwise keep original colors
    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(
        np.vstack([np.asarray(original_pcd.points), np.asarray(sampled.points)])
    )
    if original_pcd.has_colors() or sampled.has_colors():
        # Combine colors (fallback to gray if missing)
        def colors_or_gray(pcd, count):
            if pcd.has_colors():
                return np.asarray(pcd.colors)
            return np.tile(np.array([[0.5, 0.5, 0.5]]), (count, 1))
        c0 = colors_or_gray(original_pcd, len(original_pcd.points))
        c1 = colors_or_gray(sampled, len(sampled.points))
        merged.colors = o3d.utility.Vector3dVector(np.vstack([c0, c1]))

    # Optional voxel merge to eliminate duplicates and equalize density
    if DENSIFY_VOXEL_MERGE and DENSIFY_VOXEL_MERGE > 0:
        print(f"Voxel merging at {DENSIFY_VOXEL_MERGE} m ...")
        merged = merged.voxel_down_sample(voxel_size=DENSIFY_VOXEL_MERGE)

    print(f"Densified cloud: {len(merged.points)} points (was {n0})")
    return merged


def save_pointcloud_mesh_and_stl(input_ply_path: Path, out_ply_path: Path, out_pcd_path: Path):
    """Load PLY -> (optional clean) -> remove ground -> write PLY/PCD -> mesh -> STL -> densify -> write upsampled PLY/PCD."""
    pcd = o3d.io.read_point_cloud(str(input_ply_path))
    print(f"Loaded {input_ply_path} with {len(pcd.points)} points")

    # Optional cleaning
    if APPLY_CLEANING and len(pcd.points) > 0:
        print("Applying statistical outlier removal...")
        pcd, _ = o3d.geometry.PointCloud.remove_statistical_outlier(
            pcd, nb_neighbors=CLEAN_nb_neighbors, std_ratio=CLEAN_std_ratio
        )
        print(f"After cleaning: {len(pcd.points)} points")

    # --- Ground removal
    non_ground_pcd, plane_model, inlier_count = segment_ground(pcd)
    print(f"After ground removal: {len(non_ground_pcd.points)} points remain")

    # Save the non-ground point cloud (PLY + PCD)
    o3d.io.write_point_cloud(str(out_ply_path), non_ground_pcd, write_ascii=False, compressed=True)
    o3d.io.write_point_cloud(str(out_pcd_path), non_ground_pcd, write_ascii=False, compressed=True)

    # --- Mesh reconstruction
    if len(non_ground_pcd.points) == 0:
        print("[warn] No points after ground removal; skipping meshing/densification.")
        return

    print("Reconstructing surface mesh...")
    mesh = reconstruct_mesh_from_pcd(non_ground_pcd)
    print(f"Mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

    # Save mesh as STL (same stem as out_ply_path)
    mesh_stl_path = out_ply_path.with_suffix(".stl")
    o3d.io.write_triangle_mesh(str(mesh_stl_path), mesh, write_ascii=MESH_STL_ASCII)
    print(f"Saved:\n  PLY: {out_ply_path}\n  PCD: {out_pcd_path}\n  STL: {mesh_stl_path}")

    # --- Densification (new)
    if DENSIFY:
        dense_pc = densify_from_mesh(non_ground_pcd, mesh)
        up_ply = out_ply_path.parent / f"{out_ply_path.stem}_upsampled.ply"
        up_pcd = out_pcd_path.parent / f"{out_pcd_path.stem}_upsampled.pcd"
        o3d.io.write_point_cloud(str(up_ply), dense_pc, write_ascii=False, compressed=True)
        o3d.io.write_point_cloud(str(up_pcd), dense_pc, write_ascii=False, compressed=True)
        print(f"Extra (densified):\n  PLY: {up_ply}\n  PCD: {up_pcd}")

# =======================
# COLMAP steps
# =======================

def run_feature_extractor():
    run([
        "colmap", "feature_extractor",
        "--database_path", str(DB_PATH),
        "--image_path", str(EFFECTIVE_IMAGE_DIR),
        "--ImageReader.single_camera", "1",
        "--ImageReader.camera_model", "PINHOLE",
        "--SiftExtraction.use_gpu", USE_GPU_SIFT,
        "--SiftExtraction.num_threads", SIFT_THREADS,
        "--SiftExtraction.max_image_size", str(MAX_IMAGE_SIZE),
        "--SiftExtraction.max_num_features", str(MAX_FEATURES),
        # More permissive on flats
        "--SiftExtraction.peak_threshold", str(PEAK_THRESHOLD),  # 0.006 recommended; go to 0.004 if needed
        # Multi-scale & more intra-octave samples
        "--SiftExtraction.first_octave", "-1",
        "--SiftExtraction.octave_resolution", "4",
        # Slightly relax the edge filter
        "--SiftExtraction.edge_threshold", "12",
        # Robustness on slanted/planar surfaces
        "--SiftExtraction.estimate_affine_shape", "1",
        "--SiftExtraction.domain_size_pooling", "1",
    ])


def run_matching():
    """
    Try exhaustive matching first; if OOM or failure, back off threads and disable guided matching.
    Finally fall back to sequential matching (much lighter), assuming images are in capture order.
    """
    # Attempt 1: exhaustive + guided matching, medium threads
    try:
        run([
            "colmap", "exhaustive_matcher",
            "--database_path", str(DB_PATH),
            "--SiftMatching.use_gpu", USE_GPU_MATCH,
            "--SiftMatching.num_threads", MATCH_THREADS_1,
            "--SiftMatching.guided_matching", "1"
        ])
        return
    except Exception as e:
        print(f"[warn] exhaustive_matcher (guided, threads={MATCH_THREADS_1}) failed: {e}")

    # Attempt 2: exhaustive, fewer threads, no guided matching
    try:
        run([
            "colmap", "exhaustive_matcher",
            "--database_path", str(DB_PATH),
            "--SiftMatching.use_gpu", USE_GPU_MATCH,
            "--SiftMatching.num_threads", MATCH_THREADS_2,
            "--SiftMatching.guided_matching", "0"
        ])
        return
    except Exception as e:
        print(f"[warn] exhaustive_matcher (no guided, threads={MATCH_THREADS_2}) failed: {e}")

    # Attempt 3: sequential matcher (lightweight)
    try:
        run([
            "colmap", "sequential_matcher",
            "--database_path", str(DB_PATH),
            "--SiftMatching.use_gpu", USE_GPU_MATCH,
            "--SiftMatching.num_threads", MATCH_THREADS_2,
            "--SequentialMatching.overlap", "5",
            "--SequentialMatching.vocab_tree", ""
        ])
        print("[info] Used sequential_matcher fallback.")
        return
    except Exception as e:
        print(f"[warn] sequential_matcher failed: {e}")
        raise RuntimeError(
            "All matching attempts failed. Try lowering MAX_FEATURES, lowering MAX_IMAGE_SIZE, "
            "or increasing system memory/swap."
        )


def run_mapper():
    run([
        "colmap", "mapper",
        "--database_path", str(DB_PATH),
        "--image_path", str(EFFECTIVE_IMAGE_DIR),
        "--output_path", str(SPARSE_DIR),
        # Relaxed to triangulate more points (can add a bit of noise)
        "--Mapper.min_num_matches", "8",          # default 15
        "--Mapper.tri_min_angle", "1.0",          # default 2.0
        "--Mapper.filter_max_reproj_error", "4.0" # default 4.0 (keep at 4 or 3)
    ])


def export_sparse_model(model_dir: Path):
    run([
        "colmap", "model_converter",
        "--input_path", str(model_dir),
        "--output_path", str(SPARSE_PLY),
        "--output_type", "PLY"
    ])
    # Save PLY/PCD (no-ground) + STL + densified PLY/PCD
    save_pointcloud_mesh_and_stl(SPARSE_PLY, SPARSE_PLY, SPARSE_PCD)


def run_dense_pipeline(model_dir: Path):
    # 1) Undistort into dense workspace
    run([
        "colmap", "image_undistorter",
        "--image_path", str(EFFECTIVE_IMAGE_DIR),
        "--input_path", str(model_dir),
        "--output_path", str(UNDIST_DIR),
        "--output_type", "COLMAP"
    ])

    # 2) PatchMatch stereo — wider support, more samples/iters, gentler filtering
    run([
        "colmap", "patch_match_stereo",
        "--workspace_path", str(UNDIST_DIR),
        # Geometry consistency helps reject impossible matches
        "--PatchMatchStereo.geom_consistency", "1",
        # Make the photometric window a bit larger for homogeneous areas
        "--PatchMatchStereo.window_radius", "7",      # default 5
        # Explore more hypotheses (helps smooth planes converge)
        "--PatchMatchStereo.num_samples", "20",       # default ~15
        "--PatchMatchStereo.num_iterations", "8",     # default 5-6
        # If lighting differs, loosen NCC filter slightly
        "--PatchMatchStereo.filter_min_ncc", "0.05",  # default 0.1
        # Keep small triangles; planes are often fronto-parallel
        "--PatchMatchStereo.min_triangulation_angle", "1.5"  # default 2.0
    ])

    # 3) Fuse to dense cloud — allow thin/flat regions to pass
    run([
        "colmap", "stereo_fusion",
        "--workspace_path", str(UNDIST_DIR),
        "--input_type", "geometric",
        # Minimum consistent pixels per depth; 2–3 works for planes
        "--StereoFusion.min_num_pixels", "3",         # default 5
        # Keep points with slightly weaker support (good for flats)
        "--StereoFusion.max_reproj_error", "3.0",     # default 2.0
        "--StereoFusion.max_depth_error", "0.02",     # scene-scale dependent
        "--output_path", str(DENSE_PLY)
    ])

    # Save PLY/PCD (no-ground) + STL + densified PLY/PCD
    save_pointcloud_mesh_and_stl(DENSE_PLY, DENSE_PLY, DENSE_PCD)

# =======================
# Main
# =======================

def main():
    ensure_dirs()
    if PREPROCESS:
        print("Preprocessing images for better planar features...")
        preprocess_images(IMAGE_DIR, PREP_DIR)
    if DB_PATH.exists():
        DB_PATH.unlink()

    # 1) Feature extraction
    run_feature_extractor()

    # 2) Matching with OOM-aware fallbacks
    run_matching()

    # 3) Sparse reconstruction
    run_mapper()

    # 4) Export sparse PLY/PCD + STL (+ densified)
    model_dir = pick_model_folder()
    export_sparse_model(model_dir)

    # 5) Dense cloud (optional)
    if RUN_DENSE:
        run_dense_pipeline(model_dir)

    print("\nDone!")
    if SPARSE_PLY.exists():
        print(
            f"Sparse:\n  PLY: {SPARSE_PLY}\n  PCD: {SPARSE_PCD}\n  STL: {SPARSE_PLY.with_suffix('.stl')}\n  PLY (upsampled): {SPARSE_PLY.parent / (SPARSE_PLY.stem + '_upsampled.ply')}\n  PCD (upsampled): {SPARSE_PCD.parent / (SPARSE_PCD.stem + '_upsampled.pcd')}"
        )
    if RUN_DENSE and DENSE_PLY.exists():
        print(
            f"Dense :\n  PLY: {DENSE_PLY}\n  PCD: {DENSE_PCD}\n  STL: {DENSE_PLY.with_suffix('.stl')}\n  PLY (upsampled): {DENSE_PLY.parent / (DENSE_PLY.stem + '_upsampled.ply')}\n  PCD (upsampled): {DENSE_PCD.parent / (DENSE_PCD.stem + '_upsampled.pcd')}"
        )

if __name__ == "__main__":
    main()

