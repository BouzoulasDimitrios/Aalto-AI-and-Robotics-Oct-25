#!/usr/bin/env python3
import argparse, math, sys
import numpy as np
import cv2
import open3d as o3d

def parse_args():
    p = argparse.ArgumentParser(description="CPU-only video SfM to PCD")
    p.add_argument("--video", required=True, type=str, help="Input video path")
    p.add_argument("--output", required=True, type=str, help="Output .pcd path")
    p.add_argument("--stride", type=int, default=5, help="Process every Nth frame")
    p.add_argument("--max-frames", type=int, default=300, help="Max frames to use")
    p.add_argument("--focal", type=float, default=0.0, help="fx=fy focal length; 0=auto")
    p.add_argument("--K", type=str, default="", help="fx,fy,cx,cy (overrides --focal)")
    p.add_argument("--min-track", type=int, default=100, help="Min matches to accept pose")
    p.add_argument("--use-orb", action="store_true", help="Use ORB instead of SIFT")
    p.add_argument("--no-denoise", action="store_true", help="Skip statistical outlier removal")
    return p.parse_args()

def build_camera_matrix(w, h, focal=0.0, K_override=""):
    if K_override:
        fx, fy, cx, cy = map(float, K_override.split(","))
    else:
        if focal <= 0:
            # A decent guess: ~0.9 * max dimension
            f = 0.9 * max(w, h)
        else:
            f = focal
        fx, fy, cx, cy = f, f, w * 0.5, h * 0.5
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    return K

def get_detector(use_orb=False):
    if not use_orb:
        try:
            sift = cv2.SIFT_create(nfeatures=6000)
            return sift, "SIFT", cv2.NORM_L2
        except Exception:
            pass
    orb = cv2.ORB_create(nfeatures=8000, fastThreshold=5, scaleFactor=1.2, WTA_K=2)
    return orb, "ORB", cv2.NORM_HAMMING

def sample_frames(video_path, stride=5, max_frames=300):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")
    frames = []
    idx = 0
    used = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % stride == 0:
            frames.append(frame)
            used += 1
            if used >= max_frames:
                break
        idx += 1
    cap.release()
    if len(frames) < 2:
        raise RuntimeError("Not enough frames after sampling. Try smaller --stride or a longer video.")
    return frames

def detect_and_describe(detector, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kps, des = detector.detectAndCompute(gray, None)
    if des is None or len(kps) < 10:
        return [], None, gray
    return kps, des, gray

def match_descriptors(des1, des2, norm_type, ratio=0.75):
    bf = cv2.BFMatcher(norm_type, crossCheck=False)
    knn = bf.knnMatch(des1, des2, k=2)
    good = []
    for m, n in knn:
        if m.distance < ratio * n.distance:
            good.append(m)
    return good

def normalize_points(pts, K):
    Kinv = np.linalg.inv(K)
    pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])
    nrm = (Kinv @ pts_h.T).T[:, :3]
    return nrm[:, :2] / nrm[:, 2:3]

def triangulate_points(P1, P2, pts1, pts2):
    # pts1/pts2 Nx2 in pixel; convert to hom. for cv2.triangulatePoints
    pts1_h = cv2.convertPointsToHomogeneous(pts1).reshape(-1, 3).T
    pts2_h = cv2.convertPointsToHomogeneous(pts2).reshape(-1, 3).T
    X_h = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)  # expects 2xN (non-hom) actually
    X = (X_h[:3] / X_h[3]).T
    return X

def make_proj_matrix(K, R, t):
    Rt = np.hstack([R, t.reshape(3,1)])
    return K @ Rt

def pose_from_essential(kps1, kps2, K):
    pts1 = np.array([kp.pt for kp in kps1], dtype=np.float64)
    pts2 = np.array([kp.pt for kp in kps2], dtype=np.float64)
    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None:
        return None, None, None
    inl = mask.ravel().astype(bool)
    if inl.sum() < 8:
        return None, None, None
    _, R, t, mask2 = cv2.recoverPose(E, pts1[inl], pts2[inl], K, mask=mask)
    inliers = inl & mask2.ravel().astype(bool)
    return R, t, inliers

def filter_cheirality(X, R, t):
    # Keep points in front of both cameras
    # Cam1: [I|0], Cam2: [R|t]
    z1 = X[:, 2] > 0
    X2 = (R @ X.T + t.reshape(3,1)).T
    z2 = X2[:, 2] > 0
    return z1 & z2

def reproj_error(K, R, t, X, pts):
    P = make_proj_matrix(K, R, t)
    X_h = np.hstack([X, np.ones((X.shape[0], 1))]).T
    x = (P @ X_h)
    x = (x[:2] / x[2]).T
    err = np.linalg.norm(x - pts, axis=1)
    return err

def incremental_sfm(frames, K, detector, norm_type, min_track=100):
    H, W = frames[0].shape[:2]
    # Global pose chain
    R_global = np.eye(3, dtype=np.float64)
    t_global = np.zeros((3,), dtype=np.float64)

    all_points = []

    prev = frames[0]
    kp1, des1, _ = detect_and_describe(detector, prev)
    if des1 is None or len(kp1) < min_track:
        raise RuntimeError("Not enough features in first frame.")

    for i in range(1, len(frames)):
        curr = frames[i]
        kp2, des2, _ = detect_and_describe(detector, curr)
        if des2 is None or len(kp2) < 10:
            continue

        matches = match_descriptors(des1, des2, norm_type)
        if len(matches) < min_track:
            # too few reliable matches; skip this step but keep previous for stability
            continue

        pts1 = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float64)
        pts2 = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float64)

        # Estimate relative motion
        E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            continue
        inl = mask.ravel().astype(bool)
        if inl.sum() < 8:
            continue

        _, R_rel, t_rel, mask2 = cv2.recoverPose(E, pts1[inl], pts2[inl], K, mask=mask)
        inliers = inl & mask2.ravel().astype(bool)

        # Build projection matrices for triangulation
        P1 = make_proj_matrix(K, np.eye(3), np.zeros(3))
        P2 = make_proj_matrix(K, R_rel, t_rel)

        # Triangulate in the local pair coordinates
        X = cv2.triangulatePoints(P1, P2, pts1[inliers].T, pts2[inliers].T).T
        X = (X[:, :3] / X[:, 3:4])

        # Cheirality + reprojection sanity
        keep = filter_cheirality(X, R_rel, t_rel)
        if keep.sum() < 50:
            # bad pair; skip
            continue

        X = X[keep]
        p1 = pts1[inliers][keep]
        p2 = pts2[inliers][keep]

        # Cull large reprojection error in the pair space
        err1 = reproj_error(K, np.eye(3), np.zeros(3), X, p1)
        err2 = reproj_error(K, R_rel, t_rel, X, p2)
        good = (err1 < 3.0) & (err2 < 3.0)
        X = X[good]

        # Scale is arbitrary; keep relative scale consistent by normalizing translation
        scale = np.linalg.norm(t_rel)
        if scale > 1e-6:
            t_rel_scaled = t_rel / scale
        else:
            t_rel_scaled = t_rel

        # Update global pose
        R_global = R_rel @ R_global
        t_global = R_rel @ t_global + t_rel_scaled

        # Bring points from local pair coords ([I|0]/[R_rel|t_rel]) to global world coords
        # Here, local world == first camera of the pair. Its pose in global after update is (R_global_prev, t_global_prev).
        # We need points in the previous global frame before the update.
        # Compute previous global pose by undoing the last update:
        R_prev = R_rel.T @ R_global
        t_prev = -R_rel.T @ (t_global - (R_rel @ t_prev)) if i == 1 else None  # tricky to back-solve without storing
        # Easier approach: accumulate points in the *current* global frame by transforming with current R_global,t_global relative to camera1 of pair.
        # Since X is in cam1=identity of the pair, and that cam1 is at the previous global pose (call it Rg_old,tg_old).
        # To avoid bookkeeping, store and use the running (Rg_old,tg_old) we had *before* updating.
        # So keep a copy:
        if i == 1:
            # initialize
            Rg_old_cache = np.eye(3)
            tg_old_cache = np.zeros(3)
        # Transform X by the old global pose:
        Xg = (Rg_old_cache @ X.T + tg_old_cache.reshape(3,1)).T
        all_points.append(Xg)

        # Prepare for next iteration: now this old becomes the current global pose (camera at new spot)
        Rg_old_cache = R_global.copy()
        tg_old_cache = t_global.copy()

        # Advance features
        kp1, des1 = kp2, des2

    if len(all_points) == 0:
        raise RuntimeError("No 3D points reconstructed. Try lowering --min-track, stride, or ensure good video content.")

    pts = np.vstack(all_points)

    return pts

def clean_and_save_pcd(points, output_path, denoise=True):
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    if denoise and len(points) >= 100:
        # light statistical outlier removal
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    # optional: voxel downsample for lighter files
    if np.asarray(pcd.points).shape[0] > 200000:
        pcd = pcd.voxel_down_sample(voxel_size=0.01)
    o3d.io.write_point_cloud(output_path, pcd, write_ascii=True, compressed=False, print_progress=False)

def main():
    args = parse_args()
    frames = sample_frames(args.video, args.stride, args.max_frames)
    h, w = frames[0].shape[:2]
    K = build_camera_matrix(w, h, focal=args.focal, K_override=args.K)
    detector, name, norm = get_detector(args.use_orb)
    print(f"[info] Using {name} features | frames={len(frames)} | K=\n{K}")

    pts = incremental_sfm(frames, K, detector, norm, min_track=args.min_track)

    # Drop NaNs/infs
    m = np.isfinite(pts).all(axis=1)
    pts = pts[m]

    clean_and_save_pcd(pts, args.output, denoise=(not args.no_denoise))
    print(f"[ok] Saved point cloud: {args.output}  | points={pts.shape[0]}")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[error] {e}")
        sys.exit(1)
