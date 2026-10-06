#!/usr/bin/env python3
"""
Estimate the VLP-16 mounting roll, pitch and height from the floor plane.

Record a short bag with the robot STANDING STILL on a flat, open floor
(no big objects within a few metres, nothing on the floor), then run:

    python3 /home/ros/utils/calib/floor_plane.py <bag> [--scans 50]

It fits a plane to the floor points of the accumulated scans (RANSAC + SVD
refinement) and prints:
  - LiDAR roll/pitch relative to the floor (URDF rpy convention)
  - LiDAR origin height above the floor
  - the resulting base2lidar joint values (b2l_R, b2l_P, b2l_z) for velodyne.xacro

Yaw cannot be observed from the floor; get it from the straight-driving test
described in CALIBRATION.md.
"""
import argparse
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import calib_common as cc

BAGS_ROOT = "/home/ros/bags"


def resolve_bag(bag):
    for p in (bag, os.path.join(BAGS_ROOT, bag)):
        if os.path.isdir(p):
            return os.path.abspath(p)
    sys.exit(f"Error: bag '{bag}' not found (checked as-is and under {BAGS_ROOT}/)")


def read_clouds(bag, topic, n_scans, skip_s):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    clouds, t0 = [], None
    while r.has_next() and len(clouds) < n_scans:
        _, data, t = r.read_next()
        t0 = t if t0 is None else t0
        if (t - t0) * 1e-9 < skip_s:
            continue
        msg = deserialize_message(data, PointCloud2)
        pts = pc2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        clouds.append(pts.astype(np.float64))
    if not clouds:
        sys.exit(f"Error: no messages on {topic}")
    return np.vstack(clouds), len(clouds)


def fit_plane_ransac(pts, iters, thresh, rng):
    best_inliers, n_pts = None, len(pts)
    for _ in range(iters):
        s = pts[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        d = -n @ s[0]
        inl = np.abs(pts @ n + d) < thresh
        if best_inliers is None or inl.sum() > best_inliers.sum():
            best_inliers = inl
    # SVD refinement on the inliers
    p = pts[best_inliers]
    c = p.mean(axis=0)
    n = np.linalg.svd(p - c, full_matrices=False)[2][2]
    if n[2] < 0:  # normal pointing up (towards the sensor)
        n = -n
    d = -n @ c
    return n, d, best_inliers


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help=f"bag folder (name under {BAGS_ROOT} or full path)")
    ap.add_argument("--topic", default="/velodyne_points")
    ap.add_argument("--scans", type=int, default=50, help="number of scans to accumulate (default 50 = 5 s)")
    ap.add_argument("--skip", type=float, default=1.0, help="seconds to skip at the start of the bag")
    ap.add_argument("--min-range", type=float, default=1.0, help="ignore points closer than this [m]")
    ap.add_argument("--max-range", type=float, default=10.0, help="ignore points farther than this [m]")
    ap.add_argument("--expected-height", type=float, default=0.973,
                    help="rough LiDAR height above the floor [m] (URDF: 0.373 + 0.600); only points within "
                         "--window of this are floor candidates, so desks/benches are not mistaken for the floor")
    ap.add_argument("--window", type=float, default=0.3, help="+/- band around -expected-height [m]")
    ap.add_argument("--thresh", type=float, default=0.03, help="RANSAC inlier distance [m]")
    ap.add_argument("--base-height", type=float, default=0.373,
                    help="base_link height above the floor [m] (URDF: chassis_height + 0.1 = 0.373)")
    ap.add_argument("--out", default=None, help="output folder (default: calibration/runs/...)")
    args = ap.parse_args()

    bag = resolve_bag(args.bag)
    pts, n_used = read_clouds(bag, args.topic, args.scans, args.skip)

    rng_xy = np.linalg.norm(pts[:, :2], axis=1)
    z_ok = np.abs(pts[:, 2] + args.expected_height) < args.window
    cand = pts[(rng_xy > args.min_range) & (rng_xy < args.max_range) & z_ok]
    if len(cand) < 500:
        sys.exit(f"Error: only {len(cand)} floor candidate points; check --expected-height / --window / ranges")

    rng = np.random.default_rng(0)
    n, d, inl = fit_plane_ransac(cand, 300, args.thresh, rng)
    resid = np.abs(cand[inl] @ n + d)

    # Floor normal seen in the LiDAR frame, for a LiDAR mounted with URDF rpy
    # (R = Rz(yaw) Ry(pitch) Rx(roll)) on a level base: n_L = (-sin p, sin r cos p, cos r cos p)
    pitch = -math.asin(max(-1.0, min(1.0, n[0])))
    roll = math.atan2(n[1], n[2])
    height = abs(d)  # distance from the LiDAR origin to the floor plane

    print(f"bag: {os.path.basename(bag)}  scans used: {n_used}  floor candidates: {len(cand)}  inliers: {inl.sum()}")
    print(f"plane fit residual: mean {resid.mean() * 1000:.1f} mm, 95% {np.percentile(resid, 95) * 1000:.1f} mm")
    print(f"floor normal in LiDAR frame: [{n[0]:+.4f} {n[1]:+.4f} {n[2]:+.4f}]")
    print()
    print(f"LiDAR roll  : {math.degrees(roll):+.2f} deg  ({roll:+.4f} rad)")
    print(f"LiDAR pitch : {math.degrees(pitch):+.2f} deg  ({pitch:+.4f} rad)")
    print(f"LiDAR origin height above floor: {height:.3f} m")
    print()
    print("velodyne.xacro (joint base2lidar adds lidar_len_h/2 = 0.036 to b2l_z):")
    print(f"  b2l_R = {roll:.4f}")
    print(f"  b2l_P = {pitch:.4f}")
    print(f"  b2l_z = {height - args.base_height - 0.036:.4f}   "
          f"(origin {height - args.base_height:.3f} m above base_link, base_link {args.base_height:.3f} m above floor)")
    if inl.sum() < 0.2 * len(cand) or np.percentile(resid, 95) > 0.8 * args.thresh * 1.25:
        print("\nWARNING: weak plane fit (few inliers or large residual) -- floor not flat/open enough?")

    result = {"scans": n_used, "floor_candidates": int(len(cand)), "inliers": int(inl.sum()),
              "residual_mm": {"mean": float(resid.mean() * 1000), "p95": float(np.percentile(resid, 95) * 1000)},
              "floor_normal_lidar": n, "lidar_roll_rad": roll, "lidar_pitch_rad": pitch,
              "lidar_height_above_floor_m": height,
              "urdf": {"b2l_R": roll, "b2l_P": pitch, "b2l_z": height - args.base_height - 0.036}}
    run = cc.new_run_dir("floor", bag, args.out)
    cc.save_run(run, "floor_plane", args, result, bag)


if __name__ == "__main__":
    main()
