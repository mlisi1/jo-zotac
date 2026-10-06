#!/usr/bin/env python3
"""
LiDAR <-> IMU rotation and time offset from gyro alignment (CALIBRATION.md section 7.4).

    python3 /home/ros/utils/calib/lidar_imu_rotation.py <bag> [--max-scans 1500]

Needs a bag with /velodyne_points and /imu/data recorded while the robot rotates a lot
(turning on the spot both ways; tilting over a ramp/kerb if possible).

Method
  1. LiDAR-only motion: point-to-plane ICP between consecutive scans (no IMU used),
     with per-point deskewing from the previous estimate. Gives the rotation of the
     LiDAR between scan end times, theta_L (rotation vector, LiDAR frame).
  2. IMU motion: integrate the gyro over the same interval, shifted by a time offset dt,
     theta_I(dt) (IMU frame).
  3. Time offset: the dt that best matches |theta_L| and |theta_I(dt)| (the norm doesn't
     depend on the unknown rotation).
  4. Rotation: theta_L = R_lidar_imu * theta_I, solved as a Wahba/Kabsch problem.
     If the motion only excited one axis (turning on flat ground), the rotation about that
     axis is not observable: then only the tilt is estimated and yaw is kept from the
     current GLIM T_lidar_imu.

Translation is NOT estimated (take it from CAD / tape measure).

Sign of the time offset: GLIM adds imu_time_offset to every IMU stamp. The script reports
the value to put there directly.
"""
import argparse
import math
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

import calib_common as cc


# ----------------------------------------------------------------------------- point cloud utils

def voxel_down(p, size):
    keys = np.floor(p[:, :3] / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return p[idx]


def normals(p, k=12):
    tree = cKDTree(p)
    _, nn = tree.query(p, k=k)
    nb = p[nn]                                   # (M, k, 3)
    c = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("mki,mkj->mij", c, c) / k
    w, v = np.linalg.eigh(cov)                   # ascending eigenvalues
    n = v[:, :, 0]
    planar = w[:, 0] < 0.1 * w[:, 1]             # keep well-defined planes only
    return n, planar, tree


def deskew(pts, times, omega, vel):
    """Move every point to the scan-end frame assuming a constant body twist (omega, vel)."""
    if times is None or omega is None:
        return pts
    dt = times - times.max()                     # <= 0
    R = Rotation.from_rotvec(np.outer(dt, omega))
    return R.apply(pts) + np.outer(dt, vel)


def icp_point_to_plane(src, tgt, tgt_n, tgt_ok, tree, T0, iters=20, max_dist=1.0):
    """Returns 4x4 tgt_T_src, inlier fraction, rms [m]."""
    T = T0.copy()
    fit, rms = 0.0, float("inf")
    for it in range(iters):
        p = src @ T[:3, :3].T + T[:3, 3]
        d, idx = tree.query(p, distance_upper_bound=max_dist)
        m = np.isfinite(d)
        m[m] &= tgt_ok[idx[m]]
        if m.sum() < 50:
            return T, 0.0, float("inf")
        p, q, n = p[m], tgt[idx[m]], tgt_n[idx[m]]
        r = np.einsum("ij,ij->i", n, p - q)
        s = 0.1                                   # Huber threshold [m]
        w = np.where(np.abs(r) < s, 1.0, s / np.abs(r))
        J = np.hstack([np.cross(p, n), n])        # d r / d [omega, v]
        H = (J * w[:, None]).T @ J
        g = (J * w[:, None]).T @ r
        try:
            x = -np.linalg.solve(H + 1e-6 * np.eye(6), g)
        except np.linalg.LinAlgError:
            return T, 0.0, float("inf")
        dT = np.eye(4)
        dT[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
        dT[:3, 3] = x[3:]
        T = dT @ T
        fit, rms = m.mean(), math.sqrt(np.mean(r ** 2))
        if np.linalg.norm(x) < 1e-6:
            break
        if it == iters // 2:
            max_dist = max(0.3, max_dist / 2)
    return T, fit, rms


# ----------------------------------------------------------------------------- IMU utils

class GyroIntegrator:
    """Orientation of the IMU over time from the gyro, interpolated at arbitrary times."""

    def __init__(self, t, w):
        dt = np.diff(t)
        steps = Rotation.from_rotvec(0.5 * (w[1:] + w[:-1]) * dt[:, None])
        q = [Rotation.identity()]
        for s in steps:
            q.append(q[-1] * s)
        self.t = t
        self.slerp = Slerp(t, Rotation.concatenate(q))

    def rel(self, a, b):
        """Rotation vector of the IMU from time a to time b (IMU frame at a)."""
        ra, rb = self.slerp([a, b])
        return (ra.inv() * rb).as_rotvec()


# ----------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--points-topic", default="/velodyne_points")
    ap.add_argument("--imu-topic", default="/imu/data")
    ap.add_argument("--max-scans", type=int, default=1500, help="limit the number of scans processed")
    ap.add_argument("--voxel", type=float, default=0.3, help="voxel size for ICP [m]")
    ap.add_argument("--min-range", type=float, default=1.0)
    ap.add_argument("--max-range", type=float, default=40.0)
    ap.add_argument("--min-rot-deg", type=float, default=0.5,
                    help="ignore scan pairs rotating less than this (no information)")
    ap.add_argument("--max-offset", type=float, default=0.1, help="time offset search range +/- [s]")
    ap.add_argument("--glim-config", default=cc.GLIM_SENSORS, help="config_sensors.json (prior T_lidar_imu)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bag = cc.resolve_bag(args.bag)
    T_prior = cc.glim_T_lidar_imu(args.glim_config)
    R_prior = Rotation.from_quat(T_prior[3:])

    # ---- read data
    print("reading bag...")
    from sensor_msgs_py import point_cloud2 as pc2
    imu_t, imu_w, scans = [], [], []
    for topic, m, _ in cc.read_messages(bag, [args.points_topic, args.imu_topic]):
        if topic == args.imu_topic:
            imu_t.append(cc.stamp_s(m.header))
            imu_w.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        elif len(scans) < args.max_scans:
            names = [f.name for f in m.fields]
            fields = ["x", "y", "z"] + (["time"] if "time" in names else [])
            a = pc2.read_points_numpy(m, field_names=fields, skip_nans=True).astype(np.float64)
            r = np.linalg.norm(a[:, :3], axis=1)
            a = a[(r > args.min_range) & (r < args.max_range)]
            stamp = cc.stamp_s(m.header)
            times = a[:, 3] if a.shape[1] == 4 else None
            t_end = stamp + (times.max() if times is not None else 0.0)
            scans.append((t_end, a[:, :3], times))
    imu_t, imu_w = np.array(imu_t), np.array(imu_w)
    imu_t, o = np.unique(imu_t, return_index=True)   # sorted, duplicate stamps dropped (Slerp needs strictly increasing)
    imu_w = imu_w[o]
    scans.sort(key=lambda s: s[0])
    print(f"scans: {len(scans)}  imu samples: {len(imu_t)}  "
          f"per-point times: {'yes' if scans and scans[0][2] is not None else 'NO (no deskew)'}")
    if len(scans) < 50:
        sys.exit("Error: need at least 50 scans")

    # ---- LiDAR-only odometry between consecutive scans
    print("scan-to-scan ICP (LiDAR only)...")
    pairs = []                                    # (t_prev_end, t_end, theta_L, fitness, rms)
    T_guess = np.eye(4)
    omega, vel = None, None
    prev = None
    for k, (t_end, pts, times) in enumerate(scans):
        pts_d = deskew(pts, times, omega, vel)
        cur = voxel_down(pts_d, args.voxel)
        if prev is not None:
            t_prev, tgt, tgt_n, tgt_ok, tree = prev
            T, fit, rms = icp_point_to_plane(cur, tgt, tgt_n, tgt_ok, tree, T_guess)
            dt = t_end - t_prev
            theta = Rotation.from_matrix(T[:3, :3]).as_rotvec()   # prev_R_cur
            if fit > 0.3 and rms < 0.15 and 0.05 < dt < 0.2:
                pairs.append((t_prev, t_end, theta, fit, rms))
                T_guess = T
                omega, vel = theta / dt, T[:3, 3] / dt
            else:
                T_guess, omega, vel = np.eye(4), None, None
        n, ok, tree = normals(cur)
        prev = (t_end, cur, n, ok, tree)
        if k % 100 == 0:
            print(f"  scan {k}/{len(scans)}  good pairs: {len(pairs)}", file=sys.stderr)

    pairs = [p for p in pairs if np.degrees(np.linalg.norm(p[2])) >= args.min_rot_deg]
    pairs = [p for p in pairs if imu_t[0] + args.max_offset < p[0] and p[1] < imu_t[-1] - args.max_offset]
    print(f"usable rotating pairs: {len(pairs)}")
    if len(pairs) < 30:
        sys.exit("Error: fewer than 30 rotating scan pairs -- rotate more while recording")

    gyro = GyroIntegrator(imu_t, imu_w)
    theta_L = np.array([p[2] for p in pairs])
    ta = np.array([p[0] for p in pairs])
    tb = np.array([p[1] for p in pairs])

    def theta_I(offset):
        # IMU stamp t_imu corresponds to LiDAR time t_imu + imu_time_offset (GLIM convention),
        # so the LiDAR interval [a, b] is IMU interval [a - off, b - off].
        return np.array([gyro.rel(a - offset, b - offset) for a, b in zip(ta, tb)])

    # ---- time offset
    nl = np.linalg.norm(theta_L, axis=1)
    offsets = np.arange(-args.max_offset, args.max_offset + 1e-9, 0.001)
    costs = np.array([np.mean((nl - np.linalg.norm(theta_I(o), axis=1)) ** 2) for o in offsets])
    i = int(np.argmin(costs))
    best = offsets[i]
    if 0 < i < len(offsets) - 1:                  # parabolic refinement
        c0, c1, c2 = costs[i - 1:i + 2]
        den = c0 - 2 * c1 + c2
        if den > 0:
            best += 0.5 * (c0 - c2) / den * 0.001
    at_edge = i in (0, len(offsets) - 1)
    thI = theta_I(best)

    # ---- rotation
    S = thI.T @ thI
    ev = np.sort(np.linalg.eigvalsh(S))[::-1]
    full = ev[1] > 0.05 * ev[0]
    if full:
        B = theta_L.T @ thI
        U, _, Vt = np.linalg.svd(B)
        D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
        R_est = Rotation.from_matrix(U @ D @ Vt)
        mode = "full rotation (motion excited >= 2 axes)"
    else:
        # only the dominant rotation axis is observed: align it, keep the rest from the prior
        a_I = np.linalg.eigh(S)[1][:, -1]
        a_L = np.linalg.eigh(theta_L.T @ theta_L)[1][:, -1]
        if np.sum((theta_L @ a_L) * (thI @ a_I)) < 0:
            a_L = -a_L
        u = R_prior.apply(a_I)
        axis = np.cross(u, a_L)
        ang = math.atan2(np.linalg.norm(axis), np.dot(u, a_L))
        Rc = Rotation.from_rotvec(axis / (np.linalg.norm(axis) + 1e-12) * ang)
        R_est = Rc * R_prior
        mode = ("tilt only: motion excited essentially one axis (turning on flat ground); rotation about "
                "that axis kept from the current T_lidar_imu")

    res_prior = np.degrees(np.linalg.norm(theta_L - R_prior.apply(thI), axis=1))
    res_est = np.degrees(np.linalg.norm(theta_L - R_est.apply(thI), axis=1))
    T_new = list(T_prior[:3]) + list(R_est.as_quat())
    rpy = R_est.as_euler("xyz", degrees=True)
    rpy_prior = R_prior.as_euler("xyz", degrees=True)
    change = np.degrees(np.linalg.norm((R_prior.inv() * R_est).as_rotvec()))

    print(f"\nexcitation eigenvalues (IMU rotation, deg^2): {np.degrees(np.degrees(ev)).round(1)}")
    print(f"mode: {mode}")
    print(f"\ntime offset: imu_time_offset = {best * 1000:+.1f} ms"
          + ("   WARNING: at the edge of the search range, increase --max-offset" if at_edge else ""))
    print(f"rotation R_lidar_imu rpy [deg]: {rpy[0]:+.3f} {rpy[1]:+.3f} {rpy[2]:+.3f}   "
          f"(current: {rpy_prior[0]:+.3f} {rpy_prior[1]:+.3f} {rpy_prior[2]:+.3f}, change {change:.3f} deg)")
    print(f"residual per scan pair [deg]: current {np.median(res_prior):.3f} (median) -> new {np.median(res_est):.3f}")
    print(f"\nconfig_sensors.json:  \"T_lidar_imu\": {cc.fmt_tum(T_new)}")
    print(f"config_ros.json:      \"imu_time_offset\": {best:.4f}")
    print("(translation copied from the current config; then update the URDF with extrinsics.py imu-from-lidar)")

    result = {
        "mode": mode, "full_rotation_observable": bool(full),
        "excitation_eigenvalues_deg2": np.degrees(np.degrees(ev)),
        "pairs_used": len(pairs),
        "imu_time_offset_s": float(best), "time_offset_at_search_edge": bool(at_edge),
        "T_lidar_imu": T_new, "T_lidar_imu_prior": T_prior,
        "rpy_deg": rpy, "rpy_prior_deg": rpy_prior, "change_deg": float(change),
        "residual_median_deg": {"prior": float(np.median(res_prior)), "new": float(np.median(res_est))},
    }
    run = cc.new_run_dir("lidar_imu", bag, args.out)
    plt = cc.try_pyplot()
    if plt:
        fig, axs = plt.subplots(2, 1, figsize=(11, 7))
        axs[0].plot(offsets * 1000, costs)
        axs[0].axvline(best * 1000, color="r")
        axs[0].set_xlabel("imu_time_offset [ms]")
        axs[0].set_ylabel("rotation-norm mismatch")
        tt = 0.5 * (ta + tb) - ta[0]
        axs[1].plot(tt, np.degrees(nl), ".", ms=3, label="LiDAR |theta|")
        axs[1].plot(tt, np.degrees(np.linalg.norm(thI, axis=1)), ".", ms=3, label="IMU |theta| (shifted)")
        axs[1].set_xlabel("time [s]")
        axs[1].set_ylabel("rotation per scan [deg]")
        axs[1].legend()
        fig.tight_layout()
        fig.savefig(f"{run}/lidar_imu.png", dpi=120)
    cc.save_run(run, "lidar_imu_rotation", args, result, bag)


if __name__ == "__main__":
    main()
