#!/usr/bin/env python3
"""
Mounting yaw of the IMU / LiDAR relative to the robot's forward axis (CALIBRATION.md section 6.2).

    python3 /home/ros/utils/calib/straight_drive_yaw.py <bag> [--topic /glim_ros/odom]

Record GLIM odometry while driving STRAIGHT lines (>= 5 m each, both forwards and
backwards, several times). The script finds the straight segments automatically and, for
each, compares the direction of travel with the heading of the IMU frame:

    psi = heading(IMU x-axis) - direction of travel   ==  IMU yaw on the robot (b2i_Y)

The LiDAR yaw on the robot follows from psi and the rotation in GLIM's T_lidar_imu.
A tracked robot never drives perfectly straight; the chord of each segment is used, and
forward/backward runs are averaged so a systematic drift to one side cancels.

GLIM's ~/odom gives the pose of the IMU frame (child_frame_id = imu frame) in the odom
frame; this is what the script expects.
"""
import argparse
import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation

import calib_common as cc


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/glim_ros/odom", help="nav_msgs/Odometry of the IMU frame")
    ap.add_argument("--min-speed", type=float, default=0.15, help="[m/s]")
    ap.add_argument("--max-yaw-rate", type=float, default=0.05, help="[rad/s], above this it's not straight")
    ap.add_argument("--min-length", type=float, default=5.0, help="minimum segment length [m]")
    ap.add_argument("--glim-config", default=cc.GLIM_SENSORS)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bag = cc.resolve_bag(args.bag)
    t, xy, yaw = [], [], []
    for _, m, _ in cc.read_messages(bag, [args.topic]):
        t.append(cc.stamp_s(m.header))
        p, q = m.pose.pose.position, m.pose.pose.orientation
        xy.append((p.x, p.y))
        yaw.append(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2])
    t, xy, yaw = np.array(t), np.array(xy), np.unwrap(np.array(yaw))
    if len(t) < 20:
        sys.exit(f"Error: only {len(t)} messages on {args.topic}")

    # smoothed speed and yaw rate over ~0.5 s
    w = max(1, int(round(0.5 / np.median(np.diff(t)))))
    k = np.ones(w) / w
    vx = np.gradient(np.convolve(xy[:, 0], k, "same"), t)
    vy = np.gradient(np.convolve(xy[:, 1], k, "same"), t)
    speed = np.hypot(vx, vy)
    yaw_rate = np.gradient(np.convolve(yaw, k, "same"), t)
    straight = (speed > args.min_speed) & (np.abs(yaw_rate) < args.max_yaw_rate)
    straight[:w] = straight[-w:] = False       # convolution edges

    segs, i = [], 0
    while i < len(t):
        if straight[i]:
            j = i
            while j + 1 < len(t) and straight[j + 1]:
                j += 1
            d = xy[j] - xy[i]
            length = float(np.hypot(*d))
            if length >= args.min_length:
                travel = math.atan2(d[1], d[0])
                heading = float(np.mean(yaw[i:j + 1]))
                fwd = math.cos(wrap(heading - travel)) >= 0
                if not fwd:
                    travel = wrap(travel + math.pi)
                segs.append({"t_start": float(t[i] - t[0]), "t_end": float(t[j] - t[0]), "length_m": length,
                             "direction": "forward" if fwd else "backward",
                             "psi_deg": math.degrees(wrap(heading - travel))})
            i = j + 1
        else:
            i += 1

    if not segs:
        sys.exit("Error: no straight segments found (check --min-length / --min-speed / --max-yaw-rate)")

    print(f"{'#':>2} {'start':>7} {'end':>7} {'length':>7} {'dir':>9} {'psi[deg]':>9}")
    for n, s in enumerate(segs):
        print(f"{n:2d} {s['t_start']:7.1f} {s['t_end']:7.1f} {s['length_m']:7.2f} {s['direction']:>9} {s['psi_deg']:+9.3f}")

    def wmean(sel):
        if not sel:
            return None
        L = np.array([s["length_m"] for s in sel])
        return float(np.sum(L * np.array([s["psi_deg"] for s in sel])) / L.sum())

    fw = [s for s in segs if s["direction"] == "forward"]
    bw = [s for s in segs if s["direction"] == "backward"]
    m_f, m_b = wmean(fw), wmean(bw)
    psi = 0.5 * (m_f + m_b) if (m_f is not None and m_b is not None) else wmean(segs)
    spread = float(np.std([s["psi_deg"] for s in segs]))
    print(f"\nforward mean: {m_f if m_f is None else round(m_f, 3)}  backward mean: "
          f"{m_b if m_b is None else round(m_b, 3)}  spread (std): {spread:.3f} deg")
    if m_f is None or m_b is None:
        print("warning: only one driving direction; a systematic drift to one side is not cancelled")

    # LiDAR yaw on the robot: R_base_lidar = R_base_imu * R_lidar_imu^-1 (yaw part only, small angles)
    R_li = Rotation.from_quat(cc.glim_T_lidar_imu(args.glim_config)[3:])
    R_bi = Rotation.from_euler("z", psi, degrees=True)
    yaw_lidar = (R_bi * R_li.inv()).as_euler("xyz", degrees=True)[2]
    print(f"\nIMU yaw on the robot   (imu.xacro b2i_Y):      {psi:+.3f} deg = {math.radians(psi):+.5f} rad")
    print(f"LiDAR yaw on the robot (velodyne.xacro b2l_Y): {yaw_lidar:+.3f} deg = {math.radians(yaw_lidar):+.5f} rad")
    print("Only change the URDF if the value is consistent across runs and larger than ~0.5 deg.")

    result = {"segments": segs, "psi_forward_deg": m_f, "psi_backward_deg": m_b, "spread_deg": spread,
              "imu_yaw_on_base_deg": psi, "lidar_yaw_on_base_deg": float(yaw_lidar)}
    run = cc.new_run_dir("straight_yaw", bag, args.out)
    cc.save_run(run, "straight_drive_yaw", args, result, bag)


if __name__ == "__main__":
    main()
