#!/usr/bin/env python3
"""
Check the Xsens magnetometer calibration from a turn-in-place recording (CALIBRATION.md section 4).

    python3 /home/ros/utils/calib/mag_check.py <bag> [--topic /imu/mag]

Record /imu/mag and /imu/data while the robot turns SLOWLY on the spot, at least
two full turns, on level ground away from large steel objects. This script does NOT
calibrate the Xsens (that's done with MT Manager's Magnetic Field Mapper, whose result is
stored in the device); it measures how good the current calibration is, so you can
check it before and after running the MFM.

For a well-calibrated magnetometer, the horizontal field traces a circle centred on
zero while turning. The script fits an ellipse to the horizontal components (tilt-
compensated with the orientation from /imu/data) and reports:
  - hard-iron offset   : ellipse centre, as % of the radius
  - soft-iron distortion: axis ratio of the ellipse
  - worst heading error these cause
  - field-norm variation (should be ~constant while turning)
"""
import argparse
import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation

import calib_common as cc


def fit_ellipse(x, y):
    """Least-squares conic fit a x^2 + b xy + c y^2 + d x + e y = 1 -> centre, axes, angle."""
    D = np.column_stack([x * x, x * y, y * y, x, y])
    a, b, c, d, e = np.linalg.lstsq(D, np.ones_like(x), rcond=None)[0]
    M = np.array([[a, b / 2], [b / 2, c]])
    centre = np.linalg.solve(2 * M, [-d, -e])
    k = 1 + centre @ M @ centre
    w, v = np.linalg.eigh(M / k)
    if np.any(w <= 0):
        sys.exit("Error: data isn't elliptical (did the robot do full turns?)")
    axes = 1.0 / np.sqrt(w)
    return centre, axes, v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu/mag")
    ap.add_argument("--imu-topic", default="/imu/data", help="orientation for tilt compensation ('' to skip)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bag = cc.resolve_bag(args.bag)
    topics = [args.topic] + ([args.imu_topic] if args.imu_topic else [])
    mt, mag, qt, quat = [], [], [], []
    for topic, m, _ in cc.read_messages(bag, topics):
        if topic == args.topic:
            v = m.magnetic_field if hasattr(m, "magnetic_field") else m.vector   # MagneticField or Vector3Stamped
            mt.append(cc.stamp_s(m.header))
            mag.append((v.x, v.y, v.z))
        else:
            o = m.orientation
            qt.append(cc.stamp_s(m.header))
            quat.append((o.x, o.y, o.z, o.w))
    mt, mag = np.array(mt), np.array(mag)
    if len(mt) < 100:
        sys.exit(f"Error: only {len(mt)} messages on {args.topic}")

    if quat:
        qt, quat = np.array(qt), np.array(quat)
        idx = np.clip(np.searchsorted(qt, mt), 0, len(qt) - 1)
        rp = Rotation.from_quat(quat[idx]).as_euler("xyz")
        tilt = Rotation.from_euler("xy", rp[:, :2])            # remove roll & pitch only
        h = tilt.apply(mag)
    else:
        h = mag
    x, y = h[:, 0], h[:, 1]

    heading = np.unwrap(np.arctan2(y, x))
    turns = abs(heading[-1] - heading[0]) / (2 * math.pi)
    if turns < 1.0:
        print(f"warning: only {turns:.2f} turns in the data; do at least 2 full turns")

    centre, axes, vecs = fit_ellipse(x, y)
    radius = float(np.mean(axes))
    offset_pct = float(np.linalg.norm(centre) / radius * 100)
    ratio = float(axes.max() / axes.min())

    # heading error: raw heading vs heading after removing the fitted distortion
    p = np.column_stack([x, y]) - centre
    p_corr = (p @ vecs) / axes * radius @ vecs.T
    err = np.degrees(np.abs(np.angle(np.exp(1j * (np.arctan2(y, x) - np.arctan2(p_corr[:, 1], p_corr[:, 0]))))))
    norm = np.linalg.norm(mag, axis=1)
    norm_var = float(np.std(norm) / np.mean(norm) * 100)

    ok = offset_pct < 5 and ratio < 1.05 and np.percentile(err, 95) < 3
    print(f"samples: {len(mt)}  turns: {turns:.2f}  tilt compensation: {'yes' if quat is not None and len(quat) else 'no'}")
    print(f"hard-iron offset : {offset_pct:.1f} % of radius   (centre {centre.round(4)})")
    print(f"soft-iron ratio  : {ratio:.3f}   (1.000 = perfect circle)")
    print(f"heading error    : 95% {np.percentile(err, 95):.1f} deg, max {err.max():.1f} deg")
    print(f"field norm var.  : {norm_var:.1f} % while turning")
    print(f"\nverdict: {'OK' if ok else 'NEEDS CALIBRATION (run the Magnetic Field Mapper, section 4)'}")

    result = {"samples": int(len(mt)), "turns": float(turns), "hard_iron_offset_pct": offset_pct,
              "hard_iron_centre": centre, "soft_iron_axis_ratio": ratio,
              "heading_error_deg": {"p95": float(np.percentile(err, 95)), "max": float(err.max())},
              "field_norm_variation_pct": norm_var, "ok": bool(ok)}
    run = cc.new_run_dir("mag_check", bag, args.out)
    plt = cc.try_pyplot()
    if plt:
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(x, y, ".", ms=2, label="horizontal field")
        a = np.linspace(0, 2 * math.pi, 200)
        e = (np.column_stack([np.cos(a), np.sin(a)]) * axes) @ vecs.T + centre
        ax.plot(e[:, 0], e[:, 1], "r-", label="fitted ellipse")
        ax.plot(0, 0, "k+", ms=12)
        ax.set_aspect("equal")
        ax.legend()
        ax.set_title("magnetometer, turning in place")
        fig.savefig(f"{run}/mag_check.png", dpi=120)
    cc.save_run(run, "mag_check", args, result, bag)


if __name__ == "__main__":
    main()
