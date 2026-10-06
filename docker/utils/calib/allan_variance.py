#!/usr/bin/env python3
"""
Allan deviation of the Xsens IMU from a long static recording (CALIBRATION.md section 3).

    python3 /home/ros/utils/calib/allan_variance.py <bag> [--skip-start 900] [--inflate 5]

Reads /imu/data (gyro rad/s, accel m/s^2), computes the overlapping Allan deviation
per axis and extracts:
  N  white noise density      (-1/2 slope, value at tau = 1 s)   -> GLIM imu_gyro_noise / imu_acc_noise
  K  bias random walk         (+1/2 slope, value at tau = 3 s)   -> GLIM imu_bias_noise
  B  bias instability         (flat minimum / 0.664)             -> reference only
and suggests GLIM values = inflate x worst axis.

Output: result.yaml, meta.yaml, allan.png (if matplotlib) and allan.csv in the run folder.
"""
import argparse
import math
import sys

import numpy as np

import calib_common as cc


def overlapping_adev(rate, fs, n_taus=120):
    """Overlapping Allan deviation of a rate signal sampled at fs. Returns (taus, adev)."""
    theta = np.concatenate([[0.0], np.cumsum(rate) / fs])  # integrated signal
    n = len(theta)
    ms = np.unique(np.logspace(0, math.log10((n - 1) / 2), n_taus).astype(int))
    ms = ms[ms >= 1]
    taus, adev = [], []
    for m in ms:
        d = theta[2 * m:] - 2.0 * theta[m:n - m] + theta[:n - 2 * m]
        tau = m / fs
        taus.append(tau)
        adev.append(math.sqrt(np.sum(d * d) / (2.0 * tau * tau * (n - 2 * m))))
    return np.array(taus), np.array(adev)


def fit_params(taus, adev, tau_max):
    """N (slope -1/2 at tau=1), K (slope +1/2 at tau=3), B (min/0.664). K is None if not reached.
    Only tau <= tau_max is used: beyond ~1/10 of the recording there are too few averaging
    windows and the curve is noise."""
    use = taus <= tau_max
    taus, adev = taus[use], adev[use]
    lt, la = np.log10(taus), np.log10(adev)
    slope = np.gradient(la, lt)
    i_min = int(np.argmin(adev))

    # white noise: the point before the minimum whose local slope is closest to -0.5
    left = np.arange(0, max(i_min, 1))
    i_n = left[np.argmin(np.abs(slope[left] + 0.5))]
    N = adev[i_n] * math.sqrt(taus[i_n])

    # rate random walk: the point after the minimum whose slope is closest to +0.5
    K, i_k = None, None
    right = np.arange(i_min + 1, len(taus))
    if len(right) >= 3:
        i_k = right[np.argmin(np.abs(slope[right] - 0.5))]
        if abs(slope[i_k] - 0.5) < 0.25:
            K = adev[i_k] * math.sqrt(3.0 / taus[i_k])
    rising = slope[i_min + 1:]
    drift = bool(len(rising) >= 3 and np.max(rising) > 0.8)   # +1 slope = rate ramp (e.g. temperature), not random walk
    B = adev[i_min] / 0.664
    return {
        "N": float(N), "N_fit_tau_s": float(taus[i_n]), "N_fit_slope": float(slope[i_n]),
        "K": None if K is None else float(K),
        "K_fit_tau_s": None if i_k is None else float(taus[i_k]),
        "B": float(B), "B_tau_s": float(taus[i_min]),
        "drift_suspected": drift, "tau_max_used_s": float(taus[-1]),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/imu/data")
    ap.add_argument("--skip-start", type=float, default=0.0,
                    help="seconds to drop at the start (e.g. 900 if the IMU wasn't warmed up)")
    ap.add_argument("--skip-end", type=float, default=0.0, help="seconds to drop at the end")
    ap.add_argument("--inflate", type=float, default=5.0,
                    help="factor applied to the worst axis for the suggested GLIM values (5-10)")
    ap.add_argument("--out", default=None, help="output folder (default: calibration/runs/...)")
    args = ap.parse_args()

    bag = cc.resolve_bag(args.bag)
    print("reading IMU messages (a 3 h bag at 400 Hz takes a few minutes)...")
    t, gyr, acc = [], [], []
    for i, (_, m, _) in enumerate(cc.read_messages(bag, [args.topic])):
        t.append(cc.stamp_s(m.header))
        gyr.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        acc.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        if i % 500000 == 0 and i:
            print(f"  {i} messages", file=sys.stderr)
    t, gyr, acc = np.array(t), np.array(gyr), np.array(acc)
    order = np.argsort(t)
    t, gyr, acc = t[order], gyr[order], acc[order]

    keep = (t >= t[0] + args.skip_start) & (t <= t[-1] - args.skip_end)
    t, gyr, acc = t[keep], gyr[keep], acc[keep]
    if len(t) < 1000:
        sys.exit("Error: too few samples after skipping")

    dt = np.diff(t)
    duration = t[-1] - t[0]
    # The Allan deviation treats the samples as a uniform grid, so the sampling rate must be the
    # *average* rate. The median gap is misleading when host-side stamps jitter (e.g. 1.5/3 ms).
    fs = (len(t) - 1) / duration
    fs_median = 1.0 / np.median(dt)
    gaps = int(np.sum(dt > 3.0 / fs))
    print(f"samples: {len(t)}  duration: {duration / 3600:.2f} h  average rate: {fs:.1f} Hz  gaps (>3 periods): {gaps}")
    print(f"stamp spacing [ms]: median {np.median(dt) * 1000:.3f}, p5 {np.percentile(dt, 5) * 1000:.3f}, "
          f"p95 {np.percentile(dt, 95) * 1000:.3f}, max {dt.max() * 1000:.1f}")
    if abs(fs_median / fs - 1) > 0.05:
        print(f"note: median-based rate ({fs_median:.1f} Hz) differs from the average rate -> the timestamps jitter "
              "(host-side stamping). The Allan deviation uses the average rate, so the result is not affected.")
    if duration < 3600:
        print("warning: < 1 h of data; the bias random walk (K) will be unreliable or missing")
    if gaps:
        print(f"warning: {gaps} gaps; the Allan deviation assumes uniform sampling "
              "(a few short gaps are fine, many are not)")
    # Allan deviation assumes a uniform grid: the bag's samples are used in order at the median rate.

    result = {"bag": bag, "samples": int(len(t)), "duration_s": float(duration), "rate_hz": float(fs),
              "rate_median_hz": float(fs_median),
              "gaps": gaps, "gyro": {}, "accel": {}}
    curves = {}
    for name, data, unit in (("gyro", gyr, "rad/s"), ("accel", acc, "m/s^2")):
        for k, ax in enumerate("xyz"):
            x = data[:, k] - data[:, k].mean()
            taus, adev = overlapping_adev(x, fs)
            p = fit_params(taus, adev, duration / 10.0)
            result[name][ax] = p
            curves[f"{name}_{ax}"] = (taus, adev)
            k_txt = "n/a (recording too short)" if p["K"] is None else f"{p['K']:.3e}"
            print(f"{name} {ax}: N = {p['N']:.3e} {unit}/sqrt(Hz)   K = {k_txt}   "
                  f"B = {p['B']:.3e} {unit} (tau {p['B_tau_s']:.0f} s)"
                  + ("   [slope ~+1 at long tau: drift, K unreliable]" if p["drift_suspected"] else ""))

    def worst(name, key):
        vals = [result[name][a][key] for a in "xyz" if result[name][a][key] is not None]
        return max(vals) if vals else None

    n_g, n_a = worst("gyro", "N"), worst("accel", "N")
    k_g, k_a = worst("gyro", "K"), worst("accel", "K")
    k_max = max([v for v in (k_g, k_a) if v is not None], default=None)
    ks = [result[n][a]["K"] for n in ("gyro", "accel") for a in "xyz" if result[n][a]["K"] is not None]
    if len(ks) >= 3 and k_max > 3 * float(np.median(ks)):
        print(f"\nwarning: the largest K ({k_max:.2e}) is > 3x the median of all axes ({np.median(ks):.2e}); "
              "check that axis in allan.png (temperature drift?) before using imu_bias_noise")
    suggestion = {
        "inflate": args.inflate,
        "imu_gyro_noise": args.inflate * n_g,
        "imu_acc_noise": args.inflate * n_a,
        "imu_bias_noise": None if k_max is None else args.inflate * k_max,
    }
    result["glim_suggestion"] = suggestion
    print("\nSuggested GLIM config_sensors.json values (worst axis x inflate):")
    for k, v in suggestion.items():
        if k != "inflate":
            print(f'  "{k}": {v:.3e},' if v is not None else f'  "{k}": (keep current, K not measured)')
    print('  "imu_int_noise": keep as is')

    run = cc.new_run_dir("allan", bag, args.out)
    with open(f"{run}/allan.csv", "w") as f:
        f.write("series,tau_s,adev\n")
        for name, (taus, adev) in curves.items():
            for a, b in zip(taus, adev):
                f.write(f"{name},{a:.6g},{b:.6g}\n")
    plt = cc.try_pyplot()
    if plt:
        fig, axs = plt.subplots(1, 2, figsize=(12, 5))
        for i, (name, unit) in enumerate((("gyro", "rad/s"), ("accel", "m/s²"))):
            for ax in "xyz":
                taus, adev = curves[f"{name}_{ax}"]
                axs[i].loglog(taus, adev, label=ax)
            axs[i].set_title(f"{name} Allan deviation")
            axs[i].set_xlabel("tau [s]")
            axs[i].set_ylabel(f"[{unit}]")
            axs[i].grid(True, which="both", alpha=0.3)
            axs[i].legend()
        fig.tight_layout()
        fig.savefig(f"{run}/allan.png", dpi=120)
    cc.save_run(run, "allan_variance", args, result, bag)


if __name__ == "__main__":
    main()
