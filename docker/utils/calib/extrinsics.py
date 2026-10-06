#!/usr/bin/env python3
"""
Extrinsics bookkeeping between the URDF, GLIM and the calibration tools (CALIBRATION.md sections 2, 7.7, 9.3).

  check
      Compare the LiDAR<->IMU transform implied by the URDF with GLIM's T_lidar_imu.

  imu-from-lidar [--t-lidar-imu x y z qx qy qz qw]
      Given T_lidar_imu (default: the one in GLIM's config) and the URDF's base->LiDAR joint,
      print the imu.xacro values that make the URDF agree with GLIM.

  camera --camera front|back --calib calib.json (--tf-bag BAG | --optical-to-link x y z qx qy qz qw)
      Turn direct_visual_lidar_calibration's T_lidar_camera (LiDAR -> colour optical frame)
      into the base_link -> <camera>_link joint for front_depth.xacro / back_depth.xacro.

Run inside the container with the workspace sourced (uses xacro to expand the URDF).
"""
import argparse
import json
import sys

import numpy as np

import calib_common as cc

LIDAR_JOINT = "base2lidar"
IMU_JOINT = "base2imu"
CAM_JOINT = {"front": "base2front_camera", "back": "base2back_camera"}


def cmd_check(args):
    j = cc.urdf_joints()
    T_bl = cc.urdf_T(j, "base_link", "velodyne")
    T_bi = cc.urdf_T(j, "base_link", "imu_link")
    urdf_li = np.linalg.inv(T_bl) @ T_bi
    glim_li = cc.tum_to_mat(cc.glim_T_lidar_imu(args.glim_config))
    dt = np.linalg.norm(urdf_li[:3, 3] - glim_li[:3, 3])
    da = cc.rot_angle_deg(urdf_li, glim_li)
    print(f"URDF  T_lidar_imu: {cc.fmt_tum(cc.mat_to_tum(urdf_li))}")
    print(f"GLIM  T_lidar_imu: {cc.fmt_tum(cc.mat_to_tum(glim_li))}")
    print(f"difference: translation {dt * 1000:.1f} mm, rotation {da:.3f} deg")
    good = dt < 0.005 and da < 0.05
    print("CONSISTENT" if good else "INCONSISTENT -> run 'extrinsics.py imu-from-lidar' and update imu.xacro")
    return 0 if good else 1


def cmd_imu_from_lidar(args):
    j = cc.urdf_joints()
    T_bl = cc.urdf_T(j, "base_link", "velodyne")
    T_li = args.t_lidar_imu or cc.glim_T_lidar_imu(args.glim_config)
    T_bi = T_bl @ cc.tum_to_mat(T_li)
    print(f"T_lidar_imu used: {cc.fmt_tum(T_li)}")
    print("imu.xacro (joint base2imu has no extra offsets):")
    print(cc.fmt_xyz_rpy(T_bi, "b2i"))


def read_tf_static(bag):
    """{child: (parent, 4x4 parent_T_child)} from /tf_static in a bag."""
    tf = {}
    for _, m, _ in cc.read_messages(bag, ["/tf_static"]):
        for t in m.transforms:
            tr, q = t.transform.translation, t.transform.rotation
            tf[t.child_frame_id] = (t.header.frame_id, cc.tum_to_mat([tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w]))
    return tf


def tf_lookup(tf, target, source):
    """target_T_source from a tf_static dict."""
    def to_root(f):
        M = np.eye(4)
        while f in tf:
            p, T = tf[f]
            M = T @ M
            f = p
        return f, M
    ra, Ma = to_root(target)
    rb, Mb = to_root(source)
    if ra != rb:
        sys.exit(f"Error: {target} and {source} are not connected in /tf_static (roots {ra}, {rb})")
    return np.linalg.inv(Ma) @ Mb


def find_key(o, key):
    if isinstance(o, dict):
        if key in o:
            return o[key]
        for v in o.values():
            r = find_key(v, key)
            if r is not None:
                return r
    return None


def cmd_camera(args):
    calib = json.load(open(args.calib))
    T_lc = find_key(calib, "T_lidar_camera")
    if T_lc is None:
        sys.exit(f"Error: no T_lidar_camera in {args.calib}")
    link = f"{args.camera}_camera_link"
    optical = f"{args.camera}_camera_color_optical_frame"
    if args.optical_to_link:
        T_ol = cc.tum_to_mat(args.optical_to_link)
    elif args.tf_bag:
        T_ol = tf_lookup(read_tf_static(cc.resolve_bag(args.tf_bag)), optical, link)
    else:
        sys.exit("Error: give --tf-bag (a bag with /tf_static from the camera node) or --optical-to-link")

    j = cc.urdf_joints()
    T_bl = cc.urdf_T(j, "base_link", "velodyne")
    T_bc = T_bl @ cc.tum_to_mat(T_lc) @ T_ol          # base_T_link = base_T_lidar * lidar_T_optical * optical_T_link
    T_bc_old = cc.urdf_T(j, "base_link", link)
    xyz, rpy = cc.mat_to_xyz_rpy(T_bc)
    print(f"T_lidar_camera ({optical}): {cc.fmt_tum(T_lc)}")
    print(f"\n{CAM_JOINT[args.camera]} joint origin (base_link -> {link}):")
    print(f'  <origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>')
    print(f"\nchange vs current URDF: translation "
          f"{np.linalg.norm(T_bc[:3, 3] - T_bc_old[:3, 3]) * 1000:.1f} mm, rotation {cc.rot_angle_deg(T_bc, T_bc_old):.2f} deg")
    print("Note: front_depth.xacro computes x as -b2c_x + 0.03; write the joint origin with these numbers directly.")
    run = cc.new_run_dir(f"camera_{args.camera}", None, args.out)
    cc.save_run(run, f"camera_extrinsics_{args.camera}", args,
                {"T_lidar_camera": T_lc, "T_base_camera_link": cc.mat_to_tum(T_bc), "xyz": xyz, "rpy": rpy},
                notes=f"calib.json: {args.calib}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glim-config", default=cc.GLIM_SENSORS)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    p = sub.add_parser("imu-from-lidar")
    p.add_argument("--t-lidar-imu", type=float, nargs=7, default=None)
    p = sub.add_parser("camera")
    p.add_argument("--camera", choices=["front", "back"], required=True)
    p.add_argument("--calib", required=True, help="calib.json from direct_visual_lidar_calibration")
    p.add_argument("--tf-bag", default=None)
    p.add_argument("--optical-to-link", type=float, nargs=7, default=None,
                   help="T from tf2_echo <cam>_camera_color_optical_frame <cam>_camera_link, as x y z qx qy qz qw")
    p.add_argument("--out", default=None)
    args = ap.parse_args()
    sys.exit({"check": cmd_check, "imu-from-lidar": cmd_imu_from_lidar, "camera": cmd_camera}[args.cmd](args) or 0)


if __name__ == "__main__":
    main()
