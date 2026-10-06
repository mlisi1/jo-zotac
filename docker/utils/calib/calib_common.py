"""
Shared helpers for the Jo calibration scripts (run inside the jo-zotac container).

- bag reading (storage format auto-detected: mcap or sqlite3)
- SE3 helpers in GLIM's TUM order [x, y, z, qx, qy, qz, qw]
- current values from the URDF (via xacro) and from GLIM's config_sensors.json
- a standard output folder per calibration run (see CALIBRATION.md, section 13)
"""
import datetime
import hashlib
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

BAGS_ROOT = "/home/ros/bags"
CALIB_ROOT = os.environ.get("JO_CALIB_ROOT", "/home/ros/calibration")   # repo-root calibration/, mounted in compose.yaml
URDF_XACRO = "/home/ros/src/jo_description/urdf/jo_main.urdf.xacro"
GLIM_SENSORS = "/home/ros/src/jo_navigation/config/glim/glim_config_bunker/config_sensors.json"


# ----------------------------------------------------------------------------- bags

def resolve_bag(bag):
    for p in (bag, os.path.join(BAGS_ROOT, bag)):
        if os.path.isdir(p):
            return os.path.abspath(p)
    sys.exit(f"Error: bag '{bag}' not found (checked as-is and under {BAGS_ROOT}/)")


def read_messages(bag, topics):
    """Yield (topic, msg, recv_time_s) for the given topics, in file order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    # empty storage_id -> rosbag2 infers the format from metadata.yaml
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        print(f"warning: topics not in bag: {missing}", file=sys.stderr)
    wanted = [t for t in topics if t in types]
    if not wanted:
        sys.exit("Error: none of the requested topics are in the bag")
    r.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    while r.has_next():
        topic, data, t = r.read_next()
        yield topic, deserialize_message(data, classes[topic]), t * 1e-9


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


# ----------------------------------------------------------------------------- SE3 (TUM order)

def tum_to_mat(T):
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat(T[3:7]).as_matrix()
    M[:3, 3] = T[:3]
    return M


def mat_to_tum(M):
    return [float(v) for v in M[:3, 3]] + [float(v) for v in Rotation.from_matrix(M[:3, :3]).as_quat()]


def xyz_rpy_to_mat(xyz, rpy):
    """URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll) == scipy extrinsic 'xyz'."""
    M = np.eye(4)
    M[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    M[:3, 3] = xyz
    return M


def mat_to_xyz_rpy(M):
    return [float(v) for v in M[:3, 3]], [float(v) for v in Rotation.from_matrix(M[:3, :3]).as_euler("xyz")]


def rot_angle_deg(Ma, Mb):
    """Angle of the rotation between two transforms, degrees."""
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(Ma[:3, :3].T @ Mb[:3, :3]).as_rotvec())))


def fmt_tum(T):
    return "[" + ", ".join(f"{v:.6f}" for v in T) + "]"


def fmt_xyz_rpy(M, prefix):
    xyz, rpy = mat_to_xyz_rpy(M)
    names = ["x", "y", "z", "R", "P", "Y"]
    return "\n".join(f'  <xacro:property name="{prefix}_{n}" value="{v:.5f}" />'
                     for n, v in zip(names, list(xyz) + list(rpy)))


# ----------------------------------------------------------------------------- current values

def load_jsonc(path):
    """GLIM configs are JSON with // and /* */ comments."""
    s = open(path).read()
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    s = re.sub(r"(^|[^:])//[^\n]*", r"\1", s)
    return json.loads(s)


def glim_T_lidar_imu(path=GLIM_SENSORS):
    return list(load_jsonc(path)["sensors"]["T_lidar_imu"])


def urdf_joints(xacro_path=URDF_XACRO):
    """{joint_name: (parent, child, 4x4 parent_T_child)} from the expanded URDF."""
    try:
        urdf = subprocess.run(["xacro", xacro_path], check=True, capture_output=True, text=True).stdout
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit(f"Error: could not expand {xacro_path} with xacro (is the workspace sourced?): {e}")
    joints = {}
    for j in ET.fromstring(urdf).iter("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        joints[j.get("name")] = (j.find("parent").get("link"), j.find("child").get("link"), xyz_rpy_to_mat(xyz, rpy))
    return joints


def urdf_T(joints, parent, child):
    """parent_T_child by walking the joint tree (only works for a direct joint chain parent->...->child)."""
    by_child = {c: (p, M) for p, c, M in joints.values()}
    M, link = np.eye(4), child
    while link != parent:
        if link not in by_child:
            sys.exit(f"Error: no URDF chain from {parent} to {child}")
        p, Mj = by_child[link]
        M = Mj @ M
        link = p
    return M


# ----------------------------------------------------------------------------- output

def new_run_dir(step, bag=None, out=None):
    """Create <CALIB_ROOT>/runs/<YYYYmmdd_HHMMSS>_<step>[_<bag>]/ (or --out)."""
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{ts}_{step}" + (f"__{os.path.basename(bag.rstrip('/'))}" if bag else "")
    d = out or os.path.join(CALIB_ROOT, "runs", name)
    os.makedirs(d, exist_ok=True)
    return d


def save_run(run_dir, step, args, result, bag=None, notes=None):
    """Write result.yaml (the numbers) and meta.yaml (how they were produced)."""
    script = os.path.abspath(sys.argv[0])
    meta = {
        "step": step,
        "date": datetime.datetime.now().isoformat(timespec="seconds"),
        "script": os.path.basename(script),
        "script_sha256": hashlib.sha256(open(script, "rb").read()).hexdigest()[:16],
        "command": " ".join(sys.argv),
        "args": {k: v for k, v in vars(args).items()},
        "bag": bag,
        "notes": notes or "",
    }
    with open(os.path.join(run_dir, "meta.yaml"), "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    with open(os.path.join(run_dir, "result.yaml"), "w") as f:
        yaml.safe_dump(_plain(result), f, sort_keys=False)
    print(f"\nsaved: {run_dir}/result.yaml (+ meta.yaml)")


def _plain(o):
    if isinstance(o, dict):
        return {k: _plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v) for v in o]
    if isinstance(o, np.ndarray):
        return _plain(o.tolist())
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    return o


def try_pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        print("note: matplotlib not available, skipping plots", file=sys.stderr)
        return None
