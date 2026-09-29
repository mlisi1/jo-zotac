#!/usr/bin/env python3
"""
Export every distinct RGB (and optionally depth) camera stream in a ROS 2 bag
to an .mp4 file, streaming frames straight into ffmpeg (no per-frame files on disk).

Run this from inside the jo-zotac container (it needs rosbag2_py, cv_bridge,
opencv and ffmpeg, all available there). Normally invoked via the
`extract_videos` alias (see utilities.sh) or extract_camera_videos.sh.

Camera topics normally exist as several sibling "transports" of the same
logical stream, e.g.:
    .../color/image_raw                (sensor_msgs/Image, raw)
    .../color/image_raw/compressed      (sensor_msgs/CompressedImage, jpeg)
    .../color/image_raw/zstd            (sensor_msgs/CompressedImage, custom)
    .../color/image_raw/theora          (theora_image_transport/Packet)
These are grouped by their common "base" topic and only ONE is exported per
group: prefer raw image_raw, else compressed (jpeg/png), else whatever else
is actually available.
"""
import argparse
import gzip
import os
import re
import struct
import subprocess
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage"

# Where bags live inside the container (matches the ./bags:/home/ros/bags
# mount in compose.yaml) -- lets you pass just a bag folder name.
BAGS_ROOT = "/home/ros/bags"


def resolve_bag_path(bag_arg):
    if os.path.isdir(bag_arg):
        return os.path.abspath(bag_arg)
    candidate = os.path.join(BAGS_ROOT, bag_arg)
    if os.path.isdir(candidate):
        return candidate
    raise SystemExit(
        f"Error: could not find bag '{bag_arg}' (checked as-is and under {BAGS_ROOT}/)"
    )

# Suffixes image_transport appends for each publishing plugin. Order = preference
# when picking which sibling transport to actually export (index 0 wins).
TRANSPORT_PRIORITY = ["", "compressed", "zstd", "compressedDepth", "theora"]


def strip_transport_suffix(topic):
    """Return (base_topic, suffix) for a topic name, suffix='' for raw image_raw."""
    for suf in ("compressedDepth", "compressed", "zstd", "theora"):
        if topic.endswith("/" + suf):
            return topic[: -(len(suf) + 1)], suf
    return topic, ""


def classify(base_topic):
    b = base_topic.lower()
    if "depth" in b:
        return "depth"
    if re.search(r"infra|/ir[0-9]?/|ir_", b):
        return "ir"
    return "color"


def discover_streams(reader):
    meta = reader.get_metadata()
    counts = {t.topic_metadata.name: t.message_count for t in meta.topics_with_message_count}
    types = {t.topic_metadata.name: t.topic_metadata.type for t in meta.topics_with_message_count}
    duration_s = meta.duration.nanoseconds / 1e9

    groups = {}  # base_topic -> {suffix: topic_name}
    for topic, mtype in types.items():
        if mtype not in (IMAGE_TYPE, COMPRESSED_TYPE):
            continue
        if counts.get(topic, 0) == 0:
            continue
        base, suf = strip_transport_suffix(topic)
        groups.setdefault(base, {})[suf] = topic

    streams = []
    for base, variants in groups.items():
        chosen_suf = next((s for s in TRANSPORT_PRIORITY if s in variants), None)
        if chosen_suf is None:
            continue
        topic = variants[chosen_suf]
        mtype = types[topic]
        count = counts[topic]
        fps = count / duration_s if duration_s > 0 else 15.0
        streams.append({
            "base": base,
            "topic": topic,
            "suffix": chosen_suf,
            "type": mtype,
            "category": classify(base),
            "count": count,
            "fps": fps,
            "available_suffixes": sorted(variants.keys()),
        })
    streams.sort(key=lambda s: s["base"])
    return streams


def make_reader(bag_path, topic=None):
    so = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    co = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    r = rosbag2_py.SequentialReader()
    r.open(so, co)
    if topic is not None:
        r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return r


def colorize_depth(depth_mm, max_mm=8000):
    depth = np.clip(depth_mm.astype(np.float32), 0, max_mm)
    depth_8u = (depth / max_mm * 255).astype(np.uint8)
    depth_8u[depth_mm == 0] = 0
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_TURBO)


def decode_zstd_depth_frame(data):
    """
    This bag's '.../zstd' depth topic is NOT actually zstd: it's a small
    (height:int32, width:int32, ...) header followed by a plain gzip stream
    of raw 16UC1 (mm) pixel data -- reverse engineered by inspecting the raw
    bytes, since the format field alone just says 'zstd'. If real zstd
    ever shows up instead (proper zstd magic bytes 28 B5 2F FD), fall back
    to that.
    """
    height, width = struct.unpack_from("<ii", data, 0)
    gzip_magic = b"\x1f\x8b"
    idx = data.find(gzip_magic)
    if idx != -1:
        raw = gzip.decompress(bytes(data[idx:]))
    else:
        try:
            import zstandard
            raw = zstandard.ZstdDecompressor().decompress(bytes(data), max_output_size=height * width * 4)
        except Exception as e:
            raise RuntimeError(f"could not decode zstd-labeled depth frame: {e}")
    depth = np.frombuffer(raw, dtype=np.uint16, count=height * width).reshape(height, width)
    return depth


def decode_compressed_depth_frame(data):
    """
    Standard compressed_depth_image_transport format: a 12-byte header
    (format:int32, depthQuantA:float32, depthQuantB:float32) followed by a
    PNG. Not present in the bags this tool was built against -- best effort.
    """
    png_bytes = bytes(data[12:])
    arr = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if arr is None:
        raise RuntimeError("compressedDepth PNG decode failed")
    return arr


def export_stream(bag_path, stream, out_path, depth_max_mm):
    topic = stream["topic"]
    suffix = stream["suffix"]
    category = stream["category"]
    fps = stream["fps"]

    print(f"[{stream['base']}] topic={topic} type={stream['type']} "
          f"category={category} frames={stream['count']} fps~={fps:.2f} -> {out_path}")

    reader = make_reader(bag_path, topic)

    if stream["type"] == IMAGE_TYPE:
        msg_type = get_message("sensor_msgs/msg/Image")
        from cv_bridge import CvBridge
        bridge = CvBridge()
    else:
        msg_type = get_message("sensor_msgs/msg/CompressedImage")
        bridge = None

    # For the plain "compressed" transport we can pass bytes straight to ffmpeg
    # untouched (no decode/re-encode) -- sniff the codec from the first frame's
    # magic bytes rather than assuming jpeg.
    direct_passthrough = False
    vcodec = "mjpeg"
    if suffix == "compressed":
        peek = make_reader(bag_path, topic)
        if peek.has_next():
            _, data0, _ = peek.read_next()
            msg0 = deserialize_message(data0, msg_type)
            head = bytes(msg0.data)[:8]
            if head.startswith(b"\xff\xd8"):
                direct_passthrough, vcodec = True, "mjpeg"
            elif head.startswith(b"\x89PNG"):
                direct_passthrough, vcodec = True, "png"
        peek.close()

    proc = subprocess.Popen([
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "image2pipe", "-vcodec", vcodec, "-r", f"{fps:.3f}",
        "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-r", f"{fps:.3f}",
        out_path,
    ], stdin=subprocess.PIPE)

    n_ok, n_fail = 0, 0
    consecutive_fail = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_type)
        try:
            if direct_passthrough:
                jpg_bytes = bytes(msg.data)
            elif suffix == "compressed":
                # recognized-but-uncommon codec in this transport: decode+recode
                arr = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
                if arr is None:
                    raise RuntimeError("could not decode 'compressed' frame with any known codec")
                ok, buf = cv2.imencode(".jpg", arr)
                if not ok:
                    raise RuntimeError("jpg encode failed")
                jpg_bytes = buf.tobytes()
            elif stream["type"] == IMAGE_TYPE:
                frame = bridge.imgmsg_to_cv2(msg)
                if category == "depth":
                    frame = colorize_depth(frame.astype(np.float32), depth_max_mm)
                elif frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif msg.encoding.lower() in ("rgb8", "rgba8"):
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                ok, buf = cv2.imencode(".jpg", frame)
                if not ok:
                    raise RuntimeError("jpg encode failed")
                jpg_bytes = buf.tobytes()
            elif suffix == "zstd":
                depth = decode_zstd_depth_frame(bytes(msg.data))
                frame = colorize_depth(depth, depth_max_mm)
                ok, buf = cv2.imencode(".jpg", frame)
                if not ok:
                    raise RuntimeError("jpg encode failed")
                jpg_bytes = buf.tobytes()
            elif suffix == "compressedDepth":
                depth = decode_compressed_depth_frame(bytes(msg.data))
                if depth.dtype != np.uint16:
                    depth = depth.astype(np.uint16)
                frame = colorize_depth(depth, depth_max_mm)
                ok, buf = cv2.imencode(".jpg", frame)
                if not ok:
                    raise RuntimeError("jpg encode failed")
                jpg_bytes = buf.tobytes()
            else:
                raise RuntimeError(f"no decoder implemented for transport '{suffix}'")

            proc.stdin.write(jpg_bytes)
            n_ok += 1
            consecutive_fail = 0
        except Exception as e:
            n_fail += 1
            consecutive_fail += 1
            if n_fail <= 5:
                print(f"  warning: dropped a frame ({e})", file=sys.stderr)
            if consecutive_fail > 100:
                print(f"  aborting '{topic}': {consecutive_fail} consecutive decode failures", file=sys.stderr)
                break

    proc.stdin.close()
    proc.wait()
    reader.close()
    print(f"  wrote {n_ok} frames ({n_fail} dropped) -> {out_path}")
    return n_ok > 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag", help=f"Bag folder name (looked up under {BAGS_ROOT}) or a full path to it")
    ap.add_argument("--out-dir", default=None,
                    help=f"Directory to write .mp4 files into (default: {BAGS_ROOT}/<bag_name>_videos)")
    ap.add_argument("--depth", action="store_true", help="Also export depth streams (colorized)")
    ap.add_argument("--depth-max-mm", type=float, default=8000.0, help="Depth colorization clip range in mm (default 8000)")
    ap.add_argument("--fps", type=float, default=None, help="Override auto-detected fps for every stream")
    ap.add_argument("--list", action="store_true", help="Only list discovered streams, don't export")
    args = ap.parse_args()

    bag_path = resolve_bag_path(args.bag)
    out_dir = args.out_dir or os.path.join(BAGS_ROOT, os.path.basename(bag_path.rstrip("/")) + "_videos")

    reader = make_reader(bag_path)  # opened just to read metadata, no topic filter
    streams = discover_streams(reader)
    reader.close()

    if not streams:
        print("No Image/CompressedImage topics with data found in this bag.")
        return

    os.makedirs(out_dir, exist_ok=True)

    print("Discovered camera streams:")
    for s in streams:
        skip = ""
        if s["category"] == "ir":
            skip = "  [skipped: infrared, not RGB or depth -- rerun logic to include if needed]"
        elif s["category"] == "depth" and not args.depth:
            skip = "  [skipped: depth stream, pass --depth to include]"
        print(f"  {s['base']:60s} via {s['suffix'] or 'image_raw':15s} "
              f"({s['category']}, {s['count']} frames, siblings={s['available_suffixes']}){skip}")

    if args.list:
        return

    ok, failed = [], []
    for s in streams:
        if s["category"] == "ir":
            continue
        if s["category"] == "depth" and not args.depth:
            continue
        if args.fps is not None:
            s["fps"] = args.fps
        safe_name = s["base"].strip("/").replace("/", "_")
        out_path = f"{out_dir.rstrip('/')}/{safe_name}.mp4"
        success = export_stream(bag_path, s, out_path, args.depth_max_mm)
        (ok if success else failed).append(s["base"])

    print()
    print(f"Done. {len(ok)} video(s) written to {out_dir}")
    if failed:
        print(f"Failed/empty: {failed}", file=sys.stderr)


if __name__ == "__main__":
    main()
