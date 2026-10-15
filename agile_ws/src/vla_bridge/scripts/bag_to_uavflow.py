#!/usr/bin/env python3
"""Convert recorded vla_bridge bags into SimpleVLADataset folders. No RLDS/HDF5."""

from __future__ import print_function

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

try:
    import rosbag
except ImportError:
    rosbag = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _repo_root():
    here = SCRIPT_DIR
    for _ in range(8):
        if os.path.isfile(os.path.join(here, "session_handoff.md")):
            return here
        here = os.path.dirname(here)
    return os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", "..", ".."))


sys.path.insert(0, SCRIPT_DIR)
from vla_geometry import apply_odom_to_enu_rpy, first_frame_log6, wrap_pi
from vla_image_proc import decode_jpeg_to_rgb, encode_jpeg_rgb, resize_rgb

IMAGE_TOPIC = "/uav1/odin1/image/compressed"
ODIN_ODOM_TOPIC = "/uav1/odin1/odometry"
MAVROS_ODOM_TOPIC = "/uav1/mavros/local_position/odom"
INSTRUCTION_TOPIC = "/vla/instruction"
STATE_TOPIC = "/uav1/mavros/state"

POSE_TOPICS = {
    "odin": ODIN_ODOM_TOPIC,
    "mavros": MAVROS_ODOM_TOPIC,
}


def _stamp_sec(stamp):
    try:
        return stamp.to_sec()
    except AttributeError:
        return float(stamp.secs) + 1e-9 * float(stamp.nsecs)


def _load_meta(bag_path):
    meta_path = os.path.splitext(bag_path)[0] + ".meta.json"
    if not os.path.isfile(meta_path):
        return None, meta_path
    with open(meta_path, "r") as handle:
        return json.load(handle), meta_path


def _read_bag(bag_path):
    images = []
    odoms = {ODIN_ODOM_TOPIC: [], MAVROS_ODOM_TOPIC: []}
    instructions = []
    armed_stamps = []
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(
            topics=[IMAGE_TOPIC, ODIN_ODOM_TOPIC, MAVROS_ODOM_TOPIC, INSTRUCTION_TOPIC, STATE_TOPIC]
        ):
            stamp = _stamp_sec(getattr(msg, "header", t).stamp) if hasattr(msg, "header") else _stamp_sec(t)
            if stamp <= 0.0:
                stamp = _stamp_sec(t)
            if topic == IMAGE_TOPIC:
                images.append((stamp, bytes(bytearray(msg.data))))
            elif topic in odoms:
                pose = msg.pose.pose
                odoms[topic].append(
                    (
                        stamp,
                        np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64),
                        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                    )
                )
            elif topic == INSTRUCTION_TOPIC:
                text = (msg.data or "").strip()
                if text:
                    instructions.append((stamp, text))
            elif topic == STATE_TOPIC:
                if bool(getattr(msg, "armed", False)):
                    armed_stamps.append(stamp)
    images.sort(key=lambda item: item[0])
    for key in odoms:
        odoms[key].sort(key=lambda item: item[0])
    instructions.sort(key=lambda item: item[0])
    return images, odoms, instructions, armed_stamps


def _choose_odom(odoms, pose_source):
    if pose_source == "auto":
        if odoms[MAVROS_ODOM_TOPIC]:
            return MAVROS_ODOM_TOPIC, odoms[MAVROS_ODOM_TOPIC]
        return ODIN_ODOM_TOPIC, odoms[ODIN_ODOM_TOPIC]
    topic = POSE_TOPICS[pose_source]
    return topic, odoms[topic]


def _interp_vec(t0, v0, t1, v1, tq):
    if t1 <= t0:
        return np.asarray(v0, dtype=np.float64)
    alpha = (tq - t0) / (t1 - t0)
    v0 = np.asarray(v0, dtype=np.float64)
    v1 = np.asarray(v1, dtype=np.float64)
    return (1.0 - alpha) * v0 + alpha * v1


def _interp_angle(a0, a1, alpha):
    return wrap_pi(float(a0) + alpha * wrap_pi(float(a1) - float(a0)))


def _pose_at(samples, tq, timeout, rot, trans):
    if not samples:
        return None
    times = [item[0] for item in samples]
    idx = int(np.searchsorted(times, tq))
    if idx <= 0:
        stamp, pos, quat = samples[0]
        if abs(tq - stamp) > timeout:
            return None
        return apply_odom_to_enu_rpy(pos, quat, rot, trans)
    if idx >= len(samples):
        stamp, pos, quat = samples[-1]
        if abs(tq - stamp) > timeout:
            return None
        return apply_odom_to_enu_rpy(pos, quat, rot, trans)
    t0, p0, q0 = samples[idx - 1]
    t1, p1, q1 = samples[idx]
    if (t1 - t0) > (2.0 * timeout):
        return None
    pose0 = apply_odom_to_enu_rpy(p0, q0, rot, trans)
    pose1 = apply_odom_to_enu_rpy(p1, q1, rot, trans)
    alpha = 0.0 if t1 <= t0 else (tq - t0) / (t1 - t0)
    p = _interp_vec(t0, pose0[0], t1, pose1[0], tq)
    rpy = tuple(_interp_angle(pose0[1][i], pose1[1][i], alpha) for i in range(3))
    return p, rpy


def _nearest_image(images, tq):
    times = np.array([item[0] for item in images], dtype=np.float64)
    idx = int(np.argmin(np.abs(times - tq)))
    return images[idx]


def _grid_times(t0, t1, rate_hz):
    step = 1.0 / max(float(rate_hz), 0.1)
    count = int(math.floor((t1 - t0) / step)) + 1
    return [t0 + i * step for i in range(max(count, 1))]


def _write_episode(out_dir, episode_id, instruction, instruction_zh, frames, size, quality, save_full):
    episode_dir = os.path.join(out_dir, episode_id)
    os.makedirs(episode_dir, exist_ok=True)
    if save_full:
        os.makedirs(os.path.join(episode_dir, "full"), exist_ok=True)

    p0 = frames[0]["p"]
    rpy0 = frames[0]["rpy"]
    raw_logs = []
    preprocessed_logs = []
    kept = 0
    for index, frame in enumerate(frames):
        rgb = decode_jpeg_to_rgb(frame["jpeg"])
        if rgb is None:
            raise RuntimeError("failed to decode JPEG at frame %d" % index)
        if save_full:
            with open(os.path.join(episode_dir, "full", "%06d.jpg" % index), "wb") as handle:
                handle.write(frame["jpeg"])
        processed = resize_rgb(rgb, size, "letterbox")
        encoded = encode_jpeg_rgb(processed, quality)
        if encoded is None:
            raise RuntimeError("failed to encode letterbox JPEG at frame %d" % index)
        with open(os.path.join(episode_dir, "%06d.jpg" % index), "wb") as handle:
            handle.write(encoded)
        p_enu = frame["p"]
        roll, pitch, yaw = frame["rpy"]
        raw_logs.append(
            [
                float(p_enu[0]),
                float(p_enu[1]),
                float(p_enu[2]),
                math.degrees(roll),
                math.degrees(yaw),
                math.degrees(pitch),
            ]
        )
        preprocessed_logs.append(first_frame_log6(p_enu, frame["rpy"], p0, rpy0))
        kept += 1

    log = {
        "id": episode_id,
        "instruction": instruction,
        "instruction_zh": instruction_zh,
        "instruction_unified": instruction,
        "raw_logs": raw_logs,
        "preprocessed_logs": preprocessed_logs,
        "length": kept,
    }
    with open(os.path.join(episode_dir, "log.json"), "w") as handle:
        json.dump(log, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return log


def convert_bag(bag_path, args, rot, trans):
    meta, meta_path = _load_meta(bag_path)
    if meta is not None and meta.get("success") is False:
        return {"bag": bag_path, "skipped": "meta.success=false"}

    images, odoms, instructions, armed_stamps = _read_bag(bag_path)
    pose_topic, pose_samples = _choose_odom(odoms, args.pose_source)
    if not images:
        return {"bag": bag_path, "error": "no compressed images"}
    if not pose_samples:
        return {"bag": bag_path, "error": "no odometry on %s" % pose_topic}

    instruction = args.instruction
    if not instruction:
        if instructions:
            instruction = instructions[0][1]
        elif meta:
            instruction = meta.get("instruction") or meta.get("instruction_unified") or ""
    if not instruction:
        return {"bag": bag_path, "error": "empty instruction"}
    instruction_zh = args.instruction_zh
    if not instruction_zh and meta:
        instruction_zh = meta.get("instruction_zh") or ""

    if args.require_armed and not armed_stamps:
        return {"bag": bag_path, "skipped": "not armed"}

    t0, t1 = images[0][0], images[-1][0]
    times = _grid_times(t0, t1, args.rate_hz)
    aligned = []
    dropped_odom = 0
    for tq in times:
        img_stamp, jpeg = _nearest_image(images, tq)
        if abs(img_stamp - tq) > args.image_timeout:
            continue
        pose = _pose_at(pose_samples, tq, args.odom_timeout, rot, trans)
        if pose is None:
            dropped_odom += 1
            continue
        aligned.append({"t": tq, "jpeg": jpeg, "p": pose[0], "rpy": pose[1]})

    if dropped_odom:
        print("warn: %s dropped %d frames for odom timeout" % (os.path.basename(bag_path), dropped_odom))

    cleaned = []
    jumps = 0
    for frame in aligned:
        if cleaned:
            delta = float(np.linalg.norm(frame["p"] - cleaned[-1]["p"]))
            if delta > args.max_jump_m:
                jumps += 1
                continue
        cleaned.append(frame)
    if jumps:
        print("warn: %s skipped %d jump frames (> %.2f m)" % (os.path.basename(bag_path), jumps, args.max_jump_m))

    chunks = [cleaned]
    if args.chunk_sec and args.chunk_sec > 0 and cleaned:
        chunks = []
        start_t = cleaned[0]["t"]
        current = []
        for frame in cleaned:
            if current and (frame["t"] - start_t) >= args.chunk_sec:
                chunks.append(current)
                current = []
                start_t = frame["t"]
            current.append(frame)
        if current:
            chunks.append(current)

    bag_stem = os.path.splitext(os.path.basename(bag_path))[0]
    written = []
    for index, frames in enumerate(chunks):
        if len(frames) < args.min_frames:
            continue
        suffix = "" if len(chunks) == 1 else "_%03d" % (index + 1)
        episode_id = bag_stem + suffix
        log = _write_episode(
            args.out,
            episode_id,
            instruction,
            instruction_zh,
            frames,
            args.size,
            args.jpeg_quality,
            args.save_full,
        )
        duration = frames[-1]["t"] - frames[0]["t"]
        xy = np.array([[row[0], row[1]] for row in log["raw_logs"]], dtype=np.float64)
        yaw = np.array([row[4] for row in log["raw_logs"]], dtype=np.float64)
        displacement = float(np.linalg.norm(xy[-1] - xy[0])) if len(xy) else 0.0
        written.append(
            {
                "id": episode_id,
                "frames": log["length"],
                "duration_sec": round(duration, 3),
                "displacement_m": round(displacement, 3),
                "yaw_deg_min": round(float(np.min(yaw)), 2) if len(yaw) else 0.0,
                "yaw_deg_max": round(float(np.max(yaw)), 2) if len(yaw) else 0.0,
                "pose_topic": pose_topic,
                "instruction": instruction,
            }
        )
    result = {"bag": bag_path, "meta": meta_path if meta else None, "episodes": written}
    if not written:
        result["error"] = "no episode passed filters (min_frames=%d)" % args.min_frames
    return result


def _identity_rt():
    return np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64)


def check_dataset(root):
    episodes = []
    for name in sorted(os.listdir(root)):
        episode_dir = os.path.join(root, name)
        log_path = os.path.join(episode_dir, "log.json")
        if not os.path.isdir(episode_dir) or not os.path.isfile(log_path):
            continue
        with open(log_path, "r") as handle:
            log = json.load(handle)
        images = sorted(
            item
            for item in os.listdir(episode_dir)
            if item.lower().endswith(".jpg") and os.path.isfile(os.path.join(episode_dir, item))
        )
        raw = np.asarray(log["raw_logs"], dtype=np.float64)
        local = np.asarray(log["preprocessed_logs"], dtype=np.float64)
        if raw.ndim != 2 or raw.shape[1] != 6:
            raise SystemExit("%s raw_logs must be [T,6]" % name)
        if local.shape != raw.shape:
            raise SystemExit("%s preprocessed_logs shape mismatch" % name)
        if len(images) != raw.shape[0]:
            raise SystemExit("%s image count %d != logs %d" % (name, len(images), raw.shape[0]))
        if abs(local[0, 0]) > 1e-6 or abs(local[0, 1]) > 1e-6 or abs(local[0, 4]) > 1e-4:
            raise SystemExit("%s frame0 local pose not origin: %s" % (name, local[0].tolist()))
        rgb = decode_jpeg_to_rgb(open(os.path.join(episode_dir, images[0]), "rb").read())
        if rgb is None or rgb.shape[0] != 224 or rgb.shape[1] != 224:
            raise SystemExit("%s first image is not 224x224" % name)
        trajectory = local[:, [0, 1, 2, 4]]
        episodes.append(
            {
                "id": log.get("id", name),
                "frames": int(raw.shape[0]),
                "instruction": log.get("instruction", ""),
                "proprio0": trajectory[0].tolist(),
                "image": images[0],
            }
        )
    if not episodes:
        raise SystemExit("no episodes in %s" % root)
    return episodes


def main():
    parser = argparse.ArgumentParser(description="bag -> SimpleVLADataset folders")
    parser.add_argument("--bags", nargs="+", default=None, help="bag files or globs")
    parser.add_argument("--out", default=os.path.join(_repo_root(), "datasets", "odin_real"))
    parser.add_argument("--pose-source", choices=["auto", "odin", "mavros"], default="auto")
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--size", type=int, default=224)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--image-timeout", type=float, default=0.25)
    parser.add_argument("--max-jump-m", type=float, default=0.5)
    parser.add_argument("--min-frames", type=int, default=5)
    parser.add_argument("--chunk-sec", type=float, default=0.0, help="split a long bag into episodes")
    parser.add_argument("--instruction", default="", help="override bag/meta instruction")
    parser.add_argument("--instruction-zh", default="")
    parser.add_argument("--require-armed", action="store_true")
    parser.add_argument("--save-full", action="store_true")
    parser.add_argument("--check", action="store_true", help="only validate --out")
    args = parser.parse_args()

    if args.check:
        episodes = check_dataset(args.out)
        print(json.dumps({"episodes": episodes, "count": len(episodes)}, indent=2, ensure_ascii=False))
        print("DATASET_OK n=%d" % len(episodes))
        return

    if rosbag is None:
        raise SystemExit("rosbag is required: source /opt/ros/noetic/setup.bash")
    if not args.bags:
        raise SystemExit("--bags is required unless --check")

    bag_paths = []
    for item in args.bags:
        matched = sorted(glob.glob(item))
        if matched:
            bag_paths.extend(matched)
        elif os.path.isfile(item):
            bag_paths.append(item)
    bag_paths = [path for path in bag_paths if path.endswith(".bag") and not path.endswith(".bag.active")]
    if not bag_paths:
        raise SystemExit("no bags matched")

    os.makedirs(args.out, exist_ok=True)
    rot, trans = _identity_rt()
    summaries = [convert_bag(path, args, rot, trans) for path in bag_paths]
    n_ep = sum(len(item.get("episodes") or []) for item in summaries)
    print(json.dumps({"out": args.out, "bags": summaries, "episode_count": n_ep}, indent=2, ensure_ascii=False))
    if n_ep <= 0:
        raise SystemExit("CONVERT_FAILED")
    check_dataset(args.out)
    print("CONVERT_OK episodes=%d out=%s" % (n_ep, args.out))


if __name__ == "__main__":
    main()
