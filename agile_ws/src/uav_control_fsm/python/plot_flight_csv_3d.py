#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 uav_path_recorder 宽表 CSV，在三维网格中对比实际轨迹与期望指令。

用法示例：
  python3 plot_flight_csv_3d.py
  python3 plot_flight_csv_3d.py ~/flight_logs/uav1_trajectory_xxx.csv
  python3 plot_flight_csv_3d.py --dir ~/flight_logs --latest
  python3 plot_flight_csv_3d.py --list
"""

from __future__ import print_function

import argparse
import csv
import glob
import os
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # 注册 3D


DEFAULT_LOG_DIR = os.path.expanduser("~/flight_logs")
CSV_GLOB = "uav*_trajectory_*.csv"

MOVE_MODE_NAMES = {
    0: "pos_xyz",
    1: "pos_xyz_yaw_rat",
    2: "vel_xyz",
    3: "vel_xyz_yaw_rat",
    4: "pos_z_vel_xy",
    5: "body_pos_xyz",
    6: "body_pos_xyz_yaw_rat",
    7: "body_vel_xyz",
    8: "body_vel_xyz_yaw_rat",
    9: "body_pos_z_vel_xy",
    10: "traj",
}


def expand_path(path):
    return os.path.abspath(os.path.expanduser(path))


def list_csv_files(log_dir):
    pattern = os.path.join(expand_path(log_dir), CSV_GLOB)
    files = sorted(glob.glob(pattern), key=os.path.getmtime)
    return files


def pick_csv(log_dir, use_latest=False):
    files = list_csv_files(log_dir)
    if not files:
        raise FileNotFoundError(
            "未找到 CSV：{}（目录: {}）".format(CSV_GLOB, expand_path(log_dir))
        )

    if use_latest or len(files) == 1:
        return files[-1]

    print("可选飞行日志：")
    for i, path in enumerate(files, 1):
        print("  [{:d}] {}".format(i, path))
    print("  [Enter] 使用最新: {}".format(files[-1]))

    while True:
        raw = input("请选择编号: ").strip()
        if raw == "":
            return files[-1]
        if raw.isdigit():
            idx = int(raw)
            if 1 <= idx <= len(files):
                return files[idx - 1]
        print("输入无效，请重试。")


def _to_float(text):
    text = (text or "").strip()
    if text == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _to_int(text):
    value = _to_float(text)
    if value is None:
        return None
    return int(value)


def load_wide_csv(csv_path):
    """返回 actual 点列、带 Move_mode 的 cmd 点列，以及统计信息。"""
    actual_x, actual_y, actual_z = [], [], []
    cmd_x, cmd_y, cmd_z, cmd_modes = [], [], [], []
    move_modes = set()
    n_odom = 0
    n_cmd = 0
    last_actual = None

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        required = {
            "actual_x",
            "actual_y",
            "actual_z",
            "cmd_pos_x",
            "cmd_pos_y",
            "cmd_pos_z",
            "Move_mode",
            "event",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError("CSV 缺少列: {}".format(", ".join(sorted(missing))))

        for row in reader:
            event = (row.get("event") or "").strip()
            ax = _to_float(row.get("actual_x"))
            ay = _to_float(row.get("actual_y"))
            az = _to_float(row.get("actual_z"))
            cx = _to_float(row.get("cmd_pos_x"))
            cy = _to_float(row.get("cmd_pos_y"))
            cz = _to_float(row.get("cmd_pos_z"))
            mode = _to_int(row.get("Move_mode"))

            if mode is not None:
                move_modes.add(mode)

            # 实际轨迹：优先用 event=odom 的采样；若只有 cmd 行也带 actual，则按位置变化去重追加
            if ax is not None and ay is not None and az is not None:
                pt = (ax, ay, az)
                if event == "odom":
                    actual_x.append(ax)
                    actual_y.append(ay)
                    actual_z.append(az)
                    last_actual = pt
                    n_odom += 1
                elif last_actual is None or (
                    abs(pt[0] - last_actual[0]) > 1e-4
                    or abs(pt[1] - last_actual[1]) > 1e-4
                    or abs(pt[2] - last_actual[2]) > 1e-4
                ):
                    # cmd 行上的 actual 仅在位置变化时补充，避免 100Hz 重复点把线糊死
                    if event == "cmd" and n_odom == 0:
                        actual_x.append(ax)
                        actual_y.append(ay)
                        actual_z.append(az)
                        last_actual = pt

            if event == "cmd" and cx is not None and cy is not None and cz is not None:
                cmd_x.append(cx)
                cmd_y.append(cy)
                cmd_z.append(cz)
                cmd_modes.append(mode if mode is not None else -1)
                n_cmd += 1

    return {
        "actual": (actual_x, actual_y, actual_z),
        "cmd": (cmd_x, cmd_y, cmd_z, cmd_modes),
        "move_modes": sorted(move_modes),
        "n_odom": n_odom,
        "n_cmd": n_cmd,
    }


def _cmd_segments_by_mode(cmd_x, cmd_y, cmd_z, cmd_modes):
    """把连续相同 Move_mode 的 cmd 点切成若干段。"""
    if not cmd_x:
        return []

    segments = []
    start = 0
    cur_mode = cmd_modes[0]
    for i in range(1, len(cmd_modes)):
        if cmd_modes[i] != cur_mode:
            segments.append(
                (
                    cur_mode,
                    cmd_x[start:i],
                    cmd_y[start:i],
                    cmd_z[start:i],
                )
            )
            start = i
            cur_mode = cmd_modes[i]
    segments.append((cur_mode, cmd_x[start:], cmd_y[start:], cmd_z[start:]))
    return segments


MODE_COLORS = {
    0: "#ff7f0e",
    1: "#2ca02c",
    2: "#d62728",
    3: "#9467bd",
    4: "#8c564b",
    5: "#e377c2",
    6: "#7f7f7f",
    7: "#bcbd22",
    8: "#17becf",
    9: "#aec7e8",
    10: "#ffbb78",
}


def _mode_color(mode):
    return MODE_COLORS.get(mode, "#333333")


def _mode_label(mode):
    name = MOVE_MODE_NAMES.get(mode, "?")
    return "cmd mode={} ({})".format(mode, name)


def _axis_limits(xs, ys, zs, margin_ratio=0.08, min_span=1.0):
    vals_x = [v for v in xs if v is not None]
    vals_y = [v for v in ys if v is not None]
    vals_z = [v for v in zs if v is not None]
    if not vals_x:
        return (-1, 1), (-1, 1), (-1, 1)

    xmin, xmax = min(vals_x), max(vals_x)
    ymin, ymax = min(vals_y), max(vals_y)
    zmin, zmax = min(vals_z), max(vals_z)

    def pad(a, b):
        span = max(b - a, min_span)
        m = span * margin_ratio
        mid = 0.5 * (a + b)
        half = 0.5 * span + m
        return mid - half, mid + half

    return pad(xmin, xmax), pad(ymin, ymax), pad(zmin, zmax)


def plot_trajectories(csv_path, show_start_end=True, equal_aspect=True, mode_filter=None):
    data = load_wide_csv(csv_path)
    ax_pts = data["actual"]
    cmd_x, cmd_y, cmd_z, cmd_modes = data["cmd"]

    if mode_filter is not None:
        keep = [i for i, m in enumerate(cmd_modes) if m in mode_filter]
        cmd_x = [cmd_x[i] for i in keep]
        cmd_y = [cmd_y[i] for i in keep]
        cmd_z = [cmd_z[i] for i in keep]
        cmd_modes = [cmd_modes[i] for i in keep]

    if not ax_pts[0] and not cmd_x:
        raise RuntimeError("CSV 中没有可用的 actual / cmd 位置点: {}".format(csv_path))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    if ax_pts[0]:
        ax.plot(
            ax_pts[0],
            ax_pts[1],
            ax_pts[2],
            color="#1f77b4",
            linewidth=2.0,
            label="actual",
        )
        if show_start_end:
            ax.scatter(
                [ax_pts[0][0]],
                [ax_pts[1][0]],
                [ax_pts[2][0]],
                c="#2ca02c",
                s=60,
                marker="o",
                label="actual start",
                depthshade=False,
            )
            ax.scatter(
                [ax_pts[0][-1]],
                [ax_pts[1][-1]],
                [ax_pts[2][-1]],
                c="#d62728",
                s=60,
                marker="^",
                label="actual end",
                depthshade=False,
            )

    # 按 Move_mode 分段上色，同一 mode 只进图例一次
    labeled_modes = set()
    for mode, sx, sy, sz in _cmd_segments_by_mode(cmd_x, cmd_y, cmd_z, cmd_modes):
        if len(sx) < 1:
            continue
        label = None
        if mode not in labeled_modes:
            label = _mode_label(mode)
            labeled_modes.add(mode)
        ax.plot(
            sx,
            sy,
            sz,
            color=_mode_color(mode),
            linewidth=1.8,
            linestyle="--",
            label=label,
        )

    if cmd_x and show_start_end:
        ax.scatter(
            [cmd_x[0]],
            [cmd_y[0]],
            [cmd_z[0]],
            c="#9467bd",
            s=45,
            marker="o",
            label="cmd start",
            depthshade=False,
        )
        ax.scatter(
            [cmd_x[-1]],
            [cmd_y[-1]],
            [cmd_z[-1]],
            c="#8c564b",
            s=45,
            marker="^",
            label="cmd end",
            depthshade=False,
        )

    all_x = list(ax_pts[0]) + list(cmd_x)
    all_y = list(ax_pts[1]) + list(cmd_y)
    all_z = list(ax_pts[2]) + list(cmd_z)
    xlim, ylim, zlim = _axis_limits(all_x, all_y, all_z)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)

    if equal_aspect:
        try:
            ax.set_box_aspect(
                (
                    max(xlim[1] - xlim[0], 1e-6),
                    max(ylim[1] - ylim[0], 1e-6),
                    max(zlim[1] - zlim[0], 1e-6),
                )
            )
        except Exception:
            # 旧版 matplotlib 无 set_box_aspect
            pass

    ax.grid(True)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    shown_modes = sorted(labeled_modes) if labeled_modes else data["move_modes"]
    mode_text = ", ".join(
        "{}({})".format(m, MOVE_MODE_NAMES.get(m, "?")) for m in shown_modes
    ) or "none"
    # 图内文字用英文：当前环境常无中文字体，matplotlib 会把中文画成方框
    title = "{}\nactual pts≈{} | cmd pts={} | Move_mode: {}".format(
        os.path.basename(csv_path),
        len(ax_pts[0]),
        len(cmd_x),
        mode_text,
    )
    ax.set_title(title)
    ax.legend(loc="upper left")

    print("文件: {}".format(csv_path))
    print("  event=odom 行数: {}".format(data["n_odom"]))
    print("  event=cmd  行数: {}".format(data["n_cmd"]))
    print("  绘制 actual 点数: {}".format(len(ax_pts[0])))
    print("  绘制 cmd    点数: {}".format(len(cmd_x)))
    print("  Move_mode: {}".format(mode_text))
    for mode, sx, sy, sz in _cmd_segments_by_mode(cmd_x, cmd_y, cmd_z, cmd_modes):
        print(
            "  segment mode={} ({}): {} pts".format(
                mode, MOVE_MODE_NAMES.get(mode, "?"), len(sx)
            )
        )

    plt.tight_layout()
    plt.show()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="三维对比显示 path_recorder 宽表 CSV 中的实际轨迹与期望指令"
    )
    parser.add_argument(
        "csv",
        nargs="?",
        default=None,
        help="CSV 路径；省略则从 --dir 中选择/取最新",
    )
    parser.add_argument(
        "--dir",
        default=DEFAULT_LOG_DIR,
        help="日志目录，默认 ~/flight_logs",
    )
    parser.add_argument(
        "--latest",
        action="store_true",
        help="不交互，直接打开目录中最新的 CSV",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="列出目录中的 CSV 后退出",
    )
    parser.add_argument(
        "--no-markers",
        action="store_true",
        help="不画起终点标记",
    )
    parser.add_argument(
        "--mode",
        type=int,
        nargs="+",
        default=None,
        help="只显示指定 Move_mode，例如: --mode 10  或  --mode 1 10",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if args.list:
        files = list_csv_files(args.dir)
        if not files:
            print("目录中没有匹配的 CSV: {}".format(expand_path(args.dir)))
            return 1
        for path in files:
            print(path)
        return 0

    if args.csv:
        csv_path = expand_path(args.csv)
        if not os.path.isfile(csv_path):
            print("文件不存在: {}".format(csv_path), file=sys.stderr)
            return 1
    else:
        try:
            csv_path = pick_csv(args.dir, use_latest=args.latest)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            return 1

    try:
        plot_trajectories(
            csv_path,
            show_start_end=not args.no_markers,
            mode_filter=set(args.mode) if args.mode else None,
        )
    except Exception as exc:
        print("绘图失败: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
