#!/usr/bin/env python
"""
Quick check for left_robot_gripper_width in RDP zarr dataset.

For each episode:
- Print the first and last gripper width.
- Optionally mark episodes whose start is not ~0.5 or end is not ~0.1.
"""

import argparse
import numpy as np
import zarr


def check_gripper_width(zarr_path: str,
                        start_min: float = 0.047,
                        start_max: float = 0.052,
                        end_min: float = 0.09,
                        end_max: float = 0.11):
    # 打开 zarr（只读）
    root = zarr.open(zarr_path, mode="r")

    # 数据和元信息分组
    data_grp = root["data"]
    meta_grp = root["meta"]

    # 读取夹爪宽度和 episode_ends
    widths = np.asarray(data_grp["left_robot_gripper_width"][:])  # (N, 1) or (N,)
    widths = widths.reshape(-1)  # 展平成 (N,)

    episode_ends = np.asarray(meta_grp["episode_ends"][:], dtype=np.int64)
    num_episodes = len(episode_ends)

    print(f"Loaded {len(widths)} frames, {num_episodes} episodes.")
    print(f"First few episode_ends: {episode_ends[:10]}")

    # 计算每个 episode 的起始索引
    episode_starts = np.zeros_like(episode_ends)
    episode_starts[1:] = episode_ends[:-1]

    bad_start = []
    bad_end = []

    for ep_idx in range(num_episodes):
        s = int(episode_starts[ep_idx])
        e = int(episode_ends[ep_idx]) - 1  # inclusive index of last frame

        if e < s or e >= len(widths):
            print(f"[EP {ep_idx:03d}] invalid index range: start={s}, end={e}, skip.")
            continue

        w_start = float(widths[s])
        w_end = float(widths[e])

        # 是否在期望区间内
        ok_start = (start_min <= w_start <= start_max)
        ok_end = (end_min <= w_end <= end_max)

        flag = []
        if not ok_start:
            flag.append("BAD_START")
            bad_start.append(ep_idx)
        if not ok_end:
            flag.append("BAD_END")
            bad_end.append(ep_idx)

        flag_str = "" if not flag else "  <-- " + ",".join(flag)

        # 只打印前若干条的详细信息，避免刷屏
        if ep_idx < 20:
            print(
                f"[EP {ep_idx:03d}] len={e - s + 1:4d} | "
                f"start_idx={s:6d}, width_start={w_start:.3f} | "
                f"end_idx={e:6d}, width_end={w_end:.3f}{flag_str}"
            )

    print("\nSummary:")
    print(f"  Episodes with bad start (~0.5 expected): {len(bad_start)}")
    if bad_start:
        print(f"    indices: {bad_start}")

    print(f"  Episodes with bad end (~0.1 expected): {len(bad_end)}")
    if bad_end:
        print(f"    indices: {bad_end}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--zarr-path",
        type=str,
        required=True,
        help="Path to data.zarr (the root used by my_pkl2zarr.py)",
    )
    parser.add_argument(
        "--start-min", type=float, default=0.047,
        help="Lower bound for expected initial gripper width (~0.5)"
    )
    parser.add_argument(
        "--start-max", type=float, default=0.051,
        help="Upper bound for expected initial gripper width (~0.5)"
    )
    parser.add_argument(
        "--end-min", type=float, default=0.09,
        help="Lower bound for expected final gripper width (~0.1)"
    )
    parser.add_argument(
        "--end-max", type=float, default=0.11,
        help="Upper bound for expected final gripper width (~0.1)"
    )
    args = parser.parse_args()

    check_gripper_width(
        args.zarr_path,
        start_min=args.start_min,
        start_max=args.start_max,
        end_min=args.end_min,
        end_max=args.end_max,
    )


if __name__ == "__main__":
    main()
