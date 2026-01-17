import sys
from pathlib import Path

# 把你的仓库根目录加到 PYTHONPATH
REPO_ROOT = Path("/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s")
sys.path.insert(0, str(REPO_ROOT))
"""
Estimate sampling frequency (Hz) from PKL files that store:
Root.sensorMessages: list of SensorMessage
Each SensorMessage has .timestamp (float, Unix epoch seconds)

Usage:
  python check_pkl_hz.py --pkl /path/to/seq_00016.pkl
  python check_pkl_hz.py --dir /path/to/pkl_folder --pattern "seq_*.pkl"
"""

import argparse
import pickle
from pathlib import Path
import numpy as np


def load_sensor_messages(pkl_path: Path):
    with open(pkl_path, "rb") as f:
        obj = pickle.load(f)

    # Expect: obj.sensorMessages is list-like
    if not hasattr(obj, "sensorMessages"):
        raise AttributeError(f"{pkl_path} has no attribute 'sensorMessages'")

    msgs = obj.sensorMessages
    if msgs is None or len(msgs) == 0:
        raise ValueError(f"{pkl_path} sensorMessages empty")
    # Ensure each msg has timestamp
    if not hasattr(msgs[0], "timestamp"):
        raise AttributeError(f"{pkl_path} sensorMessages[0] has no attribute 'timestamp'")

    return msgs


def estimate_stats_from_timestamps(ts: np.ndarray, max_dt: float = 1.0):
    """
    ts: 1D float array of epoch seconds (monotonic within an episode generally)
    max_dt: filter threshold to remove pauses/outliers (seconds)
    """
    ts = np.asarray(ts, dtype=np.float64)
    ts = ts[np.isfinite(ts)]
    if ts.size < 30:
        return None

    # Duration based on first/last
    duration = float(ts[-1] - ts[0])

    dt = np.diff(ts)
    dt = dt[np.isfinite(dt) & (dt > 0)]

    if dt.size < 20:
        return None

    # Remove huge gaps (pauses, dropped packets). Keep typical dt range.
    dt_f = dt[dt < max_dt]
    if dt_f.size < 20:
        # fallback: use percentile-based trimming
        hi = np.percentile(dt, 99.0)
        dt_f = dt[dt < hi]
        if dt_f.size < 20:
            return None

    med_dt = float(np.median(dt_f))
    hz = float(1.0 / med_dt)

    # Jitter info
    p10 = float(np.percentile(dt_f, 10))
    p90 = float(np.percentile(dt_f, 90))

    # How many big gaps exist (help debugging)
    gap_count = int(np.sum(dt >= max_dt))

    return {
        "N": int(ts.size),
        "duration_s": duration,
        "median_dt_s": med_dt,
        "hz": hz,
        "dt_p10_s": p10,
        "dt_p90_s": p90,
        "big_gap_count": gap_count,
        "min_ts": float(ts[0]),
        "max_ts": float(ts[-1]),
    }


def analyze_one(pkl_path: Path, max_dt: float = 1.0, verbose: bool = False):
    msgs = load_sensor_messages(pkl_path)
    ts = np.array([m.timestamp for m in msgs], dtype=np.float64)

    stats = estimate_stats_from_timestamps(ts, max_dt=max_dt)
    if stats is None:
        print(f"[WARN] {pkl_path.name}: insufficient timestamps for estimation")
        return None

    if verbose:
        print(f"\n=== {pkl_path.name} ===")
        print(f"Frames N              : {stats['N']}")
        print(f"Duration              : {stats['duration_s']:.3f} s")
        print(f"Timestamp range       : {stats['min_ts']:.6f} -> {stats['max_ts']:.6f} (epoch seconds)")
        print(f"Median dt             : {stats['median_dt_s']:.6f} s")
        print(f"Estimated freq (Hz)   : {stats['hz']:.2f}")
        print(f"dt jitter (p10,p90)   : ({stats['dt_p10_s']:.6f}, {stats['dt_p90_s']:.6f}) s")
        print(f"Big gaps (dt>={max_dt}) : {stats['big_gap_count']}")
    return stats


def analyze_dir(pkl_dir: Path, pattern: str, max_dt: float = 1.0):
    files = sorted(pkl_dir.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No files matched: {pkl_dir}/{pattern}")

    all_hz = []
    all_dt = []
    all_dur = []

    print(f"[INFO] scanning {len(files)} files in {pkl_dir} pattern={pattern}")
    for p in files:
        st = analyze_one(p, max_dt=max_dt, verbose=False)
        if st is None:
            continue
        all_hz.append(st["hz"])
        all_dt.append(st["median_dt_s"])
        all_dur.append(st["duration_s"])
        print(f"{p.name:20s}  N={st['N']:4d}  dur={st['duration_s']:6.2f}s  dt_med={st['median_dt_s']:.6f}s  hz≈{st['hz']:.2f}  gaps={st['big_gap_count']}")

    if not all_hz:
        print("[ERROR] no valid stats computed")
        return

    hz_arr = np.array(all_hz, dtype=np.float64)
    dt_arr = np.array(all_dt, dtype=np.float64)
    dur_arr = np.array(all_dur, dtype=np.float64)

    print("\n==== Summary ====")
    print(f"valid files : {len(hz_arr)} / {len(files)}")
    print(f"hz   median : {np.median(hz_arr):.2f}   p10/p90: {np.percentile(hz_arr,10):.2f}/{np.percentile(hz_arr,90):.2f}")
    print(f"dt   median : {np.median(dt_arr):.6f}s  p10/p90: {np.percentile(dt_arr,10):.6f}/{np.percentile(dt_arr,90):.6f}")
    print(f"dur  median : {np.median(dur_arr):.2f}s  p10/p90: {np.percentile(dur_arr,10):.2f}/{np.percentile(dur_arr,90):.2f}")

    # Suggest downsample ratio to ~10Hz
    raw_hz = float(np.median(hz_arr))
    ratio = max(1, int(round(raw_hz / 10.0)))
    ratio = min(ratio, 6)  # safety: avoid too aggressive downsample
    print(f"\n[SUGGEST] raw_hz≈{raw_hz:.2f} => TEMPORAL_DOWNSAMPLE_RATIO≈{ratio} (target ~10Hz)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pkl", type=str, default=None, help="path to one .pkl")
    ap.add_argument("--dir", type=str, default=None, help="directory containing .pkl files")
    ap.add_argument("--pattern", type=str, default="*.pkl", help="glob pattern inside --dir")
    ap.add_argument("--max_dt", type=float, default=1.0, help="filter dt >= max_dt as gaps (seconds)")
    ap.add_argument("--verbose", action="store_true", help="verbose for single pkl")
    args = ap.parse_args()

    if args.pkl is None and args.dir is None:
        ap.error("provide --pkl or --dir")

    if args.pkl is not None:
        p = Path(args.pkl)
        st = analyze_one(p, max_dt=args.max_dt, verbose=True if args.verbose else True)
        if st is not None:
            raw_hz = st["hz"]
            ratio = max(1, int(round(raw_hz / 10.0)))
            ratio = min(ratio, 6)
            print(f"\n[SUGGEST] TEMPORAL_DOWNSAMPLE_RATIO≈{ratio} (raw_hz≈{raw_hz:.2f}, target ~10Hz)")
    else:
        analyze_dir(Path(args.dir), args.pattern, max_dt=args.max_dt)


if __name__ == "__main__":
    main()
