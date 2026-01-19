import json
import zarr
import numpy as np
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--out", type=str, default="paep_split_40_5_5.json")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--shuffle", action="store_true")
    args = ap.parse_args()

    root = zarr.open_group(args.zarr_path, mode="r")
    ends = np.array(root["meta/episode_ends"][:], dtype=np.int64)
    n_ep = len(ends)
    ep_ids = np.arange(n_ep, dtype=np.int64)

    if args.shuffle:
        rng = np.random.default_rng(args.seed)
        rng.shuffle(ep_ids)

    # 40/5/5
    assert n_ep >= 50, f"need >=50 episodes, got {n_ep}"
    train = ep_ids[:40].tolist()
    val   = ep_ids[40:45].tolist()
    test  = ep_ids[45:50].tolist()

    out = {"train": train, "val": val, "test": test, "num_episodes": int(n_ep)}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    print("Wrote:", args.out)
    print("train/val/test lens:", len(train), len(val), len(test))
    print("train ids:", train[:10], "...")
    print("val ids:", val)
    print("test ids:", test)

if __name__ == "__main__":
    main()
