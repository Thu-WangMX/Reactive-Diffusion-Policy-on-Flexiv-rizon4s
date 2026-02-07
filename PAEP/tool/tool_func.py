import json
import zarr
import numpy as np
from typing import List, Tuple, Union


def load_split_ids(split_json: str, split: str) -> List[int]:
    with open(split_json, "r", encoding="utf-8") as f:
        obj = json.load(f)
    assert split in obj, f"split must be in {list(obj.keys())}"
    return list(map(int, obj[split]))


def _open_group(zarr_path: str):
    root = zarr.open(zarr_path, mode="r")
    if isinstance(root, zarr.hierarchy.Group):
        # arrays at root
        if "external_img" in root.array_keys():
            return root, "root"
        # arrays under data/
        if "data" in root.group_keys():
            g = root["data"]
            if "external_img" in g.array_keys():
                return g, "data"
    raise KeyError("Cannot find arrays. Expect external_img at root or under group 'data'.")


def _episode_slices(episode_ends: np.ndarray) -> List[Tuple[int, int]]:
    ends = episode_ends.astype(np.int64).tolist()
    slices = []
    s = 0
    for e in ends:
        slices.append((s, e))
        s = e
    return slices


