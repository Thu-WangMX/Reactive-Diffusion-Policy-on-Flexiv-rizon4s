import dill
import pickle
from pathlib import Path

pkl_path = Path("/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/"
                "OURS_v4——0128/normalizer.pkl")

# 尝试用 dill 读取（你项目里 ckpt/load 常用 dill）
with open(pkl_path, "rb") as f:
    try:
        normalizer = dill.load(f)
        loader = "dill"
    except Exception as e1:
        f.seek(0)
        try:
            normalizer = pickle.load(f)
            loader = "pickle"
        except Exception as e2:
            raise RuntimeError(f"Failed to load normalizer.pkl with dill ({e1}) and pickle ({e2})")

print(f"[OK] Loaded normalizer with {loader}: type={type(normalizer)}")

# 1) 打印它有哪些属性（方便定位 keys 存在哪里）
attrs = [a for a in dir(normalizer) if not a.startswith("_")]
print("\n[attrs]\n", attrs)

# 2) 优先尝试拿到 params_dict（你 v4 的 _has_norm_key 也依赖这个）
if hasattr(normalizer, "params_dict"):
    keys = list(normalizer.params_dict.keys())
    print("\n[params_dict keys] (count=", len(keys), ")\n", keys)

# 3) 有些版本用 _normalizers 或 params 等字段存 key
for cand in ["_normalizers", "normalizers", "params", "_params", "statistics", "_statistics"]:
    if hasattr(normalizer, cand):
        obj = getattr(normalizer, cand)
        if isinstance(obj, dict):
            print(f"\n[{cand} keys] (count={len(obj)})\n", list(obj.keys())[:200])

# 4) 尝试用 normalizer[key] 访问（如果实现了 __getitem__）
test_keys = [
    "external_img", "left_wrist_img", "right_wrist_img",
    "left_robot_tcp_pose", "left_robot_gripper_width",
    "left_robot_tcp_wrench", "wrench_hist", "action"
]
print("\n[getitem test]")
for k in test_keys:
    try:
        item = normalizer[k]
        print(f"  ✓ {k}: {type(item)}")
    except Exception as e:
        print(f"  ✗ {k}: {e}")
