# paep_loader.py
import importlib.util
from pathlib import Path

def load_paep_future_net():
    # 1. 确定项目根目录 (根据脚本位置调整 parents 的层级)
    repo_root = Path(__file__).resolve().parents[3] 
    paep_dir = repo_root / "PAEP"

    # 2. 定义查找路径列表 (按优先级排序)
    candidates = [
        paep_dir / "train" / "paep_model.py",
        paep_dir / "paep_train" / "paep_model.py",
        paep_dir / "paep_model.py",
        paep_dir / "models" / "paep_model.py",
    ]
    
    # 3. 暴力搜索兜底
    if paep_dir.exists():
        extra = list(paep_dir.rglob("paep_model.py"))
        for p in extra:
            if p not in candidates:
                candidates.append(p)

    # 4. 遍历尝试加载
    for p in candidates:
        if not p.exists():
            continue
        try:
            spec = importlib.util.spec_from_file_location("paep_dyn", str(p))
            if spec and spec.loader:
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                if hasattr(mod, "PAEPFutureNet"):
                    print(f"[PAEP] Successfully loaded PAEPFutureNet from: {p}")
                    return getattr(mod, "PAEPFutureNet")
        except Exception as e:
            print(f"[PAEP] Warning: Found {p} but failed to load: {e}")
            continue

    # 5. 加载失败抛出异常
    raise ModuleNotFoundError(
        f"Cannot find 'PAEPFutureNet'. Searched in:\n" + "\n".join(str(c) for c in candidates)
    )