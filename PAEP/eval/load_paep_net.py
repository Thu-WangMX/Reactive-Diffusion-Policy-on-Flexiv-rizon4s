import importlib.util
import os
import sys
from pathlib import Path
from contextlib import contextmanager
from typing import Optional


@contextmanager
def _temp_sys_path(paths):
    """临时把若干路径加入 sys.path，退出后还原（避免污染全局）"""
    old = list(sys.path)
    try:
        for p in paths:
            p = str(p)
            if p not in sys.path:
                sys.path.insert(0, p)
        yield
    finally:
        sys.path[:] = old


def _find_paep_root_from_here() -> Optional[Path]:
    """从当前文件位置（PAEP/eval/load_paep_net.py）向上找 PAEP 根目录"""
    here = Path(__file__).resolve()
    # .../PAEP/eval/load_paep_net.py -> .../PAEP
    paep_root = here.parent.parent
    if (paep_root / "eval").is_dir():
        return paep_root
    return None


def _find_repo_root_from_ckpt(ckpt_path: str) -> Optional[Path]:
    """
    从 ckpt 路径推断 repo 根目录（包含 PAEP/ 的那个根目录）。
    兼容传入的是：
      - .../PAEP/ckpt/.../best.pt  (文件)
      - .../PAEP/ckpt/...          (目录)
    """
    if not ckpt_path:
        return None

    p = Path(ckpt_path).resolve()
    # 如果是文件（best.pt），先取父目录
    if p.is_file():
        p = p.parent

    # 向上爬，寻找包含 "PAEP" 文件夹的 repo 根目录
    # repo_root/PAEP/...
    for parent in [p] + list(p.parents):
        if (parent / "PAEP").is_dir():
            return parent

    return None


def _import_symbol_from_py(py_path: Path, symbol: str):
    spec = importlib.util.spec_from_file_location("paep_dyn", str(py_path))
    if spec is None or spec.loader is None:
        raise ModuleNotFoundError(f"Cannot import module from: {py_path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if not hasattr(mod, symbol):
        raise ModuleNotFoundError(f"File found but class '{symbol}' is missing in: {py_path}")
    return getattr(mod, symbol)


def load_paep_future_net(
    paep_ckpt_path: Optional[str] = None,
    paep_code_root: Optional[str] = None,
):
    """
    兼容训练/评估的 PAEPFutureNet loader。

    参数：
      - paep_ckpt_path: 你 hydra 里传的 +paep_ckpt_path=.../best.pt（可选）
        注意：它只用于“推断 repo root”，不会被当作代码目录拼接！
      - paep_code_root: 如果你想显式指定 PAEP 代码目录（可选），传 .../repo_root/PAEP
    """

    # 1) 先确定 PAEP 根目录（包含 train/, eval/ 的那个 PAEP 文件夹）
    candidates_paep_root = []

    if paep_code_root:
        candidates_paep_root.append(Path(paep_code_root).resolve())

    # 从当前文件位置推断（最稳定，训练/评估都适用）
    paep_root_here = _find_paep_root_from_here()
    if paep_root_here:
        candidates_paep_root.append(paep_root_here)

    # 从 ckpt 路径推断 repo_root，再 repo_root/PAEP
    repo_root = _find_repo_root_from_ckpt(paep_ckpt_path) if paep_ckpt_path else None
    if repo_root:
        candidates_paep_root.append((repo_root / "PAEP").resolve())

    # 去重
    uniq_paep_roots = []
    for r in candidates_paep_root:
        if r not in uniq_paep_roots:
            uniq_paep_roots.append(r)

    # 2) 在 PAEP 根目录下找 paep_model.py（兼容两种常见结构）
    # 你之前出现过 PAEP/train/paep_model.py 和 PAEP/paep_train/paep_model.py
    searched = []
    for paep_root in uniq_paep_roots:
        model_candidates = [
            paep_root / "train" / "paep_model.py",
            paep_root / "paep_train" / "paep_model.py",
        ]
        for py in model_candidates:
            searched.append(py)

            print(f"[PAEP] Looking for model file at: {py}")
            if not py.exists():
                print(f"[PAEP] File not found: {py}")
                continue

            # 3) 为了让 paep_model.py 内部的 “from PAEP.xxx import ...” 成功：
            #    必须把 repo_root 加入 sys.path（即包含 PAEP 文件夹的那个目录）
            #    如果我们只有 paep_root（.../PAEP），则 repo_root = paep_root.parent
            repo_root_guess = paep_root.parent

            with _temp_sys_path([repo_root_guess, paep_root, py.parent]):
                try:
                    cls = _import_symbol_from_py(py, "PAEPFutureNet")
                    print(f"[PAEP] Successfully loaded PAEPFutureNet from: {py}")
                    return cls
                except Exception as e:
                    print(f"[PAEP] Warning: Found {py} but failed to load: {e}")
                    import traceback
                    traceback.print_exc()
                    continue

    raise ModuleNotFoundError(
        "Cannot find 'PAEPFutureNet'. Searched in:\n" +
        "\n".join(str(p) for p in searched)
    )
