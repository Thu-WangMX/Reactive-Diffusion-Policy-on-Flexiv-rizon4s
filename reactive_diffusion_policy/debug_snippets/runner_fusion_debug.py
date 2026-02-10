# reactive_diffusion_policy/debug_snippets/runner_fusion_debug.py
# Purpose:
#   - Safe fusion debug logging utilities for real-robot runner.
#   - Runner usage:
#       from reactive_diffusion_policy.debug_snippets.runner_fusion_debug import should_log, log_fusion_debug, extract_fusion_debug

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def should_log(step: int, every: int) -> bool:
    """
    Return True if we should log at this step with the given frequency.
    - every <= 0 => never log
    - step can be any int-like
    """
    try:
        e = int(every)
        s = int(step)
        return (e > 0) and (s % e == 0)
    except Exception:
        return False


def _to_float_or_list(x: Any):
    """
    Best-effort: torch scalar -> float; torch tensor -> mean float;
                numpy scalar -> float; numpy array -> list;
                python number -> float; otherwise return None.
    """
    if x is None:
        return None

    # torch
    try:
        import torch
        if torch.is_tensor(x):
            x = x.detach().cpu()
            if x.numel() == 0:
                return None
            if x.numel() == 1:
                return float(x.item())
            # for tensors, keep mean scalar (avoid huge dump)
            return float(x.float().mean().item())
    except Exception:
        pass

    # numpy
    try:
        import numpy as np
        arr = np.asarray(x)
        if arr.size == 0:
            return None
        if arr.size == 1:
            return float(arr.reshape(-1)[0])
        # arrays -> list (for small vectors)
        return arr.reshape(-1).tolist()
    except Exception:
        pass

    # python numeric
    try:
        return float(x)
    except Exception:
        return None


def _fmt(name: str, val: Optional[float], fmt: str = ".3f") -> str:
    """Format scalar debug value safely."""
    if val is None:
        return f"{name}=NA"
    try:
        v = float(val)
        if not math.isfinite(v):
            return f"{name}=NA"
        return f"{name}={v:{fmt}}"
    except Exception:
        return f"{name}=NA"


def _find_fusion_module(policy: Any) -> Any:
    """Try multiple attribute paths to locate the fusion module. Returns module or None."""
    for cand in ("fusion", "model.fusion", "net.fusion", "policy.fusion"):
        obj = policy
        ok = True
        for part in cand.split("."):
            if not hasattr(obj, part):
                ok = False
                break
            obj = getattr(obj, part)
        if ok:
            return obj
    return None


def _extract_fusion_debug_dict(policy: Any) -> Optional[Dict[str, Any]]:
    """Return fusion._last_fusion_debug dict if present, else None."""
    fusion = _find_fusion_module(policy)
    if fusion is None:
        return None
    dd = getattr(fusion, "_last_fusion_debug", None)
    return dd if isinstance(dd, dict) else None


def log_fusion_debug(
    policy: Any,
    logger: Any,
    *,
    infer_step: int,
    step_count: int,
    print_keys_once: bool = True,
    prefix: str = "[DBG][FUSION]",
) -> None:
    """
    Print a compact summary of fusion injection stats from fusion._last_fusion_debug.
    Only for console logs; never raises.
    """
    try:
        dd = _extract_fusion_debug_dict(policy)
        if dd is None:
            if not hasattr(log_fusion_debug, "_warn_no_fusion_once"):
                setattr(log_fusion_debug, "_warn_no_fusion_once", True)
                try:
                    logger.warning(f"{prefix} fusion module or _last_fusion_debug not found (warn once).")
                except Exception:
                    pass
            return

        if print_keys_once and (not hasattr(log_fusion_debug, "_printed_keys_once")):
            setattr(log_fusion_debug, "_printed_keys_once", True)
            try:
                logger.info(f"{prefix}_KEYS {sorted(list(dd.keys()))}")
            except Exception:
                pass

        # Common keys (your v4 fusion tends to have these)
        alpha = _to_float_or_list(dd.get("alpha"))
        g_contact_mean = _to_float_or_list(dd.get("g_contact_mean"))
        g_head_mean = _to_float_or_list(dd.get("g_head_mean"))
        v_norm_mean = _to_float_or_list(dd.get("v_norm_mean"))
        inj_norm_mean = _to_float_or_list(dd.get("inj_norm_mean"))
        inj_over_v_mean = _to_float_or_list(dd.get("inj_over_v_mean"))
        cos_vd_mean = _to_float_or_list(dd.get("cos_v_delta_mean"))
        delta_norm_mean = _to_float_or_list(dd.get("delta_norm_mean"))
        scale_mean = _to_float_or_list(dd.get("scale_mean"))

        msg = (
            f"infer_step={int(infer_step)} step={int(step_count)} "
            + _fmt("alpha", alpha if isinstance(alpha, float) else None, ".4f") + " "
            + _fmt("g_contact", g_contact_mean if isinstance(g_contact_mean, float) else None, ".3f") + " "
            + _fmt("g_head", g_head_mean if isinstance(g_head_mean, float) else None, ".3f") + " "
            + _fmt("v_norm", v_norm_mean if isinstance(v_norm_mean, float) else None, ".3f") + " "
            + _fmt("inj_norm", inj_norm_mean if isinstance(inj_norm_mean, float) else None, ".3f") + " "
            + _fmt("inj_over_v", inj_over_v_mean if isinstance(inj_over_v_mean, float) else None, ".3f") + " "
            + _fmt("cos_vd", cos_vd_mean if isinstance(cos_vd_mean, float) else None, ".3f") + " "
            + _fmt("delta_norm", delta_norm_mean if isinstance(delta_norm_mean, float) else None, ".3f") + " "
            + _fmt("scale", scale_mean if isinstance(scale_mean, float) else None, ".4f")
        )
        logger.info(f"{prefix} {msg}")

    except Exception as e:
        try:
            logger.warning(f"{prefix} failed: {repr(e)}")
        except Exception:
            pass


def extract_fusion_debug(policy: Any) -> Dict[str, Any]:
    """
    Extract debug caches into a JSON-serializable dict.
    - attn/* from policy.fusion.cross_attn._last_attn_debug (if exists)
    - fusion/* from policy.fusion._last_fusion_debug (if exists)
    - pol/* from policy._last_debug (if exists; e.g. p_contact_last, p_phase_last, g_contact_last)
    """
    out: Dict[str, Any] = {}

    # attention stats
    attn = getattr(getattr(policy, "fusion", None), "cross_attn", None)
    attn_rt = getattr(attn, "_last_attn_debug", None)
    if isinstance(attn_rt, dict):
        for k, v in attn_rt.items():
            out[f"attn/{k}"] = _to_float_or_list(v)

    # fusion stats
    fus = getattr(policy, "fusion", None)
    fus_rt = getattr(fus, "_last_fusion_debug", None)
    if isinstance(fus_rt, dict):
        for k, v in fus_rt.items():
            out[f"fusion/{k}"] = _to_float_or_list(v)

    # policy stash (paep last-step)
    pol_last = getattr(policy, "_last_debug", None)
    if isinstance(pol_last, dict):
        for k, v in pol_last.items():
            out[f"pol/{k}"] = _to_float_or_list(v)

    return out
