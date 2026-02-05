#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_deploy_runtime.py (Route A)  ✅ deadband + raw/norm err + HOLD

Train/deploy contract:
- wrench_mean/std computed on RAW wrench (train split)
- runtime: w_norm = (w_raw-mean)/std
- optional: w_norm_z_in = w_norm_z - (fz_target/std_z)   (same as training)
- deadband in RAW space:
    fz_err_raw = Fz - fz_target
    if |fz_err_raw| < fz_deadband: fz_err = 0  => HOLD
    else: fz_err = fz_err_raw
- rule check:
    fz_err > 0 => PRESS_DOWN => dz_ee > 0
    fz_err < 0 => LIFT_UP    => dz_ee < 0
"""

from __future__ import annotations

import os
import json
import importlib.util
from dataclasses import dataclass
from typing import Optional, Sequence, Dict, Any, Tuple

import numpy as np
import torch
from collections import deque


def rot6d_to_matrix(rot6d: np.ndarray) -> np.ndarray:
    x = np.asarray(rot6d, dtype=np.float32)
    orig_shape = x.shape[:-1]
    x = x.reshape(-1, 6)

    a1 = x[:, 0:3]
    a2 = x[:, 3:6]

    b1 = a1 / (np.linalg.norm(a1, axis=1, keepdims=True) + 1e-8)
    dot12 = np.sum(b1 * a2, axis=1, keepdims=True)
    b2 = a2 - dot12 * b1
    b2 = b2 / (np.linalg.norm(b2, axis=1, keepdims=True) + 1e-8)
    b3 = np.cross(b1, b2)

    R = np.stack([b1, b2, b3], axis=-1)
    return R.reshape(*orig_shape, 3, 3)


def matrix_to_rot6d(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=np.float32)
    c1 = R[..., :, 0]
    c2 = R[..., :, 1]
    return np.concatenate([c1, c2], axis=-1)


def _normalize_prob(p: np.ndarray) -> np.ndarray:
    p = np.asarray(p, dtype=np.float32).reshape(-1)
    s = float(np.sum(p))
    if s > 1e-8:
        return (p / s).astype(np.float32)
    return p.astype(np.float32)


@dataclass
class FastGateConfig:
    mode: str = "soft"          # "soft" | "hard"
    thr: float = 0.5            # for hard gate
    use_contact: bool = True
    active_phase_ids: Optional[Sequence[int]] = None


class FastDeployer:
    def __init__(
        self,
        ckpt_path: str,
        meta_path: str,
        device: str = "cuda",
        hist_len: Optional[int] = None,
        wrench_clip_norm: float = 6.0,
        max_delta_xyz: float = 0.003,
        gate: Optional[FastGateConfig] = None,
        apply_rot: bool = False,
        verbose: bool = False,
    ):
        self.ckpt_path = str(ckpt_path)
        self.meta_path = str(meta_path)
        self.device = device if (device is not None and device != "") else ("cuda" if torch.cuda.is_available() else "cpu")
        self.wrench_clip_norm = float(wrench_clip_norm)
        self.max_delta_xyz = float(max_delta_xyz)
        self.apply_rot = bool(apply_rot)
        self.verbose = bool(verbose)
        self.gate = gate if gate is not None else FastGateConfig()

        if not os.path.exists(self.meta_path):
            raise FileNotFoundError(f"[FAST] meta not found: {self.meta_path}")
        if not os.path.exists(self.ckpt_path):
            raise FileNotFoundError(f"[FAST] ckpt not found: {self.ckpt_path}")

        with open(self.meta_path, "r") as f:
            self.meta: Dict[str, Any] = json.load(f)

        self.hist = int(hist_len if hist_len is not None else self.meta.get("HIST", self.meta.get("hist", 8)))
        self.d_base = int(self.meta.get("d_base", 9))
        self.d_paep = int(self.meta.get("d_paep", 4))
        self.phase_k = max(0, self.d_paep - 1)

        # wrench stats
        wn = self.meta.get("wrench_norm", None)
        if not (isinstance(wn, dict) and ("mean" in wn) and ("std" in wn)):
            raise RuntimeError("[FAST] meta missing wrench_norm{mean,std}")
        self.wrench_mean = np.asarray(wn["mean"], dtype=np.float32).reshape(-1)[:6]
        self.wrench_std = np.maximum(np.asarray(wn["std"], dtype=np.float32).reshape(-1)[:6], 1e-6)

        # force target + deadband
        self.fz_target = float(self.meta.get("fz_target", -20.0))
        self.fz_deadband = float(self.meta.get("fz_deadband", 0.5))

        # shift config
        self.apply_fz_target_shift_norm = bool(self.meta.get("apply_fz_target_shift_norm", False))
        self.fz_target_shift_norm_z = self.meta.get("fz_target_shift_norm_z", None)
        if self.fz_target_shift_norm_z is None:
            self.fz_target_shift_norm_z = float(self.fz_target) / float(self.wrench_std[2])

        # load model from fast_model.py next to this file
        this_dir = os.path.dirname(os.path.abspath(__file__))
        model_py = os.path.join(this_dir, "fast_model.py")
        mod = self._import_module(model_py, module_name="fast_model_runtime")
        self.model = mod.FastResidualPolicy(d_base=self.d_base, d_paep=self.d_paep).to(self.device)
        self.model.eval()

        ckpt = torch.load(self.ckpt_path, map_location=self.device)
        sd = ckpt["model"] if (isinstance(ckpt, dict) and "model" in ckpt) else ckpt
        self.model.load_state_dict(sd, strict=True)

        self.buf_base = deque(maxlen=self.hist)
        self.buf_w = deque(maxlen=self.hist)
        self.buf_p = deque(maxlen=self.hist)

        if self.verbose:
            print(f"[FAST] loaded ckpt={self.ckpt_path}")
            print(f"[FAST] meta={self.meta_path} hist={self.hist} d_base={self.d_base} d_paep={self.d_paep}")
            print(f"[FAST] fz_target={self.fz_target} deadband={self.fz_deadband}")
            print(f"[FAST] apply_shift={self.apply_fz_target_shift_norm} shift_norm_z={self.fz_target_shift_norm_z}")

    @staticmethod
    def _import_module(py_path: str, module_name: str):
        spec = importlib.util.spec_from_file_location(module_name, py_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"[FAST] failed to import: {py_path}")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)  # type: ignore
        return mod

    def reset(self):
        self.buf_base.clear()
        self.buf_w.clear()
        self.buf_p.clear()

    def _norm_clip_wrench(self, wrench6: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray, np.ndarray]:
        w_raw = np.asarray(wrench6, dtype=np.float32).reshape(6).copy()
        w_norm = (w_raw - self.wrench_mean) / self.wrench_std

        # optional shift in normalized space
        if self.apply_fz_target_shift_norm and (self.fz_target_shift_norm_z is not None):
            w_norm[2] = w_norm[2] - float(self.fz_target_shift_norm_z)

        preclip = w_norm.copy()
        w_norm = np.clip(w_norm, -self.wrench_clip_norm, self.wrench_clip_norm).astype(np.float32)
        clip_ratio = float(np.mean(np.abs(preclip) > self.wrench_clip_norm))
        return w_norm, clip_ratio, w_raw.astype(np.float32), preclip.astype(np.float32)

    def _build_paep_feat(self, p_contact: Optional[float], p_phase: Optional[np.ndarray]) -> Tuple[np.ndarray, float, np.ndarray]:
        pc = float(p_contact) if (p_contact is not None and np.isfinite(p_contact)) else 0.0
        pc = float(np.clip(pc, 0.0, 1.0))

        if p_phase is None:
            pp = np.zeros((self.phase_k if self.phase_k > 0 else max(1, self.d_paep - 1)), dtype=np.float32)
        else:
            pp = np.asarray(p_phase, dtype=np.float32).reshape(-1)
        pp = _normalize_prob(pp)

        if self.phase_k > 0:
            K = self.phase_k
            if pp.size != K:
                tmp = np.zeros((K,), dtype=np.float32)
                tmp[: min(K, pp.size)] = pp[: min(K, pp.size)]
                pp = tmp
            feat = np.concatenate([np.array([pc], dtype=np.float32), pp], axis=0)
        else:
            K = self.d_paep
            if pp.size != K:
                tmp = np.zeros((K,), dtype=np.float32)
                tmp[: min(K, pp.size)] = pp[: min(K, pp.size)]
                pp = tmp
            feat = pp.astype(np.float32)

        if feat.size != self.d_paep:
            tmp = np.zeros((self.d_paep,), dtype=np.float32)
            tmp[: min(self.d_paep, feat.size)] = feat[: min(self.d_paep, feat.size)]
            feat = tmp
        return feat.astype(np.float32), pc, pp.astype(np.float32)

    def _gate_alpha(self, pc: float, p_phase: np.ndarray) -> float:
        if self.gate.active_phase_ids is None:
            active_score = float(np.sum(p_phase))
        else:
            ids = [int(i) for i in self.gate.active_phase_ids]
            ids = [i for i in ids if 0 <= i < p_phase.size]
            active_score = float(np.sum(p_phase[ids])) if len(ids) > 0 else 0.0

        if self.gate.mode.lower() == "hard":
            phase_id = int(np.argmax(p_phase)) if p_phase.size > 0 else 0
            cond_phase = (self.gate.active_phase_ids is None) or (phase_id in set(self.gate.active_phase_ids))
            cond_contact = (pc >= float(self.gate.thr)) if self.gate.use_contact else True
            return 1.0 if (cond_phase and cond_contact) else 0.0

        a = active_score
        if self.gate.use_contact:
            a = a * float(pc)
        return float(np.clip(a, 0.0, 1.0))

    def _stack_hist(self, buf: deque, dim: int) -> np.ndarray:
        n = len(buf)
        if n <= 0:
            return np.zeros((self.hist, dim), dtype=np.float32)
        xs = list(buf)
        if n < self.hist:
            pad = [xs[0]] * (self.hist - n)
            xs = pad + xs
        x = np.stack(xs[-self.hist:], axis=0).astype(np.float32)
        return x

    @torch.no_grad()
    def step(
        self,
        base_abs9d: np.ndarray,
        wrench6: np.ndarray,
        p_contact: Optional[float] = None,
        p_phase: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        base_abs9d = np.asarray(base_abs9d, dtype=np.float32).reshape(-1)
        if base_abs9d.size < self.d_base:
            tmp = np.zeros((self.d_base,), dtype=np.float32)
            tmp[: base_abs9d.size] = base_abs9d
            base_abs9d = tmp
        else:
            base_abs9d = base_abs9d[: self.d_base]

        w_norm, clip_ratio_w, w_raw, w_norm_preclip = self._norm_clip_wrench(wrench6)
        paep_feat, pc, pp = self._build_paep_feat(p_contact, p_phase)

        # errors (raw + deadbanded + norm)
        fz_raw = float(w_raw[2])
        fz_err_raw = float(fz_raw - self.fz_target)
        fz_err = 0.0 if abs(fz_err_raw) < self.fz_deadband else fz_err_raw
        fz_err_norm = float(fz_err / float(self.wrench_std[2]))

        # update buffers
        self.buf_base.append(base_abs9d.copy())
        self.buf_w.append(w_norm.copy())
        self.buf_p.append(paep_feat.copy())

        base_h = self._stack_hist(self.buf_base, self.d_base)
        w_h = self._stack_hist(self.buf_w, 6)
        p_h = self._stack_hist(self.buf_p, self.d_paep)

        alpha = self._gate_alpha(pc, pp)

        tb = torch.from_numpy(base_h[None]).to(self.device)
        tw = torch.from_numpy(w_h[None]).to(self.device)
        tp = torch.from_numpy(p_h[None]).to(self.device)

        pred, aux = self.model(tb, tw, tp)
        pred9 = pred.detach().float().cpu().numpy().reshape(-1)[:9].astype(np.float32)

        delta = (alpha * pred9).astype(np.float32)
        delta_p_ee_preclip = delta[:3].copy()
        delta_p_ee = np.clip(delta_p_ee_preclip, -self.max_delta_xyz, self.max_delta_xyz)

        base_pos = base_abs9d[:3].astype(np.float32)
        base_rot6 = base_abs9d[3:9].astype(np.float32)
        Rb = rot6d_to_matrix(base_rot6)
        delta_p_base = Rb.dot(delta_p_ee)

        cmd = base_abs9d.copy()
        cmd[:3] = base_pos + delta_p_base

        if self.apply_rot:
            dR = rot6d_to_matrix(delta[3:9].astype(np.float32))
            cmd[3:9] = matrix_to_rot6d(Rb.dot(dR)).astype(np.float32)
        else:
            cmd[3:9] = base_rot6

        # iron-law check (deadband aware)
        dz = float(delta_p_ee[2])
        if fz_err == 0.0:
            expected = "HOLD"
            ok = True
        elif fz_err > 0:
            expected = "PRESS_DOWN"
            ok = (dz > 0)
        else:
            expected = "LIFT_UP"
            ok = (dz < 0)

        dbg = {
            "alpha": float(alpha),
            "pc": float(pc),
            "phase_id": int(np.argmax(pp)) if pp.size > 0 else 0,

            "wrench_raw": w_raw.tolist(),
            "wrench_norm_preclip": w_norm_preclip.tolist(),
            "wrench_normed": w_norm.tolist(),
            "clip_ratio_wrench": float(clip_ratio_w),

            "fz_target": float(self.fz_target),
            "fz_deadband": float(self.fz_deadband),
            "fz_raw": float(fz_raw),
            "fz_err_raw": float(fz_err_raw),
            "fz_err": float(fz_err),               # deadbanded
            "fz_err_norm": float(fz_err_norm),     # deadbanded/std

            "apply_fz_target_shift_norm": bool(self.apply_fz_target_shift_norm),
            "fz_target_shift_norm_z": float(self.fz_target_shift_norm_z),

            "delta_pred9": pred9.tolist(),
            "delta_applied9": delta.tolist(),
            "delta_p_ee_preclip": delta_p_ee_preclip.tolist(),
            "delta_p_ee": delta_p_ee.tolist(),
            "delta_p_base": delta_p_base.tolist(),

            "fast_rule_expected": expected,
            "fast_rule_ok": bool(ok),
        }

        if isinstance(aux, dict) and "gate" in aux:
            try:
                g = aux["gate"].detach().float().cpu().numpy().reshape(-1)
                dbg["model_gate"] = g.tolist()
            except Exception:
                pass

        return cmd.astype(np.float32), dbg
