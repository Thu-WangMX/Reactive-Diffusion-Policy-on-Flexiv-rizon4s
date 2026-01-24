# eval_ours_real_robot_flexiv.py
# ---------------------------------------------------------
# Real-robot inference script for OURS policy:
# - load diffusion ckpt (hydra payload or plain torch dict)
# - load PAEP ckpt into policy.paep + norm buffers
# - wrap policy to build wrench_hist online for real robot
# - run env_runner (teleop.py + camera_node_launcher.py should be started separately)
# ---------------------------------------------------------

import os
import pathlib
import psutil
import cv2
import torch
import dill
import hydra
from omegaconf import OmegaConf
from collections import deque

from reactive_diffusion_policy.workspace.base_workspace import BaseWorkspace

# ---------- perf / threads ----------
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"
cv2.setNumThreads(12)

# bind cpu affinity (optional)
total_cores = psutil.cpu_count()
num_cores_to_bind = 10
cores_to_bind = set(range(min(num_cores_to_bind, total_cores)))
try:
    os.sched_setaffinity(0, cores_to_bind)
except Exception:
    pass

OmegaConf.register_new_resolver("eval", eval, replace=True)


# ----------------- IO helpers -----------------
def _load_torch(ckpt_path: str):
    """
    Load ckpt that might be:
      - hydra payload (dill)
      - plain torch dict
      - lightning ckpt dict
    """
    with open(ckpt_path, "rb") as f:
        try:
            return torch.load(f, pickle_module=dill, map_location="cpu")
        except Exception:
            f.seek(0)
            # sometimes still needs dill; try again
            try:
                return torch.load(f, pickle_module=dill, map_location="cpu")
            except Exception:
                f.seek(0)
                return torch.load(f, map_location="cpu")


def _find_paep_module(policy: torch.nn.Module):
    """
    Our policy uses `self.paep` for PAEP module (avoid guessing).
    """
    if hasattr(policy, "paep") and isinstance(getattr(policy, "paep"), torch.nn.Module):
        return "paep", getattr(policy, "paep")
    return None, None


def _load_paep_into_policy(policy: torch.nn.Module, paep_ckpt_path: str):
    ckpt = _load_torch(paep_ckpt_path)

    if not isinstance(ckpt, dict) or "model" not in ckpt:
        raise RuntimeError(
            f"PAEP ckpt format wrong: expect dict with key 'model'. "
            f"Got type={type(ckpt)}, keys={list(ckpt.keys()) if isinstance(ckpt, dict) else None}"
        )

    attr_name, paep_mod = _find_paep_module(policy)
    if paep_mod is None:
        raise RuntimeError("policy has no attribute `.paep` (PAEP module not found).")

    # 1) load weights
    paep_mod.load_state_dict(ckpt["model"], strict=True)
    print(f"[PAEP] Loaded weights into policy.{attr_name} from {paep_ckpt_path}")

    # 2) load norm buffers if policy defines them
    norm = ckpt.get("norm", None)
    if isinstance(norm, dict):
        for buf_name, key in [
            ("_paep_wm", "wrench_mean"),
            ("_paep_ws", "wrench_std"),
            ("_paep_pm", "pose_mean"),
            ("_paep_ps", "pose_std"),
        ]:
            if hasattr(policy, buf_name) and (key in norm):
                getattr(policy, buf_name).copy_(torch.as_tensor(norm[key]).float())
        print("[PAEP] Loaded norm buffers (wrench_mean/std, pose_mean/std).")
    else:
        print("[PAEP] No norm found in ckpt (skip).")

    # 3) freeze PAEP
    paep_mod.eval()
    for p in paep_mod.parameters():
        p.requires_grad_(False)


@torch.no_grad()
def _check_model_finite(model: torch.nn.Module, max_print=10):
    bad = []
    for name, p in model.named_parameters(recurse=True):
        if p is None:
            continue
        if torch.isnan(p).any() or torch.isinf(p).any():
            bad.append(name)
            if len(bad) >= max_print:
                break
    for name, b in model.named_buffers(recurse=True):
        if b is None:
            continue
        if torch.isnan(b).any() or torch.isinf(b).any():
            bad.append(f"[buffer]{name}")
            if len(bad) >= max_print:
                break
    if bad:
        raise RuntimeError("Model contains NaN/Inf, example keys:\n" + "\n".join(bad))


def _configure_inference(policy, cfg):
    # diffusion steps
    if hasattr(policy, "num_inference_steps"):
        policy.num_inference_steps = int(getattr(cfg, "num_inference_steps", 8))

    # some policies need normalizer binding
    if hasattr(policy, "at") and hasattr(policy, "normalizer"):
        try:
            policy.at.set_normalizer(policy.normalizer)
        except Exception:
            pass

    # enable PAEP gating flags if exist
    for k in ["use_paep", "enable_paep", "paep_enabled"]:
        if hasattr(policy, k):
            setattr(policy, k, True)

    return policy


# ----------------- Online wrench_hist wrapper -----------------
class WrenchHistWrapper(torch.nn.Module):
    """
    Inject wrench_hist online for real-robot inference.

    Key idea:
    - Real obs provides left_robot_tcp_wrench: (B, To, 6), where To = n_obs_steps (often 2)
    - Training expects wrench_hist: (B, To, L, 6) with L=48 history (typically at control_fps)
    - We approximate updating history at control_fps by appending only the newest frames per inference:
        steps_per_inference = control_fps // inference_fps
      and repeating a single latest history window over To.

    Assumes B=1 for real robot.
    """

    def __init__(self, policy, wrench_key="left_robot_tcp_wrench", wrench_hist_key="wrench_hist", L=48, steps_per_inference=2):
        super().__init__()
        self.add_module("policy", policy)   # ✅ 注册到 _modules
        self.wrench_key = wrench_key
        self.wrench_hist_key = wrench_hist_key
        self.L = int(L)
        self.steps_per_inference = int(steps_per_inference)
        self.buf = deque(maxlen=self.L)

    def eval(self):
        self.policy.eval()
        return self

    def to(self, *args, **kwargs):
        self.policy.to(*args, **kwargs)
        return self

    @property
    def num_inference_steps(self):
        return self.policy.num_inference_steps

    @num_inference_steps.setter
    def num_inference_steps(self, v):
        self.policy.num_inference_steps = v

    
    def __getattr__(self, name):
        # 先让 nn.Module 自己处理（会查 _parameters/_buffers/_modules）
        try:
            return super().__getattr__(name)
        except AttributeError:
            inner = super().__getattr__("policy")   # ✅ 关键：从 _modules 里拿
            return getattr(inner, name)

    
    @property
    def device(self):
        inner = super().__getattr__("policy")
        try:
            return next(inner.parameters()).device
        except StopIteration:
            return torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
    def reset(self, *args, **kwargs):
        self.buf.clear()
        inner = super().__getattr__("policy")
        if hasattr(inner, "reset"):
            return inner.reset(*args, **kwargs)
        
    def _cast_obs_fp32(self, obs_dict):
        out = {}
        for k, v in obs_dict.items():
            if isinstance(v, torch.Tensor) and v.dtype == torch.float64:
                out[k] = v.float()
            else:
                out[k] = v
        return out

    def predict_action(self, obs_dict):
        # already has wrench_hist
        if self.wrench_hist_key in obs_dict:
            return self.policy.predict_action(obs_dict)

        if self.wrench_key not in obs_dict:
            raise KeyError(f"Missing {self.wrench_key} in obs_dict keys={list(obs_dict.keys())}")

        w_seq = obs_dict[self.wrench_key]  # (B, To, 6)
        w_seq = w_seq.to(torch.float32)
        if not (isinstance(w_seq, torch.Tensor) and w_seq.ndim == 3 and w_seq.shape[-1] == 6):
            raise RuntimeError(f"expect {self.wrench_key} as torch.Tensor (B,To,6), got {type(w_seq)} shape={getattr(w_seq,'shape',None)}")

        B, To, _ = w_seq.shape
        if B != 1:
            raise RuntimeError(f"This wrapper assumes B=1 for real robot, got B={B}")

        device = w_seq.device
        dtype = w_seq.dtype

        # ---- init buffer if empty ----
        if len(self.buf) == 0:
            wt0 = w_seq[:, -1].detach().to(torch.float32)
            for _ in range(self.L):
                self.buf.append(wt0)

        # ---- append newest frames corresponding to last r control steps ----
        r = max(1, int(self.steps_per_inference))

        # 用最新一帧 wrench 近似补齐这次推理间隔内的 r 个 control step
        wt = w_seq[:, -1].detach().to(torch.float32)  # (B,6) 这里 B=1

        for _ in range(r):
            self.buf.append(wt)

        # ---- build latest (L,6) history window ----
        stacked = torch.cat(list(self.buf), dim=0)  # (<=L,6)
        if stacked.shape[0] < self.L:
            pad = stacked[0:1].repeat(self.L - stacked.shape[0], 1)
            stacked = torch.cat([pad, stacked], dim=0)  # (L,6)

        # repeat over To: (B,To,L,6)
        wrench_hist = stacked.unsqueeze(0).unsqueeze(0).repeat(B, To, 1, 1)
        wrench_hist = wrench_hist.to(device=device, dtype=torch.float32)
        
        obs_dict = self._cast_obs_fp32(obs_dict)  
        obs_dict = dict(obs_dict)
        obs_dict[self.wrench_hist_key] = wrench_hist

        return self.policy.predict_action(obs_dict)

    def forward(self, *args, **kwargs):
        return self.policy(*args, **kwargs)


# ----------------- Hydra entry -----------------
@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath("reactive_diffusion_policy", "config")),
    # NOTE: override this via CLI using --config-name train_ours_diffusion_unet_image_force_workspace
    config_name="train_diffusion_unet_real_image_workspace",
)
def main(cfg):
    """
    Required CLI overrides:
      +diff_ckpt_path=...
      +paep_ckpt_path=...

    Optional:
      +num_inference_steps=8

    Example:
    python eval_ours_real_robot_flexiv.py \
      --config-name train_ours_diffusion_unet_image_force_workspace \
      task=wmx_paep_real_wiping_board_image_dp_absolute_24fps \
      +task.env_runner.output_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video \
      +diff_ckpt_path=/path/to/diff.ckpt \
      +paep_ckpt_path=/path/to/paep.pt \
      +num_inference_steps=8
    """
    diff_ckpt_path = cfg.diff_ckpt_path
    paep_ckpt_path = cfg.paep_ckpt_path

    # 1) load diffusion payload + workspace
    payload = _load_torch(diff_ckpt_path)

    ws_cls = hydra.utils.get_class(cfg._target_)
    workspace: BaseWorkspace = ws_cls(cfg)
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    # 2) pick policy (ema if exists & enabled)
    policy = getattr(workspace, "model", None)
    if policy is None:
        raise RuntimeError("workspace.model not found")

    use_ema = False
    try:
        # cfg.training may be DictConfig
        use_ema = bool(getattr(cfg, "training", {}).get("use_ema", False))
    except Exception:
        pass

    if use_ema and hasattr(workspace, "ema_model") and (workspace.ema_model is not None):
        policy = workspace.ema_model
        print("[Eval] Using EMA model.")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy.eval().to(device)

    # 3) load PAEP into policy
    _load_paep_into_policy(policy, paep_ckpt_path)

    # 4) sanity checks & inference config
    _check_model_finite(policy)
    policy = _configure_inference(policy, cfg)

    # 5) wrap policy to inject wrench_hist online
    control_fps = int(getattr(cfg.task.env_runner, "control_fps", 24))
    inference_fps = int(getattr(cfg.task.env_runner, "inference_fps", 12))
    if control_fps % inference_fps != 0:
        raise RuntimeError(f"control_fps ({control_fps}) must be divisible by inference_fps ({inference_fps})")

    r = int(control_fps // inference_fps)
    policy = WrenchHistWrapper(
        policy,
        wrench_key="left_robot_tcp_wrench",
        wrench_hist_key="wrench_hist",
        L=48,
        steps_per_inference=r,
    )
    print(f"[Eval] control_fps={control_fps}, inference_fps={inference_fps}, r={r}")
    print(f"[Eval] num_inference_steps={getattr(policy, 'num_inference_steps', None)}")
    print(f"[Eval] device={device}")

    # 6) run real robot env_runner
    env_runner = hydra.utils.instantiate(cfg.task.env_runner)
    env_runner.run(policy)


if __name__ == "__main__":
    main()
