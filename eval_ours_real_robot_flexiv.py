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


# ----------------- Online wrench_hist wrapper (FIXED) -----------------
class WrenchHistWrapper(torch.nn.Module):
    """
    Build wrench_hist (B,To,L,6) online with correct per-frame end alignment.

    - Maintain an internal dense wrench buffer (approx control_fps).
    - For each obs frame i (0..To-1), build a history window ending at:
        end = latest_dense_idx - (To-1-i)*obs_ratio
      which matches the dataset logic when obs is downsampled by obs_ratio.
    """

    def __init__(
        self,
        policy,
        wrench_key="left_robot_tcp_wrench",
        wrench_hist_key="wrench_hist",
        L=48,
        steps_per_inference=2,          # r = control_fps // inference_fps
        obs_ratio=2,                    # obs_temporal_downsample_ratio
    ):
        super().__init__()
        self.add_module("policy", policy)
        self.wrench_key = wrench_key
        self.wrench_hist_key = wrench_hist_key
        self.L = int(L)
        self.r = int(steps_per_inference)
        self.obs_ratio = int(obs_ratio)

        # IMPORTANT: need a bit MORE than L to support earlier end points when To>1
        # (e.g., To=2, obs_ratio=2 => earliest end is 2 steps before latest end)
        extra = max(8, (self.obs_ratio * 4) + self.r + 4)
        self.buf = deque(maxlen=self.L + extra)  # store (B,6) tensors, B is assumed 1

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
        try:
            return super().__getattr__(name)
        except AttributeError:
            inner = super().__getattr__("policy")
            return getattr(inner, name)

    def _cast_obs_fp32(self, obs_dict):
        out = {}
        for k, v in obs_dict.items():
            if isinstance(v, torch.Tensor) and v.dtype == torch.float64:
                out[k] = v.float()
            else:
                out[k] = v
        return out

    def reset(self, *args, **kwargs):
        self.buf.clear()
        inner = super().__getattr__("policy")
        if hasattr(inner, "reset"):
            return inner.reset(*args, **kwargs)

    def _append_dense_samples(self, w_seq: torch.Tensor):
        """
        Append ~r dense wrench samples to buffer.

        We use last two obs frames (if available) to interpolate missing control steps.
        Typical setting: control_fps=24, inference_fps=8 => r=3; obs_ratio=2; To=2
        obs provides w(t-2) and w(t). We approximate:
          [w(t-2), mid(t-1), w(t)]  (3 samples)  -> matches r=3
        """
        B, To, _ = w_seq.shape
        assert B == 1

        w_cur = w_seq[:, -1].detach().to(torch.float32)  # (1,6)

        # init buffer if empty
        if len(self.buf) == 0:
            for _ in range(self.L):
                self.buf.append(w_cur)

        # if we have previous obs frame, interpolate
        if To >= 2 and self.obs_ratio >= 1:
            w_prev = w_seq[:, -2].detach().to(torch.float32)  # (1,6)

            # build samples that cover [t-obs_ratio .. t] with obs_ratio steps
            # interp points: j=1..obs_ratio => includes w_cur at the end
            interp = []
            for j in range(1, self.obs_ratio + 1):
                alpha = float(j) / float(self.obs_ratio)
                interp.append(w_prev + (w_cur - w_prev) * alpha)  # (1,6)

            # candidate dense block: include w_prev then interpolated steps to w_cur
            dense = [w_prev] + interp  # length = obs_ratio+1

            # we only need to append r samples for "since last inference"
            if self.r <= len(dense):
                dense_to_add = dense[-self.r:]
            else:
                # not enough: pad by repeating earliest
                pad_n = self.r - len(dense)
                dense_to_add = [dense[0]] * pad_n + dense

            for x in dense_to_add:
                self.buf.append(x)
        else:
            # fallback: repeat current
            for _ in range(max(1, self.r)):
                self.buf.append(w_cur)

    def _build_hist_per_frame(self, To: int, device: torch.device):
        """
        Build (B,To,L,6) with different end points per obs frame.
        """
        # ensure enough length (pad left with first)
        if len(self.buf) < self.L + (To - 1) * self.obs_ratio + 1:
            first = self.buf[0]
            need = (self.L + (To - 1) * self.obs_ratio + 1) - len(self.buf)
            for _ in range(need):
                self.buf.appendleft(first)

        buf_list = list(self.buf)  # list of (1,6)
        N = len(buf_list)
        B = 1

        hists = []
        for i in range(To):
            # i=To-1 is latest obs frame => end_offset=0
            end_offset = (To - 1 - i) * self.obs_ratio
            end_idx = (N - 1) - end_offset
            start_idx = end_idx - (self.L - 1)

            if start_idx < 0:
                pad = [buf_list[0]] * (-start_idx)
                seg = pad + buf_list[0:end_idx + 1]
            else:
                seg = buf_list[start_idx:end_idx + 1]

            # seg length should be L
            if len(seg) != self.L:
                if len(seg) < self.L:
                    seg = [seg[0]] * (self.L - len(seg)) + seg
                else:
                    seg = seg[-self.L:]

            stacked = torch.cat(seg, dim=0)          # (L,6) because each item is (1,6)
            hists.append(stacked.unsqueeze(0))       # (1,L,6)

        wrench_hist = torch.stack(hists, dim=1)      # (1,To,L,6)
        return wrench_hist.to(device=device, dtype=torch.float32)

    def predict_action(self, obs_dict):
        # already has wrench_hist
        obs_dict = self._cast_obs_fp32(obs_dict)
        if self.wrench_hist_key in obs_dict:
            return self.policy.predict_action(obs_dict)

        if self.wrench_key not in obs_dict:
            raise KeyError(f"Missing {self.wrench_key} in obs_dict keys={list(obs_dict.keys())}")

        w_seq = obs_dict[self.wrench_key]  # (B, To, 6)
        if not (isinstance(w_seq, torch.Tensor) and w_seq.ndim == 3 and w_seq.shape[-1] == 6):
            raise RuntimeError(f"expect {self.wrench_key} as torch.Tensor (B,To,6), got {type(w_seq)} shape={getattr(w_seq,'shape',None)}")

        w_seq = w_seq.to(torch.float32)
        B, To, _ = w_seq.shape
        if B != 1:
            raise RuntimeError(f"This wrapper assumes B=1 for real robot, got B={B}")

        device = w_seq.device

        # 1) update dense buffer
        self._append_dense_samples(w_seq)

        # 2) build per-frame history windows (NOT repeated)
        wrench_hist = self._build_hist_per_frame(To=To, device=device)

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

        # ✅ 关键：确保 EMA policy 也带上 normalizer
        if hasattr(workspace, "model") and hasattr(workspace.model, "normalizer"):
            try:
                policy.set_normalizer(workspace.model.normalizer)
            except Exception:
                policy.normalizer = workspace.model.normalizer

        print("[Eval] Using EMA model (normalizer synced).")


    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy.eval().to(device)



    # ✅ 启动阶段就判定：TCN 是否会吃到标准化 wrench
    has_norm = policy._has_norm_key("left_robot_tcp_wrench")
    norm_keys = list(getattr(policy.normalizer, "params_dict", {}).keys()) if hasattr(policy, "normalizer") else []
    print("has_norm_wrench =", has_norm)
    print("norm_keys =", norm_keys)

    assert has_norm, (
        "Normalizer missing key 'left_robot_tcp_wrench' -> "
        "policy_v4 TCN will use RAW wrench (distribution mismatch). "
        "Fix: ensure ckpt payload loads normalizer into policy, or sync normalizer to EMA model."
    )


    # 3) load PAEP into policy
    _load_paep_into_policy(policy, paep_ckpt_path)

    # 4) sanity checks & inference config
    _check_model_finite(policy)
    policy = _configure_inference(policy, cfg)

    # 5) wrap policy to inject wrench_hist online
    control_fps = int(getattr(cfg.task.env_runner, "control_fps", 24))
    inference_fps = int(getattr(cfg.task.env_runner, "inference_fps", 8))
    if control_fps % inference_fps != 0:
        raise RuntimeError(f"control_fps ({control_fps}) must be divisible by inference_fps ({inference_fps})")

    
    r = int(control_fps // inference_fps)
    obs_ratio = int(getattr(cfg.task.env_runner, "obs_temporal_downsample_ratio", 2))

    policy = WrenchHistWrapper(
        policy,
        wrench_key="left_robot_tcp_wrench",
        wrench_hist_key="wrench_hist",
        L=48,
        steps_per_inference=r,
        obs_ratio=obs_ratio,
    )
    print(f"[Eval] control_fps={control_fps}, inference_fps={inference_fps}, r={r}, obs_ratio={obs_ratio}")

    print(f"[Eval] num_inference_steps={getattr(policy, 'num_inference_steps', None)}")
    print(f"[Eval] device={device}")

    # 6) run real robot env_runner
    env_runner = hydra.utils.instantiate(cfg.task.env_runner)
    env_runner.run(policy)


if __name__ == "__main__":
    main()
