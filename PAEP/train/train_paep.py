# train_plug_in_charger.py
import os
import time
from typing import Dict, List
import argparse
import importlib.util
from types import SimpleNamespace

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
from tqdm import tqdm

from paep_dataset import PAEPFutureDataset
from paep_model import PAEPFutureNet

# ==========================================
# 🚀 【性能优化】开启 RTX 4090 硬件加速
# ==========================================
torch.backends.cudnn.benchmark = True          # 针对固定尺寸输入自动寻找最快算法
torch.backends.cuda.matmul.allow_tf32 = True   # 允许矩阵乘法使用 TF32
torch.backends.cudnn.allow_tf32 = True         # 允许卷积使用 TF32

# 默认配置文件路径
CFG_PATH = "/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/config/paep_plug_in_usb.py"

def atomic_torch_save(obj, path: str):
    """原子保存，防止中断导致文件损坏"""
    tmp = path + ".tmp"
    torch.save(obj, tmp)
    os.replace(tmp, path)

def _state_dict_to_cpu(state_dict):
    """将模型参数移至 CPU"""
    cpu_sd = {}
    for k, v in state_dict.items():
        if torch.is_tensor(v):
            cpu_sd[k] = v.detach().cpu()
        else:
            cpu_sd[k] = v
    return cpu_sd

def _state_dict_to_cpu(state_dict):
    """将模型参数移至 CPU，并自动移除 torch.compile 带来的 _orig_mod. 前缀"""
    cpu_sd = {}
    for k, v in state_dict.items():
        # 清洗 torch.compile 带来的前缀
        clean_key = k.replace("_orig_mod.", "") if k.startswith("_orig_mod.") else k
        
        if torch.is_tensor(v):
            cpu_sd[clean_key] = v.detach().cpu()
        else:
            cpu_sd[clean_key] = v
    return cpu_sd

def _move_ckpt_tensors_to_cpu(obj):
    """递归将 Checkpoint 中的 Tensor 移至 CPU"""
    if torch.is_tensor(obj):
        return obj.detach().cpu()
    if isinstance(obj, dict):
        return {k: _move_ckpt_tensors_to_cpu(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        t = [_move_ckpt_tensors_to_cpu(v) for v in obj]
        return type(obj)(t)
    return obj

def load_cfg(py_path: str):
    """动态加载 Python 配置文件"""
    py_path = os.path.abspath(py_path)
    if not os.path.exists(py_path):
        raise FileNotFoundError(f"[PAEP] Config file not found: {py_path}")
    spec = importlib.util.spec_from_file_location("paep_cfg", py_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    d = {k: getattr(mod, k) for k in dir(mod) if k.isupper()}
    return SimpleNamespace(**d)

def compute_class_weights(class_counts: Dict[str, int], phase_names: List[str]) -> torch.Tensor:
    counts = np.array([class_counts.get(n, 0) for n in phase_names], dtype=np.float64)
    counts = np.maximum(counts, 1.0)
    inv = 1.0 / counts
    w = inv / inv.sum() * len(inv)
    return torch.tensor(w, dtype=torch.float32)

@torch.no_grad()
def eval_n(model, loader, device, norm_torch, img_size: int, phase_w: torch.Tensor):
    """验证函数"""
    model.eval()
    total_n = 0
    loss_p_sum = 0.0
    loss_c_sum = 0.0
    phase_correct = 0
    contact_correct = 0
    
    # 增加进度条显示
    pbar = tqdm(loader, desc="Validating", leave=False)
    
    for batch in pbar:
        ext = batch["ext_img"].to(device, non_blocking=True)
        left_wrist = batch["left_wrist_img"].to(device, non_blocking=True)
        right_wrist = batch["right_wrist_img"].to(device, non_blocking=True)
        wrench = batch["wrench_hist"].to(device, non_blocking=True)
        pose = batch["pose"].to(device, non_blocking=True)
        y_phase = batch["y_phase"].to(device, non_blocking=True)
        y_contact = batch["y_contact"].to(device, non_blocking=True)

        logits_p, logit_c = model.forward_logits(
            ext, left_wrist, right_wrist, wrench, pose, norm=norm_torch, img_size=img_size
        )

        loss_p = F.cross_entropy(logits_p, y_phase, weight=phase_w)
        loss_c = F.binary_cross_entropy_with_logits(logit_c, y_contact)

        pred_p = logits_p.argmax(dim=-1)
        phase_correct += int((pred_p == y_phase).sum().item())

        pred_c = (torch.sigmoid(logit_c) >= 0.5).to(torch.float32)
        contact_correct += int((pred_c == y_contact).sum().item())

        B = int(y_phase.shape[0])
        total_n += B
        loss_p_sum += float(loss_p.item()) * B
        loss_c_sum += float(loss_c.item()) * B

    return {
        "val/loss_phase": loss_p_sum / max(total_n, 1),
        "val/loss_contact": loss_c_sum / max(total_n, 1),
        "val/acc_phase": phase_correct / max(total_n, 1),
        "val/acc_contact": contact_correct / max(total_n, 1),
        "val/n": total_n,
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, default=CFG_PATH, help="Path to PAEP config .py")
    args = parser.parse_args()
    print(f"\n[Info] Loading Config From: {args.cfg}\n")
    cfg = load_cfg(args.cfg)

    required_keys = ["ZARR_PATH", "SPLIT_JSON", "SAVE_DIR"]
    for k in required_keys:
        if not hasattr(cfg, k):
            raise KeyError(f"[PAEP] Missing required config key: {k}")

    phase_names = getattr(cfg, "PHASE_NAMES", None)
    if phase_names is None:
        phase_names = ["approach", "progress", "done"]
        print("[WARN] cfg.PHASE_NAMES not found, fallback to wiping default:", phase_names)

    os.makedirs(cfg.SAVE_DIR, exist_ok=True)
    device = torch.device(getattr(cfg, "DEVICE", "cuda"))

    train_ds = PAEPFutureDataset(
        cfg.ZARR_PATH, cfg.SPLIT_JSON, "train",
        force_hist=int(cfg.FORCE_HIST),
        delta=int(cfg.DELTA),
        future_k=int(cfg.FUTURE_K),
        seed=0,
        transition_sampling=bool(cfg.TRANSITION_SAMPLING),
        transition_prob=float(cfg.TRANSITION_PROB),
        transition_window=int(cfg.TRANSITION_WINDOW),
        compute_norm=True,
        norm_samples=20000,
        phase_names=phase_names,
        use_img_aug=bool(getattr(cfg, "USE_IMG_AUG", True)),
        color_prob=float(getattr(cfg, "COLOR_PROB", 0.4)),
        geom_prob=float(getattr(cfg, "GEOM_PROB", 0.10)),
        geom_translate=0.01,
        geom_scale=0.01,
        jitter_brightness=0.10,
        jitter_contrast=0.15,
        jitter_saturation=0.05,
        jitter_hue=0.01,
    )

    val_ds = PAEPFutureDataset(
        cfg.ZARR_PATH, cfg.SPLIT_JSON, "val",
        force_hist=int(cfg.FORCE_HIST),
        delta=int(cfg.DELTA),
        future_k=int(cfg.FUTURE_K),
        seed=0,
        transition_sampling=False,
        compute_norm=False,
        provided_norm=train_ds.norm,
        phase_names=phase_names,
        use_img_aug=False,
    )

    print(f"force_hist={cfg.FORCE_HIST}, delta={cfg.DELTA}, future_k={cfg.FUTURE_K}, force_encoder={cfg.FORCE_ENCODER}")

    # ==========================================
    # 🚀 【性能优化】DataLoader 黄金配置
    # ==========================================
    # 8 workers + prefetch=2 是防止 CPU 拥堵的最佳实践
    train_loader = DataLoader(
        train_ds,
        batch_size=int(cfg.BATCH_SIZE),
        shuffle=False,
        num_workers=8,               
        pin_memory=True,
        persistent_workers=True,     
        prefetch_factor=2,           
        drop_last=True,
    )

    class _TakeN(torch.utils.data.Dataset):
        def __init__(self, base, n): self.base, self.n = base, n
        def __len__(self): return self.n
        def __getitem__(self, i): return self.base[i]

    # 验证集上限截断，防止验证时间过长
    val_limit = min(int(cfg.VAL_SAMPLES), 4000)
    print(f"[Info] Validation samples capped at: {val_limit}")
    
    val_loader = DataLoader(
        _TakeN(val_ds, val_limit),
        batch_size=min(int(cfg.BATCH_SIZE), 256), 
        shuffle=False,
        num_workers=4,
        pin_memory=True,
        persistent_workers=True,
        prefetch_factor=2,
    )

    model = PAEPFutureNet(
        num_events=train_ds.num_events,
        img_pretrained=bool(cfg.IMG_PRETRAINED),
        img_feat_dim=256,
        force_encoder=str(cfg.FORCE_ENCODER),
        force_k=int(cfg.FORCE_K),
        force_feat_dim=256,
        tcn_channels=int(cfg.TCN_CHANNELS),
        tcn_kernel=int(cfg.TCN_KERNEL),
        tcn_blocks=int(cfg.TCN_BLOCKS),
        tcn_dropout=float(cfg.TCN_DROPOUT),
        tcn_pool=str(cfg.TCN_POOL),
    ).to(device)

    # ==========================================
    # 🚀 【性能优化】编译模型 (4090 必开)
    # ==========================================
    print("[Info] Compiling model for 4090... (First epoch will be SLOW due to compilation, please wait!)")
    model = torch.compile(model)

    norm_torch = train_ds.norm.to_torch(device=device)
    phase_w = compute_class_weights(train_ds.class_counts, train_ds.phase_names).to(device)

    mult = getattr(cfg, "PHASE_WEIGHT_MULT", None)
    if isinstance(mult, dict) and len(mult) > 0:
        for i, name in enumerate(train_ds.phase_names):
            if name in mult:
                phase_w[i] = phase_w[i] * float(mult[name])
        phase_w = phase_w / (phase_w.mean() + 1e-12)

    run = None
    if bool(cfg.USE_WANDB):
        import wandb
        run = wandb.init(project=str(cfg.WANDB_PROJECT), name=str(cfg.WANDB_NAME))
        run.config.update({k: getattr(cfg, k) for k in dir(cfg) if k.isupper()})
        run.config.update({"PHASE_NAMES": train_ds.phase_names})

    def make_optimizer(vision_lr_mult=0.1):
        vision_params = []
        other_params = []
        for n, p in model.named_parameters():
            if not p.requires_grad:
                continue
            if (
                n.startswith("ext_enc.backbone")
                or n.startswith("left_wrist_enc.backbone")
                or n.startswith("right_wrist_enc.backbone")
            ):
                vision_params.append(p)
            else:
                other_params.append(p)
        groups = [{"params": other_params, "lr": float(cfg.LR)}]
        if len(vision_params) > 0:
            groups.append({"params": vision_params, "lr": float(cfg.LR) * vision_lr_mult})
        return torch.optim.AdamW(groups, weight_decay=1e-4)

    freeze_epochs = int(cfg.FREEZE_VISION_EPOCHS)
    model.set_vision_trainable(False if freeze_epochs > 0 else True)
    optimizer = make_optimizer(vision_lr_mult=1.0) 

    scaler = torch.amp.GradScaler("cuda", enabled=bool(cfg.AMP))
    autocast = torch.amp.autocast

    steps_per_epoch = int(np.ceil(int(cfg.SAMPLES_PER_EPOCH) / int(cfg.BATCH_SIZE)))

    best_score = -1.0
    best_epoch = -1
    patience_left = int(cfg.EARLY_STOP_PATIENCE)
    global_step = 0

    for epoch in range(int(cfg.EPOCHS)):
        # 冻结/解冻 Vision Backbone 的逻辑
        if freeze_epochs > 0 and epoch == freeze_epochs:
            print(f"[info] Unfreezing vision at epoch {epoch}")
            if bool(getattr(cfg, "UNFREEZE_VISION", True)):
                model.set_vision_trainable(True, layer4_only=bool(getattr(cfg, "UNFREEZE_LAYER4_ONLY", False)))

            vision_params = []
            for n, p in model.named_parameters():
                if not p.requires_grad:
                    continue
                if (
                    n.startswith("ext_enc.backbone")
                    or n.startswith("left_wrist_enc.backbone")
                    or n.startswith("right_wrist_enc.backbone")
                ):
                    vision_params.append(p)

            existing_param_ids = set()
            for g in optimizer.param_groups:
                for p in g["params"]:
                    existing_param_ids.add(id(p))

            new_params = [p for p in vision_params if id(p) not in existing_param_ids]
            if len(new_params) > 0:
                vision_lr = float(cfg.LR) * float(getattr(cfg, "VISION_LR_MULT", 0.01))
                optimizer.add_param_group({"params": new_params, "lr": vision_lr})
                print(f"[info] Added vision param_group: n={len(new_params)}, lr={vision_lr:g}")

        model.train()
        t0 = time.time()
        n_total = 0
        loss_p_sum = 0.0
        loss_c_sum = 0.0
        phase_correct = 0
        contact_correct = 0

        pbar = tqdm(enumerate(train_loader), total=steps_per_epoch, desc=f"Epoch {epoch:03d}/{int(cfg.EPOCHS)-1:03d}")
        for it, batch in pbar:
            if it >= steps_per_epoch:
                break

            ext = batch["ext_img"].to(device, non_blocking=True)
            left_wrist = batch["left_wrist_img"].to(device, non_blocking=True)
            right_wrist = batch["right_wrist_img"].to(device, non_blocking=True)
            wrench = batch["wrench_hist"].to(device, non_blocking=True)
            pose = batch["pose"].to(device, non_blocking=True)
            y_phase = batch["y_phase"].to(device, non_blocking=True)
            y_contact = batch["y_contact"].to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)

            # 使用 BF16 (4090 最佳格式)
            with autocast(device_type="cuda", dtype=torch.bfloat16, enabled=bool(cfg.AMP)):
                logits_p, logit_c = model.forward_logits(
                    ext, left_wrist, right_wrist, wrench, pose, norm=norm_torch, img_size=int(cfg.IMG_SIZE)
                )
                loss_p = F.cross_entropy(logits_p, y_phase, weight=phase_w)
                loss_c = F.binary_cross_entropy_with_logits(logit_c, y_contact)
                loss = loss_c + float(cfg.PHASE_LOSS_W) * loss_p

            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            B = int(y_phase.shape[0])
            n_total += B
            loss_p_sum += float(loss_p.item()) * B
            loss_c_sum += float(loss_c.item()) * B
            pred_p = logits_p.argmax(dim=-1)
            phase_correct += int((pred_p == y_phase).sum().item())
            pred_c = (torch.sigmoid(logit_c) >= 0.5).to(torch.float32)
            contact_correct += int((pred_c == y_contact).sum().item())

            if (global_step % int(cfg.LOG_EVERY)) == 0:
                avg_lp = loss_p_sum / max(n_total, 1)
                avg_lc = loss_c_sum / max(n_total, 1)
                acc_p = phase_correct / max(n_total, 1)
                acc_c = contact_correct / max(n_total, 1)
                mem_gb = torch.cuda.memory_allocated(device) / (1024**3)
                pbar.set_postfix(lp=f"{avg_lp:.3f}", lc=f"{avg_lc:.3f}", ap=f"{acc_p:.3f}", ac=f"{acc_c:.3f}", mem=f"{mem_gb:.1f}G")

                if run is not None:
                    run.log({
                        "train/loss_phase": avg_lp, "train/loss_contact": avg_lc,
                        "train/acc_phase": acc_p, "train/acc_contact": acc_c,
                        "train/lr": optimizer.param_groups[0]["lr"], "iter": global_step,
                    }, step=global_step)
            global_step += 1

        train_time = time.time() - t0
        
        # 验证开始
        val_t0 = time.time()
        val = eval_n(model, val_loader, device, norm_torch, img_size=int(cfg.IMG_SIZE), phase_w=phase_w)
        val_time = time.time() - val_t0

        train_lp = loss_p_sum / max(n_total, 1)
        train_lc = loss_c_sum / max(n_total, 1)
        train_ap = phase_correct / max(n_total, 1)
        train_ac = contact_correct / max(n_total, 1)

        print(
            f"[epoch {epoch:03d}] "
            f"Train: {train_time:.1f}s | Val: {val_time:.1f}s | "
            f"Acc Contact: {train_ac:.3f}->{val['val/acc_contact']:.3f}"
        )

        if run is not None:
            run.log({
                "epoch_train/acc_contact": train_ac,
                "epoch_val/acc_contact": val["val/acc_contact"],
                "epoch_time_train": train_time,
                "epoch_time_val": val_time,
                "epoch": epoch,
            }, step=global_step)

        score = float(val["val/acc_contact"]) + float(cfg.PHASE_LOSS_W) * float(val["val/acc_phase"])
        improved = score > best_score + 1e-6
        if improved:
            best_score = score
            best_epoch = epoch
            patience_left = int(cfg.EARLY_STOP_PATIENCE)
        else:
            patience_left -= 1

        # ==========================================
        # ⚠️ 修正：完整保存 Config，防止推理脚本报错
        # ==========================================
        ckpt = {
            "model": _state_dict_to_cpu(model.state_dict()),
            "cfg": {
                # 核心路径与分割
                "zarr_path": cfg.ZARR_PATH,
                "split_json": cfg.SPLIT_JSON,
                "phase_names": train_ds.phase_names,
                
                # 窗口参数 (推理必须)
                "force_hist": int(cfg.FORCE_HIST),
                "delta": int(cfg.DELTA),
                "future_k": int(cfg.FUTURE_K),
                
                # 网络架构参数 (推理必须)
                "force_encoder": str(cfg.FORCE_ENCODER),
                "force_k": int(cfg.FORCE_K),
                "tcn": {
                    "channels": int(cfg.TCN_CHANNELS),
                    "kernel": int(cfg.TCN_KERNEL),
                    "blocks": int(cfg.TCN_BLOCKS),
                    "dropout": float(cfg.TCN_DROPOUT),
                    "pool": str(cfg.TCN_POOL),
                },
                "img_size": int(cfg.IMG_SIZE),
                "img_pretrained": bool(cfg.IMG_PRETRAINED),
                "num_events": train_ds.num_events,
                
                # 损失权重 (方便复现)
                "phase_loss_w": float(cfg.PHASE_LOSS_W),
                "PHASE_WEIGHT_MULT": getattr(cfg, "PHASE_WEIGHT_MULT", None),
                "phase_w": phase_w.detach().float().cpu().tolist(),
                
                # 统计信息
                "train_class_counts": dict(train_ds.class_counts),
            },
            "norm": {
                "wrench_mean": train_ds.norm.wrench_mean,
                "wrench_std": train_ds.norm.wrench_std,
                "pose_mean": train_ds.norm.pose_mean,
                "pose_std": train_ds.norm.pose_std,
            },
            "val": val,
        }

        ckpt = _move_ckpt_tensors_to_cpu(ckpt)
        last_path = os.path.join(cfg.SAVE_DIR, "last.pt")
        atomic_torch_save(ckpt, last_path)
        if improved:
            best_path = os.path.join(cfg.SAVE_DIR, "best.pt")
            atomic_torch_save(ckpt, best_path)

        if patience_left <= 0:
            print(f"[early-stop] Stop at epoch {epoch}.")
            break

    if run is not None:
        run.finish()
    print("Done. best_score:", best_score)

if __name__ == "__main__":
    main()