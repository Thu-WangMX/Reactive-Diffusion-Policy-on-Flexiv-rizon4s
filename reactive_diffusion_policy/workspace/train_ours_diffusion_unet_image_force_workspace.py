

if __name__ == "__main__":
    import sys
    import os
    import pathlib

    ROOT_DIR = str(pathlib.Path(__file__).parent.parent.parent)
    sys.path.append(ROOT_DIR)
    os.chdir(ROOT_DIR)

import os
import copy
import random
import pathlib
import pickle

import hydra
import torch
import tqdm
import numpy as np
from omegaconf import OmegaConf
from torch.utils.data import DataLoader
from accelerate import Accelerator
from accelerate import DistributedDataParallelKwargs

from reactive_diffusion_policy.workspace.base_workspace import BaseWorkspace
from reactive_diffusion_policy.policy.diffusion_unet_image_policy import DiffusionUnetImagePolicy
from reactive_diffusion_policy.dataset.base_dataset import BaseImageDataset
from reactive_diffusion_policy.common.checkpoint_util import TopKCheckpointManager
from reactive_diffusion_policy.common.json_logger import JsonLogger
from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.model.diffusion.ema_model import EMAModel
from reactive_diffusion_policy.model.common.lr_scheduler import get_scheduler
from reactive_diffusion_policy.model.common.lr_decay import param_groups_lrd

OmegaConf.register_new_resolver("eval", eval, replace=True)


class TrainDiffusionUnetImageWorkspace(BaseWorkspace):
    include_keys = ["global_step", "epoch"]

    def __init__(self, cfg: OmegaConf, output_dir=None):
        super().__init__(cfg, output_dir=output_dir)

        # set seed
        seed = cfg.training.seed
        torch.manual_seed(seed)
        np.random.seed(seed)
        random.seed(seed)

        # configure model
        self.model: DiffusionUnetImagePolicy = hydra.utils.instantiate(cfg.policy)
        print("[debug] model class =", type(self.model))

        self.ema_model: DiffusionUnetImagePolicy = None
        if cfg.training.use_ema:
            self.ema_model = copy.deepcopy(self.model)

        # ------------------------------
        # configure optimizer
        # ------------------------------
        if "timm" in cfg.policy.obs_encoder._target_:
            if cfg.training.layer_decay < 1.0:
                assert not cfg.policy.obs_encoder.use_lora
                assert not cfg.policy.obs_encoder.share_rgb_model
                obs_encorder_param_groups = param_groups_lrd(
                    self.model.obs_encoder,
                    shape_meta=cfg.shape_meta,
                    weight_decay=cfg.optimizer.encoder_weight_decay,
                    no_weight_decay_list=self.model.obs_encoder.no_weight_decay(),
                    layer_decay=cfg.training.layer_decay,
                )
                count = 0
                for group in obs_encorder_param_groups:
                    count += len(group["params"])
                if cfg.policy.obs_encoder.feature_aggregation == "map":
                    obs_encorder_param_groups.extend(
                        [{"params": self.model.obs_encoder.attn_pool.parameters()}]
                    )
                    for _ in self.model.obs_encoder.attn_pool.parameters():
                        count += 1
                print(f"obs_encorder params: {count}")

                param_groups = [{"params": self.model.model.parameters()}]
                param_groups.extend(obs_encorder_param_groups)
            else:
                obs_encorder_lr = cfg.optimizer.lr
                if cfg.policy.obs_encoder.pretrained and not cfg.policy.obs_encoder.use_lora:
                    obs_encorder_lr *= cfg.training.encoder_lr_coefficient
                    print("==> reduce pretrained obs_encorder's lr")
                obs_encorder_params = []
                for param in self.model.obs_encoder.parameters():
                    if param.requires_grad:
                        obs_encorder_params.append(param)
                print(f"obs_encorder params: {len(obs_encorder_params)}")
                param_groups = [
                    {"params": self.model.model.parameters()},
                    {"params": obs_encorder_params, "lr": obs_encorder_lr},
                ]

            optimizer_cfg = OmegaConf.to_container(cfg.optimizer, resolve=True)
            optimizer_cfg.pop("_target_")
            if "encoder_weight_decay" in optimizer_cfg:
                optimizer_cfg.pop("encoder_weight_decay")
            self.optimizer = torch.optim.AdamW(params=param_groups, **optimizer_cfg)

        else:
            optimizer_cfg = OmegaConf.to_container(cfg.optimizer, resolve=True)
            if "encoder_weight_decay" in optimizer_cfg:
                optimizer_cfg.pop("encoder_weight_decay")

            # hack: use larger learning rate for multiple gpus (match original DP workspace behavior)
            tmp_accelerator = Accelerator()
        
            cuda_count = tmp_accelerator.num_processes
            print("###########################################")
            print(f"Number of available CUDA devices: {cuda_count}.")
            print(f"Original learning rate: {optimizer_cfg['lr']}")
            optimizer_cfg["lr"] = optimizer_cfg["lr"] * cuda_count
            print(f"Updated learning rate: {optimizer_cfg['lr']}")
            print("###########################################")

            self.optimizer = hydra.utils.instantiate(
                optimizer_cfg, params=self.model.parameters()
            )

        # configure training state
        self.global_step = 0
        self.epoch = 0

    def run(self):
        cfg = copy.deepcopy(self.cfg)

        # IMPORTANT: if your custom policy may have unused params on some ranks,
        # let accelerate build DDP with find_unused_parameters=True
        # accelerator prepare
        ddp_kwargs = DistributedDataParallelKwargs(find_unused_parameters=True)
        accelerator = Accelerator(log_with="wandb", kwargs_handlers=[ddp_kwargs])

        # only in the main process initialize wandb
        if accelerator.is_main_process:
            accelerator.init_trackers(
                project_name=cfg.logging.project,
                config=OmegaConf.to_container(cfg, resolve=True),
                init_kwargs={"wandb": {"name": cfg.logging.run_name}},
            )

        
        os.makedirs(self.output_dir, exist_ok=True)

        log_path = os.path.join(self.output_dir, "logs.jsonl")

        # 如果不是 resume：主进程先删掉旧日志文件，避免续写混淆
        if accelerator.is_main_process and (not cfg.training.resume) and os.path.exists(log_path):
            os.remove(log_path)

        # 等所有进程同步后再创建 logger（避免多进程竞争）
        accelerator.wait_for_everyone()

        import json

        def jsonl_log(obj: dict):
            # 只让主进程写文件
            if not accelerator.is_main_process:
                return
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(obj, ensure_ascii=False) + "\n")
                f.flush()



        # save batch for sampling
        train_sampling_batch = None

        # configure dataset
        dataset: BaseImageDataset = hydra.utils.instantiate(cfg.task.dataset)
        assert isinstance(dataset, BaseImageDataset)
        train_dataloader = DataLoader(dataset, **cfg.dataloader)

        # compute normalizer on the main process and save to disk
        normalizer_path = os.path.join(self.output_dir, "normalizer.pkl")
        if accelerator.is_main_process:
            normalizer = dataset.get_normalizer()
            with open(normalizer_path, "wb") as f:
                pickle.dump(normalizer, f)

        # load normalizer on all processes
        accelerator.wait_for_everyone()
        with open(normalizer_path, "rb") as f:
            normalizer = pickle.load(f)

        # configure validation dataset
        val_dataset = dataset.get_validation_dataset()
        val_dataloader = DataLoader(val_dataset, **cfg.val_dataloader)

        # set normalizer
        self.model.set_normalizer(normalizer)
        if cfg.training.use_ema:
            self.ema_model.set_normalizer(normalizer)

        # configure lr scheduler (counts OPTIMIZER steps)
        lr_scheduler = get_scheduler(
            cfg.training.lr_scheduler,
            optimizer=self.optimizer,
            num_warmup_steps=cfg.training.lr_warmup_steps,
            num_training_steps=(len(train_dataloader) * cfg.training.num_epochs)
            // cfg.training.gradient_accumulate_every,
            last_epoch=self.global_step - 1,
        )

        # configure ema
        ema: EMAModel = None
        if cfg.training.use_ema:
            ema = hydra.utils.instantiate(cfg.ema, model=self.ema_model)

        # configure checkpoint
        topk_manager = TopKCheckpointManager(
            save_dir=os.path.join(self.output_dir, "checkpoints"),
            **cfg.checkpoint.topk,
        )

        # accelerator prepare
        train_dataloader, val_dataloader, self.model, self.optimizer, lr_scheduler = accelerator.prepare(
            train_dataloader, val_dataloader, self.model, self.optimizer, lr_scheduler
        )

        # device
        device = accelerator.device
        if self.ema_model is not None:
            self.ema_model.to(device)

        # resume
        if cfg.training.resume:
            lastest_ckpt_path = self.get_checkpoint_path(tag="latest")
            if lastest_ckpt_path is not None and os.path.isfile(lastest_ckpt_path):
                accelerator.print(f"[INFO] Resume from {lastest_ckpt_path}")
                self.load_checkpoint(path=lastest_ckpt_path)

        # main loop
        with tqdm.tqdm(
            range(cfg.training.num_epochs),
            desc="Epoch",
            leave=True,
            disable=not accelerator.is_main_process,
            mininterval=cfg.training.tqdm_interval_sec,
        ) as epoch_pbar:
            for _ in epoch_pbar:
                accelerator.wait_for_everyone()
                self.model.train()

                train_losses = []
                accum_step = 0

                # ✅ store last train-step log to be merged with val/sample at epoch end
                pending_last_train_step_log = None

                with tqdm.tqdm(
                    train_dataloader,
                    desc=f"Training epoch {self.epoch}",
                    leave=False,
                    disable=not accelerator.is_main_process,
                    mininterval=cfg.training.tqdm_interval_sec,
                ) as tepoch:
                    for batch_idx, batch in enumerate(tepoch):
                        batch = dict_apply(batch, lambda x: x.to(device, non_blocking=True))

                        # -------- debug: log episode id distribution in this batch --------
                        ep_stats = None
                        eid_tensor = None

                        if isinstance(batch, dict):
                            # preferred nested style: batch["debug"]["episode_id"]
                            if ("debug" in batch) and isinstance(batch["debug"], dict) and ("episode_id" in batch["debug"]):
                                eid_tensor = batch["debug"]["episode_id"]
                            # fallback flat key style: batch["debug/episode_id"]
                            elif "debug/episode_id" in batch:
                                eid_tensor = batch["debug/episode_id"]

                        if eid_tensor is not None:
                            eid = eid_tensor.view(-1)
                            ep_stats = {
                                "data/episode_id_min": float(eid.min().item()),
                                "data/episode_id_max": float(eid.max().item()),
                                "data/episode_id_mean": float(eid.float().mean().item()),
                            }


                        if train_sampling_batch is None:
                            train_sampling_batch = batch

                        # compute loss
                        raw_loss = self.model(batch)

                        # NaN/inf guard
                        if not torch.isfinite(raw_loss):
                            raise RuntimeError(
                                f"[NaN-DEBUG] raw_loss is not finite at step={self.global_step}, raw_loss={raw_loss}"
                            )

                        loss = raw_loss / cfg.training.gradient_accumulate_every
                        accelerator.backward(loss)

                        accum_step += 1
                        do_opt_step = (accum_step % cfg.training.gradient_accumulate_every == 0)

                        if do_opt_step:
                            grad_norm = torch.nn.utils.clip_grad_norm_(self.model.parameters(), 1.0)
                            if accelerator.is_main_process:
                                accelerator.log({"grad_norm": float(grad_norm)}, step=self.global_step)

                            self.optimizer.step()
                            self.optimizer.zero_grad(set_to_none=True)
                            lr_scheduler.step()

                        # update ema (match DP behavior: step every batch)
                        # if cfg.training.use_ema:
                        #     ema.step(accelerator.unwrap_model(self.model))
                        if do_opt_step and cfg.training.use_ema:
                            ema.step(accelerator.unwrap_model(self.model))

                        raw_loss_cpu = float(raw_loss.detach().item())
                        tepoch.set_postfix(loss=raw_loss_cpu, refresh=False)
                        train_losses.append(raw_loss_cpu)

                        step_log = {
                            # --- loss keys: DO NOT mix step and epoch meanings ---
                            "train/loss_step": raw_loss_cpu,

                            "global_step": self.global_step,
                            "epoch": self.epoch,
                            "lr": lr_scheduler.get_last_lr()[0],
                        }
                        if ep_stats is not None:
                            step_log.update(ep_stats)


                        is_last_batch = (batch_idx == (len(train_dataloader) - 1))

                        # ✅ ALWAYS merge policy-provided extra logs first (including last batch)
                        raw_model = accelerator.unwrap_model(self.model)
                        extra = getattr(raw_model, "_extra_step_log", None)
                        if isinstance(extra, dict) and len(extra) > 0:
                            step_log.update(extra)
                            raw_model._extra_step_log = None

                        if not is_last_batch:
                            # normal steps: log now
                            accelerator.log(step_log, step=self.global_step)
                            
                            if accelerator.is_main_process:
                                jsonl_log(step_log)

                            self.global_step += 1
                        else:
                            # last step: defer logging (will be combined with val/sample at SAME global_step)
                            pending_last_train_step_log = dict(step_log)

                        if (cfg.training.max_train_steps is not None) and batch_idx >= (cfg.training.max_train_steps - 1):
                            break

                # at the end of each epoch
                train_loss = float(np.mean(train_losses)) if len(train_losses) > 0 else float("nan")

                # ✅ start final_log from last train step (deferred), or fallback
                final_log = dict(pending_last_train_step_log) if pending_last_train_step_log is not None else dict(step_log)

                # keep last-step loss (already in final_log["train/loss_step"]) and add epoch summary separately
                final_log["train/loss_epoch"] = train_loss

                # (optional, helpful) make it explicit what's inside final_log
                final_log["train/loss_last_step"] = float(final_log.get("train/loss_step", float("nan")))


                # ========= eval for this epoch ==========
                policy = accelerator.unwrap_model(self.model)
                if cfg.training.use_ema:
                    policy = self.ema_model
                policy.eval()

                # run validation (match DP behavior: they still call self.model(batch); keep here for consistency)
                if cfg.task.dataset.val_ratio > 0 and (self.epoch % cfg.training.val_every) == 0 and accelerator.is_main_process:
                    with torch.no_grad():
                        val_losses = []
                        with tqdm.tqdm(
                            val_dataloader,
                            desc=f"Validation epoch {self.epoch}",
                            leave=False,
                            mininterval=cfg.training.tqdm_interval_sec,
                        ) as vepoch:
                            for vbatch_idx, vbatch in enumerate(vepoch):
                                vbatch = dict_apply(vbatch, lambda x: x.to(device, non_blocking=True))
                                vloss = policy(vbatch)
                                val_losses.append(float(vloss.detach().item()) if torch.is_tensor(vloss) else float(vloss))
                                if (cfg.training.max_val_steps is not None) and vbatch_idx >= (cfg.training.max_val_steps - 1):
                                    break
                        if len(val_losses) > 0:
                            final_log["val_loss"] = float(np.mean(val_losses))

                # ========= checkpoint saving ==========
                if (self.epoch % cfg.training.checkpoint_every) == 0 and accelerator.is_main_process:
                    model_wrapped = self.model
                    self.model = accelerator.unwrap_model(self.model)

                    if cfg.checkpoint.save_last_ckpt:
                        self.save_checkpoint()
                    if cfg.checkpoint.save_last_snapshot:
                        self.save_snapshot()

                    # =========================================================
                    # [修改] 每 100 epoch 或 最后一轮 强制保存
                    # =========================================================
                    is_last_epoch = (self.epoch == cfg.training.num_epochs - 1)

                    if self.epoch % 100 == 0 or is_last_epoch:
                        periodic_ckpt_path = os.path.join(
                            self.output_dir, 'checkpoints', f'epoch_{self.epoch:04d}.ckpt'
                        )
                        self.save_checkpoint(path=periodic_ckpt_path)
                        accelerator.print(f"[INFO] Saved periodic checkpoint to {periodic_ckpt_path}")

                    metric_dict = {k.replace("/", "_"): v for k, v in final_log.items()}
                    topk_ckpt_path = topk_manager.get_ckpt_path(metric_dict)
                    if topk_ckpt_path is not None:
                        self.save_checkpoint(path=topk_ckpt_path)

                    self.model = model_wrapped

                policy.train()

                # ✅ merge policy extra logs (e.g., computed during val/sample) into SAME step
                raw_model = accelerator.unwrap_model(self.model)
                extra = getattr(raw_model, "_extra_step_log", None)
                if isinstance(extra, dict) and len(extra) > 0:
                    final_log.update(extra)
                    raw_model._extra_step_log = None

                # end of epoch: log combined last-train-step + val metrics on current global_step
                final_log["global_step"] = self.global_step
                if accelerator.is_main_process:
                    accelerator.log(final_log, step=self.global_step)
                    jsonl_log(final_log)

                self.global_step += 1
                self.epoch += 1


@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.parent.parent / "config"),
    config_name="train_ours_diffusion_unet_image_force_workspace",
)
def main(cfg):
    workspace = TrainDiffusionUnetImageWorkspace(cfg)
    workspace.run()


if __name__ == "__main__":
    main()
