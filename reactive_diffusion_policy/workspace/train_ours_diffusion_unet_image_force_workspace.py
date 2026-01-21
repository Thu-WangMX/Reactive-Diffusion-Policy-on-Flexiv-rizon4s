# reactive_diffusion_policy/workspace/train_diffusion_unet_image_workspace.py

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

            # hack: scale lr by WORLD_SIZE
            world_size = int(os.environ.get("WORLD_SIZE", "1"))
            world_size = max(world_size, 1)
            print("###########################################")
            print(f"Number of available CUDA devices (WORLD_SIZE): {world_size}.")
            print(f"Original learning rate: {optimizer_cfg['lr']}")
            optimizer_cfg["lr"] = optimizer_cfg["lr"] * world_size
            print(f"Updated learning rate: {optimizer_cfg['lr']}")
            print("###########################################")

            self.optimizer = hydra.utils.instantiate(optimizer_cfg, params=self.model.parameters())

        # configure training state
        self.global_step = 0
        self.epoch = 0

    def run(self):
        cfg = copy.deepcopy(self.cfg)

        # IMPORTANT: if your custom policy may have unused params on some ranks,
        # let accelerate build DDP with find_unused_parameters=True
        ddp_kwargs = DistributedDataParallelKwargs(find_unused_parameters=True)

        accelerator = Accelerator(log_with="wandb", kwargs_handlers=[ddp_kwargs])
        accelerator.print(f"[debug] accelerator.mixed_precision = {accelerator.mixed_precision}")

        # init trackers
        wandb_cfg = OmegaConf.to_container(cfg.logging, resolve=True)
        wandb_cfg.pop("project")
        accelerator.init_trackers(
            project_name=cfg.logging.project,
            config=OmegaConf.to_container(cfg, resolve=True),
            init_kwargs={"wandb": wandb_cfg},
        )

        # resume training
        if cfg.training.resume:
            lastest_ckpt_path = self.get_checkpoint_path()
            if lastest_ckpt_path.is_file():
                accelerator.print(f"Resuming from checkpoint {lastest_ckpt_path}")
                self.load_checkpoint(path=lastest_ckpt_path)

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
        normalizer = pickle.load(open(normalizer_path, "rb"))

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

        # accelerator prepare (DO NOT manually wrap DDP again!)
        train_dataloader, val_dataloader, self.model, self.optimizer, lr_scheduler = accelerator.prepare(
            train_dataloader, val_dataloader, self.model, self.optimizer, lr_scheduler
        )

        # device
        device = accelerator.device
        if self.ema_model is not None:
            self.ema_model.to(device)

        # save batch for sampling
        train_sampling_batch = None

        # debug mode
        if cfg.training.debug:
            cfg.training.num_epochs = 2
            cfg.training.max_train_steps = 3
            cfg.training.max_val_steps = 3
            cfg.training.rollout_every = 1
            cfg.training.checkpoint_every = 1
            cfg.training.val_every = 1
            cfg.training.sample_every = 1

        # training loop
        log_path = os.path.join(self.output_dir, "logs.json.txt")
        with JsonLogger(log_path) as json_logger:
            for _local_epoch_idx in range(cfg.training.num_epochs):
                step_log = dict()

                # ========= train for this epoch ==========
                if cfg.training.freeze_encoder:
                    raw_model = accelerator.unwrap_model(self.model)
                    raw_model.obs_encoder.eval()
                    raw_model.obs_encoder.requires_grad_(False)

                train_losses = []
                # local counter for gradient accumulation (CORRECT)
                accum_step = 0

                with tqdm.tqdm(
                    train_dataloader,
                    desc=f"Training epoch {self.epoch}",
                    leave=False,
                    mininterval=cfg.training.tqdm_interval_sec,
                ) as tepoch:
                    for batch_idx, batch in enumerate(tepoch):
                        # device transfer
                        batch = dict_apply(batch, lambda x: x.to(device, non_blocking=True))
                        if train_sampling_batch is None:
                            train_sampling_batch = batch

                        # compute loss
                        raw_loss = self.model(batch)

                        # ===== NaN/Inf guard (before backward) =====
                        if not torch.isfinite(raw_loss).all():
                            save_dir = os.path.join(self.output_dir, "nan_debug")
                            os.makedirs(save_dir, exist_ok=True)
                            save_path = os.path.join(save_dir, f"nan_batch_step_{self.global_step}.pt")
                            try:
                                batch_cpu = dict_apply(batch, lambda x: x.detach().cpu())
                                torch.save({"global_step": self.global_step, "batch": batch_cpu}, save_path)
                                accelerator.print(f"[NaN-DEBUG] Saved batch to {save_path}")
                            except Exception as e:
                                accelerator.print(f"[NaN-DEBUG] Failed to save batch: {e}")
                            raise RuntimeError(
                                f"[NaN-DEBUG] raw_loss is not finite at step={self.global_step}, raw_loss={raw_loss}"
                            )
                        # =========================================

                        # scale loss for accumulation
                        loss = raw_loss / cfg.training.gradient_accumulate_every
                        accelerator.backward(loss)

                        accum_step += 1
                        # do optimizer step when we have accumulated enough micro-batches
                        do_opt_step = (accum_step % cfg.training.gradient_accumulate_every == 0)

                        if do_opt_step:
                            # grad clip + grad norm log (main process only)
                            grad_norm = torch.nn.utils.clip_grad_norm_(self.model.parameters(), 1.0)
                            if accelerator.is_main_process:
                                accelerator.log({"grad_norm": float(grad_norm)}, step=self.global_step)

                            self.optimizer.step()
                            self.optimizer.zero_grad(set_to_none=True)
                            lr_scheduler.step()

                        # update ema (after optimizer step is also ok; keep consistent with your previous behavior)
                        if cfg.training.use_ema:
                            ema.step(accelerator.unwrap_model(self.model))

                        # logging
                        raw_loss_cpu = float(raw_loss.detach().item())
                        tepoch.set_postfix(loss=raw_loss_cpu, refresh=False)
                        train_losses.append(raw_loss_cpu)

                        step_log = {
                            "train_loss": raw_loss_cpu,
                            "global_step": self.global_step,
                            "epoch": self.epoch,
                            "lr": lr_scheduler.get_last_lr()[0],
                        }

                        is_last_batch = (batch_idx == (len(train_dataloader) - 1))
                        if not is_last_batch:
                            # ✅ merge policy-provided extra logs into SAME step (fix wandb monotonic warnings)
                            raw_model = accelerator.unwrap_model(self.model)
                            extra = getattr(raw_model, "_extra_step_log", None)
                            if isinstance(extra, dict) and len(extra) > 0:
                                step_log.update(extra)
                                raw_model._extra_step_log = None

                            # log of last step is combined with validation and rollout
                            accelerator.log(step_log, step=self.global_step)
                            json_logger.log(step_log)
                            self.global_step += 1

                        if (cfg.training.max_train_steps is not None) and batch_idx >= (cfg.training.max_train_steps - 1):
                            break

                # at the end of each epoch
                train_loss = float(np.mean(train_losses)) if len(train_losses) > 0 else float("nan")
                step_log["train_loss"] = train_loss

                # ========= eval for this epoch ==========
                policy = accelerator.unwrap_model(self.model)
                if cfg.training.use_ema:
                    policy = self.ema_model
                policy.eval()

                # run validation (main process only)
                if (
                    cfg.task.dataset.val_ratio > 0
                    and (self.epoch % cfg.training.val_every) == 0
                    and accelerator.is_main_process
                ):
                    with torch.no_grad():
                        val_losses = []
                        with tqdm.tqdm(
                            val_dataloader,
                            desc=f"Validation epoch {self.epoch}",
                            leave=False,
                            mininterval=cfg.training.tqdm_interval_sec,
                        ) as tepoch:
                            for batch_idx, batch in enumerate(tepoch):
                                batch = dict_apply(batch, lambda x: x.to(device, non_blocking=True))
                                vloss = self.model(batch)
                                val_losses.append(float(vloss.detach().item()) if torch.is_tensor(vloss) else float(vloss))
                                if (cfg.training.max_val_steps is not None) and batch_idx >= (cfg.training.max_val_steps - 1):
                                    break

                        if len(val_losses) > 0:
                            step_log["val_loss"] = float(np.mean(val_losses))

                # run diffusion sampling on a training batch
                if (self.epoch % cfg.training.sample_every) == 0:
                    with torch.no_grad():
                        batch = dict_apply(train_sampling_batch, lambda x: x.to(device, non_blocking=True))
                        obs_dict = batch["obs"]
                        extended_obs_dict = batch.get("extended_obs", None)
                        gt_action = batch["action"]

                        if "latent" in cfg.name:
                            dataset_obs_temporal_downsample_ratio = cfg.task.dataset.obs_temporal_downsample_ratio
                            result = policy.predict_action(
                                obs_dict,
                                extended_obs_dict=extended_obs_dict,
                                dataset_obs_temporal_downsample_ratio=dataset_obs_temporal_downsample_ratio,
                            )
                        else:
                            result = policy.predict_action(obs_dict)

                        pred_action = result["action_pred"]
                        all_preds, all_gt = accelerator.gather_for_metrics((pred_action, gt_action))
                        mse = torch.nn.functional.mse_loss(all_preds, all_gt)
                        step_log["train_action_mse_error"] = float(mse.detach().item())

                        # cleanup
                        del batch, obs_dict, gt_action, result, pred_action, mse

                accelerator.wait_for_everyone()

                # checkpoint (main only)
                if (self.epoch % cfg.training.checkpoint_every) == 0 and accelerator.is_main_process:
                    model_wrapped = self.model
                    self.model = accelerator.unwrap_model(self.model)

                    if cfg.checkpoint.save_last_ckpt:
                        self.save_checkpoint()
                    if cfg.checkpoint.save_last_snapshot:
                        self.save_snapshot()

                    metric_dict = {k.replace("/", "_"): v for k, v in step_log.items()}
                    topk_ckpt_path = topk_manager.get_ckpt_path(metric_dict)
                    if topk_ckpt_path is not None:
                        self.save_checkpoint(path=topk_ckpt_path)

                    self.model = model_wrapped

                # ========= eval end for this epoch ==========
                policy.train()

                # ✅ merge policy extra logs (e.g., computed during val/sample) into SAME step
                raw_model = accelerator.unwrap_model(self.model)
                extra = getattr(raw_model, "_extra_step_log", None)
                if isinstance(extra, dict) and len(extra) > 0:
                    step_log.update(extra)
                    raw_model._extra_step_log = None

                # end of epoch: log combined train/val/sample metrics on current global_step
                accelerator.log(step_log, step=self.global_step)
                json_logger.log(step_log)
                self.global_step += 1
                self.epoch += 1

        accelerator.end_training()


@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.parent.joinpath("config")),
    config_name=pathlib.Path(__file__).stem,
)
def main(cfg):
    workspace = TrainDiffusionUnetImageWorkspace(cfg)
    workspace.run()


if __name__ == "__main__":
    main()
