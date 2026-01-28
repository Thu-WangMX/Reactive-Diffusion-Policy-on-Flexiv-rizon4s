import os
import argparse
import torch
from hydra.utils import instantiate
from omegaconf import OmegaConf
from hydra import compose, initialize_config_dir
import os, sys
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import os
import argparse
import torch
import math
import datetime
from omegaconf import OmegaConf
from hydra.utils import instantiate
from hydra import compose, initialize_config_dir

if not OmegaConf.has_resolver("eval"):
    OmegaConf.register_new_resolver("eval", lambda expr: eval(expr, {"math": math}))
if not OmegaConf.has_resolver("now"):
    OmegaConf.register_new_resolver("now", lambda fmt: datetime.datetime.now().strftime(fmt))

def load_cfg_from_yaml_path(yaml_path: str):
    yaml_path = os.path.abspath(yaml_path)
    config_dir = os.path.dirname(yaml_path)
    config_name = os.path.splitext(os.path.basename(yaml_path))[0]  # 不带 .yaml

    # 用 Hydra compose，这样 defaults 会被正确展开（task 会出现）
    with initialize_config_dir(config_dir=config_dir, version_base=None):
        cfg = compose(config_name=config_name, overrides=[])

    return cfg


def to_device(x, device):
    if torch.is_tensor(x):
        return x.to(device)
    if isinstance(x, dict):
        return {k: to_device(v, device) for k, v in x.items()}
    return x


def main(cfg_yaml_path: str):
    cfg = load_cfg_from_yaml_path(cfg_yaml_path)

    # 关闭 wandb 影响（可选）
    if "logging" in cfg:
        cfg.logging.mode = "disabled"
    if "training" in cfg:
        cfg.training.debug = True

    # ------- dataset -------
    if "task" in cfg and "dataset" in cfg.task:
        dataset_cfg = cfg.task.dataset
    elif "dataset" in cfg:
        dataset_cfg = cfg.dataset
    else:
        raise KeyError(f"Cannot find dataset config. Top-level keys: {list(cfg.keys())}")

    dataset = instantiate(dataset_cfg)
    normalizer = dataset.get_normalizer()

    # dataloader（只取一个 batch）
    dl_cfg = cfg.dataloader if "dataloader" in cfg else None
    batch_size = int(getattr(dl_cfg, "batch_size", 2)) if dl_cfg else 2
    num_workers = int(getattr(dl_cfg, "num_workers", 0)) if dl_cfg else 0

    loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=False,
        drop_last=True,
    )

    # ------- policy -------
    if "policy" not in cfg:
        raise KeyError(f"Cannot find policy in cfg. Top-level keys: {list(cfg.keys())}")

    policy = instantiate(cfg.policy)
    policy.set_normalizer(normalizer)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy.to(device)
    policy.train()

    batch = next(iter(loader))
    batch = to_device(batch, device)

    # 单次 forward（触发你加的 [DBG] 打印）
    with torch.no_grad():
        loss = policy.compute_loss(batch)

    print("[OK] loss =", float(loss.detach().cpu()))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("cfg_yaml_path", type=str)
    args = parser.parse_args()
    main(args.cfg_yaml_path)
