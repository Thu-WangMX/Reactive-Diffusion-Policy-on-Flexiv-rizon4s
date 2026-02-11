# PAEP/configs/paep_plug_tcn_last.py

# =========================
# dataset / io
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_usb/plug_in_usb_stream_downsample1_zarr/replay_buffer.zarr"
SPLIT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/config/paep_split_plugin_usb.json" 
SAVE_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/ckpt/paep_plug_in_usb_0210"

PHASE_NAMES = ["approach", "search", "recovery", "insert"]  # 

# =========================
# time window
# =========================
FORCE_HIST = 36
DELTA = 3
FUTURE_K = 8

# =========================
# force encoder
# =========================
FORCE_ENCODER = "tcn"   # "gru" or "tcn"
FORCE_K = 25            # GRU conv kernel

# TCN params (only used if FORCE_ENCODER="tcn")
TCN_CHANNELS = 256
TCN_KERNEL = 7
TCN_BLOCKS = 4
TCN_DROPOUT = 0.1
TCN_POOL = "last"       # "mean" or "last"

# =========================
# vision
# =========================
IMG_PRETRAINED = True
IMG_SIZE = 224

UNFREEZE_VISION = True
UNFREEZE_LAYER4_ONLY = True
FREEZE_VISION_EPOCHS = 5   # 或 3；别再写 100
VISION_LR_MULT = 0.001      # backbone 用更小 lr

USE_IMG_AUG = True
COLOR_PROB = 0.3     
GEOM_PROB = 0.05       

# =========================
# train
# =========================
BATCH_SIZE = 128
SAMPLES_PER_EPOCH = 20000
VAL_SAMPLES = 20000
EPOCHS = 30
LR = 3e-4

DEVICE = "cuda"
AMP = True

# transition oversampling
TRANSITION_SAMPLING = True
TRANSITION_PROB = 0.2
TRANSITION_WINDOW = 12

# loss weights
PHASE_LOSS_W = 1.2

# schedule / log
EARLY_STOP_PATIENCE = 10
LOG_EVERY = 50

# wandb
USE_WANDB = True
WANDB_PROJECT = "PAEP"
WANDB_NAME = "paep_plugin_usb_tcn_last_0211"

# per-class weight multiplier (after inverse-frequency weight computed)
PHASE_WEIGHT_MULT = {
    #"recovery": 0.3,   # 先压到 0.2~0.3 试试
}


