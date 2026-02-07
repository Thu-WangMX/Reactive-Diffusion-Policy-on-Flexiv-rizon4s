#擦白板任务的paep配置

ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
SPLIT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/config/paep_split_40_5_5_wipingboard.json"
SAVE_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_wiping_board_0124"

FORCE_HIST = 48
DELTA = 6
FUTURE_K = 12  # 24Hz * 0.5s window

# Force encoder switch: "gru" or "tcn"
FORCE_ENCODER = "tcn"

FORCE_K = 25  # GRU conv kernel

# TCN params (only used if FORCE_ENCODER="tcn")
TCN_CHANNELS = 256
TCN_KERNEL = 5
TCN_BLOCKS = 4
TCN_DROPOUT = 0.0
TCN_POOL = "mean"  # "mean" or "last"

IMG_PRETRAINED = True
IMG_SIZE = 224

BATCH_SIZE = 128
SAMPLES_PER_EPOCH = 20000
VAL_SAMPLES = 4000
EPOCHS = 20
LR = 3e-4

DEVICE = "cuda"
AMP = True

# transition oversampling (phase transition)
TRANSITION_SAMPLING = True
TRANSITION_PROB = 0.3
TRANSITION_WINDOW = 12

# wiping：phase 不是重点，保留通用性但降低权重
PHASE_LOSS_W = 0.1

FREEZE_VISION_EPOCHS = 10
EARLY_STOP_PATIENCE = 5
LOG_EVERY = 50

# wandb
USE_WANDB = True
WANDB_PROJECT = "PAEP"
WANDB_NAME = "paep_future_tcn_0124"

PHASE_NAMES = ["approach", "progress", "done"]
# =========================