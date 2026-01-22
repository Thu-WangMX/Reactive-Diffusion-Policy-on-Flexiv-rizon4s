# #!/bin/bash

# CUDA_VISIBLE_DEVICES=0 accelerate launch train.py \
#     --config-name=train_diffusion_unet_real_image_workspace \
#     task=real_peel_image_gelsight_emb_dp_absolute_12fps \
#     task.dataset_path=/home/wendi/Desktop/record_data/peel_v3_downsample2_zarr \
#     task.name=real_peel_image_gelsight_emb_dp_absolute_12fps \
#     logging.mode=online




# ================= 配置区域 =================
# 1. 物理显卡ID：只填你想用的那两张卡 (例如 6 和 7)
# 这一步将物理卡6,7映射为逻辑卡0,1
export CUDA_VISIBLE_DEVICES=6,7

# 2. 进程数：必须等于上面显卡的数量 (例如 2)
NUM_PROCESSES=2

# 3. 端口号：防止冲突 (默认 29500)
MASTER_PORT=29500

# ==== 以后只改这两行 ====
#DATASET_PATH="/home/wmx/myspace/RDP/data/plug_in_downsample1_zarr"
DATASET_PATH="/work/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample2_zarr"
DP_TASK="wmx_real_wiping_board_image_dp_absolute_12fps"
# ========================

LOGGING_MODE="online"

# 5. 日志存放目录 (会自动创建)
LOG_DIR="dp_train_logs"
# ===========================================

# 准备环境
mkdir -p "${LOG_DIR}"

TIMESTAMP=$(date +%m%d%H%M%S)
LOG_FILE="${LOG_DIR}/${DP_TASK}_${TIMESTAMP}.log"

echo "Using Physical GPUs: $CUDA_VISIBLE_DEVICES"
# echo "Logical View: 0, 1"
echo "Num Processes: $NUM_PROCESSES"
echo "Training Log: $LOG_FILE"


# CUDA_VISIBLE_DEVICES=${GPU_ID} accelerate launch train.py \
# accelerate launch --gpu_ids ${GPU_ID} train.py \
#     --config-name=train_diffusion_unet_real_image_workspace \
#     task=${DP_TASK} \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=${DP_TASK}_${TIMESTAMP} \
#     logging.mode=${LOGGING_MODE}

accelerate launch \
    --gpu_ids 6,7 \
    --num_processes ${NUM_PROCESSES} \
    --main_process_port ${MASTER_PORT} \
    train.py \
    --config-name=train_diffusion_unet_real_image_workspace \
    task=${DP_TASK} \
    task.dataset_path=${DATASET_PATH} \
    task.name=${DP_TASK}_${TIMESTAMP} \
    logging.mode=${LOGGING_MODE} \
    > "${LOG_FILE}" 2>&1