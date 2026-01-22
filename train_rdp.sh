# #!/bin/bash

# GPU_ID=0

# TASK_NAME="peel"
# DATASET_PATH="/home/wendi/Desktop/record_data/peel_v3_downsample1_zarr"
# LOGGING_MODE="online"

# TIMESTAMP=$(date +%m%d%H%M%S)
# SEARCH_PATH="./data/outputs"

# # Stage 1: Train Asymmetric Tokenizer
# echo "Stage 1: training Asymmetric Tokenizer..."
# CUDA_VISIBLE_DEVICES=${GPU_ID} python train.py \
#     --config-name=train_at_workspace \
#     task=real_${TASK_NAME}_image_gelsight_emb_at_24fps \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=real_${TASK_NAME}_image_gelsight_emb_at_24fps_${TIMESTAMP} \
#     at=at_peel \
#     logging.mode=${LOGGING_MODE}

# # find the latest checkpoint
# echo ""
# echo "Searching for the latest AT checkpoint..."
# AT_LOAD_DIR=$(find "${SEARCH_PATH}" -maxdepth 2 -path "*${TIMESTAMP}*" -type d)/checkpoints/latest.ckpt

# if [ ! -f "${AT_LOAD_DIR}" ]; then
#     echo "Error: VAE checkpoint not found at ${AT_LOAD_DIR}"
#     exit 1
# fi

# # Stage 2: Train Latent Diffusion Policy
# echo ""
# echo "Stage 2: training Latent Diffusion Policy..."
# CUDA_VISIBLE_DEVICES=${GPU_ID} accelerate launch train.py \
#     --config-name=train_latent_diffusion_unet_real_image_workspace \
#     task=real_${TASK_NAME}_image_gelsight_emb_ldp_24fps \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=real_${TASK_NAME}_image_gelsight_emb_ldp_24fps_${TIMESTAMP} \
#     at=at_peel \
#     at_load_dir=${AT_LOAD_DIR} \
#     logging.mode=${LOGGING_MODE}


# #!/bin/bash

# # ================= 配置区域 =================
# # 1. 物理显卡ID：只填你想用的那两张卡 (例如 4 和 5)
# GPU_IDS="4,5"

# # 2. 进程数 (Stage 2 使用)
# NUM_PROCESSES=2

# # 3. 端口号：【关键】必须和 train_dp.sh 不一样，防止冲突
# MASTER_PORT=29501

# # 4. 任务配置
# DATASET_PATH="/work/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"
# AT_TASK="wmx_real_wiping_board_image_wrench_at_24fps"
# LDP_TASK="wmx_real_wiping_board_two_cam_image_wrench_ldp_24fps"
# AT_CONFIG="at_peel"

# LOGGING_MODE="online"
# LOG_DIR="rdp_train_logs" # 单独的 RDP 日志文件夹
# mkdir -p "${LOG_DIR}"
# TIMESTAMP=$(date +%m%d%H%M%S)


# SEARCH_PATH="./data/outputs"

# #Stage 1: 训练 AT
# CUDA_VISIBLE_DEVICES=${GPU_ID} python train.py \
#     --config-name=train_at_workspace \
#     task=${AT_TASK} \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=${AT_TASK}_${TIMESTAMP} \
#     at=${AT_CONFIG} \
#     logging.mode=${LOGGING_MODE}

# # 找刚训练好的 AT checkpoint
# AT_LOAD_DIR=$(find "${SEARCH_PATH}" -maxdepth 2 -path "*${TIMESTAMP}*" -type d)/checkpoints/latest.ckpt


# # AT_LOAD_DIR="./data/outputs/2025.11.29/19.14.55_train_vae_wmx_real_plugin_image_wrench_at_24fps_1129191454/checkpoints/latest.ckpt"

# # Stage 2: 训练 LDP/RDP
# CUDA_VISIBLE_DEVICES=${GPU_ID} accelerate launch train.py \
#     --config-name=train_latent_diffusion_unet_real_image_workspace \
#     task=${LDP_TASK} \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=${LDP_TASK}_${TIMESTAMP} \
#     at=${AT_CONFIG} \
#     at_load_dir=${AT_LOAD_DIR} \
#     logging.mode=${LOGGING_MODE}

set -e
# ================= 配置区域 =================
# 物理显卡ID (确保这几张卡是空闲的)
GPU_IDS="4,5"
NUM_PROCESSES=2
MASTER_PORT=29501

DATASET_PATH="/work/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"
AT_TASK="wmx_real_wiping_board_image_wrench_at_24fps"
LDP_TASK="wmx_real_wiping_board_two_cam_image_wrench_ldp_24fps"
AT_CONFIG="at_peel"

# 日志设置
LOG_DIR="rdp_train_logs" 
LOGGING_MODE="online" # 强制离线，防止卡住
# ===========================================

mkdir -p "${LOG_DIR}"
rm -rf wandb/ # 清理缓存
TIMESTAMP=$(date +%m%d%H%M%S)

echo "=== RDP Pipeline Started ==="

# # ==========================================================
# # Stage 1: 训练 Action Tokenizer (AT)
# # ==========================================================
# LOG_FILE_AT="${LOG_DIR}/${AT_TASK}_${TIMESTAMP}.log"
# echo "[Stage 1] Training AT... (Logs: ${LOG_FILE_AT})"

# # ⚠️ 注意：这里显式检查 python 的退出码
# # 如果 python 报错， set -e 会自动停止，但为了保险我们加个 || exit 1
# CUDA_VISIBLE_DEVICES=${GPU_IDS} python train.py \
#     --config-name=train_at_workspace \
#     task=${AT_TASK} \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=${AT_TASK}_${TIMESTAMP} \
#     at=${AT_CONFIG} \
#     logging.mode=${LOGGING_MODE} \
#     > "${LOG_FILE_AT}" 2>&1 || { echo "❌ Stage 1 Failed! See ${LOG_FILE_AT}"; exit 1; }

# echo "✅ Stage 1 Completed Successfully."

# # ==========================================================
# # 中间检查：寻找 Checkpoint (使用绝对路径)
# # ==========================================================
# SEARCH_PATH="./data/outputs"
# # 寻找匹配任务名的最新文件夹
# LATEST_DIR=$(find "${SEARCH_PATH}" -maxdepth 2 -type d -name "${AT_TASK}_${TIMESTAMP}" | head -n 1)


# if [ -z "$LATEST_DIR" ]; then
#     echo "❌ Error: Could not find output directory for ${AT_TASK}_${TIMESTAMP}"
#     exit 1
# fi

# # 转换为绝对路径 (解决 FileNotFoundError 的关键)
# AT_LOAD_DIR=$(readlink -f "${LATEST_DIR}/checkpoints/latest.ckpt")

AT_LOAD_DIR='/work/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs/2026.01.17/20.53.09_train_vae_wmx_real_wiping_board_image_wrench_at_24fps_0117205307/checkpoints/latest.ckpt'

# if [ ! -f "${AT_LOAD_DIR}" ]; then
#     echo "❌ Error: Checkpoint file missing at ${AT_LOAD_DIR}"
#     exit 1
# else
#     echo "✅ Checkpoint found: ${AT_LOAD_DIR}"
# fi

# ==========================================================
# Stage 2: 训练 Latent Diffusion Policy (LDP)
# ==========================================================
LOG_FILE_LDP="${LOG_DIR}/${LDP_TASK}_${TIMESTAMP}.log"
echo "[Stage 2] Training LDP... (Logs: ${LOG_FILE_LDP})"

# 清除环境变量，防止干扰 accelerate
unset CUDA_VISIBLE_DEVICES

accelerate launch \
    --gpu_ids ${GPU_IDS} \
    --num_processes ${NUM_PROCESSES} \
    --main_process_port ${MASTER_PORT} \
    train.py \
    --config-name=train_latent_diffusion_unet_real_image_workspace \
    task=${LDP_TASK} \
    task.dataset_path=${DATASET_PATH} \
    task.name=${LDP_TASK}_${TIMESTAMP} \
    at=${AT_CONFIG} \
    at_load_dir=${AT_LOAD_DIR} \
    logging.mode=${LOGGING_MODE} \
    > "${LOG_FILE_LDP}" 2>&1 || { echo "❌ Stage 2 Failed! See ${LOG_FILE_LDP}"; exit 1; }

echo "🎉 All Stages Completed! Logs in ${LOG_DIR}"