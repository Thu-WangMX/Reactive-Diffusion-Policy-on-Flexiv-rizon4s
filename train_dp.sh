#!/bin/bash
set -euo pipefail  # 增强鲁棒性，出错立即终止

# ================= 核心配置区域 (仅改这里) =================
export CUDA_VISIBLE_DEVICES=0,1  # 物理显卡ID
NUM_PROCESSES=2                  # 进程数=显卡数
MASTER_PORT=29500                # 主进程端口，防冲突
DATASET_PATH="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/open_drawer/open_drawer_stream_downsample2_zarr"  # 数据集路径
DP_TASK="wmx_real_open_drawer_image_dp_absolute_12fps"  # 训练任务名
LOGGING_MODE="online"  # 关键修复：去掉等号两边的空格
# ============================================================

# 自动生成日志/任务名（无需修改）
LOG_ROOT_DIR="dp_train_logs"
TIMESTAMP=$(date +%m%d%H%M%S)
TASK_FULL_NAME="${DP_TASK}_${TIMESTAMP}"
LOG_DIR="${LOG_ROOT_DIR}/${TASK_FULL_NAME}"
LOG_FILE="${LOG_DIR}/train.log"
mkdir -p "${LOG_DIR}"  # 确保日志目录存在

# 核心训练命令（Hydra语法正确，无解析错误）
accelerate launch \
    --gpu_ids 0,1 \
    --num_processes "${NUM_PROCESSES}" \
    --main_process_port "${MASTER_PORT}" \
    train.py \
    --config-name=train_diffusion_unet_real_image_workspace \
    task="${DP_TASK}" \
    task.dataset_path="${DATASET_PATH}" \
    task.name="${TASK_FULL_NAME}" \
    logging.mode="${LOGGING_MODE}" \
    > "${LOG_FILE}" 2>&1

# 训练结束提示
echo -e ""
echo -e "========================================"
echo -e "训练结束: \033[31m$(date +'%Y-%m-%d %H:%M:%S')\033[0m"
echo -e "训练日志: \033[32m${LOG_FILE}\033[0m"
echo -e "Wandb任务名: \033[32m${TASK_FULL_NAME}\033[0m"
echo -e "========================================"