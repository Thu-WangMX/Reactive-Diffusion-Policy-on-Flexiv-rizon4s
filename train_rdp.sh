#!/bin/bash
# 增强鲁棒性：e(出错终止) + u(未定义变量报错) + o pipefail(管道错误传递)
set -euo pipefail  

# ================= 配置区域 =================
# 物理显卡ID (确保这几张卡是空闲的)
GPU_IDS="2,3,4,5"
NUM_PROCESSES=4
MASTER_PORT=29501

# 数据集绝对路径（已正确使用，保留）
DATASET_PATH="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/open_drawer/open_drawer_stream_downsample1_zarr"
AT_TASK="wmx_real_open_drawer_image_wrench_at_24fps"
LDP_TASK="wmx_real_open_drawer_three_cam_image_wrench_ldp_24fps"
AT_CONFIG="at_peel"

# 日志设置（统一用绝对路径，避免执行目录问题）
LOG_DIR="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/rdp_train_logs" 
LOGGING_MODE="online" # 修复：注释与值一致，避免wandb在线卡住
# ===========================================

# 确保日志目录存在（绝对路径）
mkdir -p "${LOG_DIR}"
# 清理wandb缓存（加-f强制删除，避免目录非空报错）
rm -rf wandb/ || true  
TIMESTAMP=$(date +%m%d%H%M%S)

echo -e "\033[32m=== RDP Pipeline Started (${TIMESTAMP}) ===\033[0m"

# # ==========================================================
# # Stage 1: 训练 Action Tokenizer (AT)
# # ==========================================================
# LOG_FILE_AT="${LOG_DIR}/${AT_TASK}_${TIMESTAMP}.log"
# echo -e "\033[34m[Stage 1] Training AT... (Logs: ${LOG_FILE_AT})\033[0m"

# # 关键修复：1. 用绝对路径执行python；2. 日志同时输出到终端+文件；3. 显式指定CUDA
# CUDA_VISIBLE_DEVICES=${GPU_IDS} python /root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/train.py \
#     --config-name=train_at_workspace \
#     task=${AT_TASK} \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=${AT_TASK}_${TIMESTAMP} \
#     at=${AT_CONFIG} \
#     logging.mode=${LOGGING_MODE} \
#     > >(tee -a "${LOG_FILE_AT}") 2>&1 || { 
#         echo -e "\033[31m❌ Stage 1 Failed! See ${LOG_FILE_AT}\033[0m"; 
#         exit 1; 
#     }

# echo -e "\033[32m✅ Stage 1 Completed Successfully.\033[0m"

# # ==========================================================
# # 中间检查：寻找 Checkpoint (核心修复：绝对路径+按时间排序)
# # ==========================================================
# # 修复1：改用绝对路径（根据你的ckpt示例路径调整）
# SEARCH_PATH="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs"
# # 修复2：find按修改时间排序，取最新的匹配目录（避免多目录匹配错误）
# LATEST_DIR=$(find "${SEARCH_PATH}" -maxdepth 3 -type d -name "*${AT_TASK}_${TIMESTAMP}*" | sort -r | head -n 1)

# # 增强检查：空值提示+打印搜索路径
# if [ -z "${LATEST_DIR:-}" ]; then
#     echo -e "\033[31m❌ Error: Could not find output directory for ${AT_TASK}_${TIMESTAMP}\033[0m"
#     echo -e "\033[31mSearch Path: ${SEARCH_PATH}\033[0m"
#     # 打印搜索路径下的所有目录，方便排查
#     ls -l "${SEARCH_PATH}" || true
#     exit 1
# fi

# # 转换为绝对路径 (解决 FileNotFoundError 的关键)
# AT_LOAD_DIR=$(readlink -f "${LATEST_DIR}/checkpoints/latest.ckpt")

AT_LOAD_DIR="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs/2026.02.13/17.18.13_train_vae_wmx_real_open_drawer_image_wrench_at_24fps_0213171811/checkpoints/latest.ckpt"

# # 增强检查：文件不存在时打印目录内容
# if [ ! -f "${AT_LOAD_DIR}" ]; then
#     echo -e "\033[31m❌ Error: Checkpoint file missing at ${AT_LOAD_DIR}\033[0m"
#     ls -l "${LATEST_DIR}/checkpoints/" || true
#     exit 1
# else
#     echo -e "\033[32m✅ Checkpoint found: ${AT_LOAD_DIR}\033[0m"
# fi



# ==========================================================
# Stage 2: 训练 Latent Diffusion Policy (LDP)
# ==========================================================
LOG_FILE_LDP="${LOG_DIR}/${LDP_TASK}_${TIMESTAMP}.log"
echo -e "\033[34m[Stage 2] Training LDP... (Logs: ${LOG_FILE_LDP})\033[0m"

# 修复：保留CUDA_VISIBLE_DEVICES，避免accelerate GPU分配冲突
# unset CUDA_VISIBLE_DEVICES 

# 核心修复：1. accelerate指定绝对路径；2. 日志实时输出；3. 参数用引号包裹避免空格问题
accelerate launch \
    --gpu_ids "${GPU_IDS}" \
    --num_processes "${NUM_PROCESSES}" \
    --main_process_port "${MASTER_PORT}" \
    /root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/train.py \
    --config-name=train_latent_diffusion_unet_real_image_workspace \
    task="${LDP_TASK}" \
    task.dataset_path="${DATASET_PATH}" \
    task.name="${LDP_TASK}_${TIMESTAMP}" \
    at="${AT_CONFIG}" \
    at_load_dir="${AT_LOAD_DIR}" \
    logging.mode="${LOGGING_MODE}" \
    > >(tee -a "${LOG_FILE_LDP}") 2>&1 || { 
        echo -e "\033[31m❌ Stage 2 Failed! See ${LOG_FILE_LDP}\033[0m"; 
        exit 1; 
    }

echo -e "\033[32m🎉 All Stages Completed! Logs in ${LOG_DIR}\033[0m"