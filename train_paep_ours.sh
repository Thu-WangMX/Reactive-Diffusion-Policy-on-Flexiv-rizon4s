#!/usr/bin/env bash
set -e  # 遇到错误立即停止

# ==================== 全局配置 ====================
# 项目根目录
PROJECT_ROOT="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s"

# 日志目录
LOG_DIR="${PROJECT_ROOT}/train_logs_autostage"
mkdir -p "${LOG_DIR}"
TIMESTAMP=$(date +%m%d%H%M%S)


# ==========================================================
#                      STAGE 1: PAEP
# ==========================================================
# --- PAEP 训练配置 (如果你想重新全自动训练，解开下面的注释) ---
PAEP_SCRIPT="${PROJECT_ROOT}/PAEP/train/train_paep.py"
PAEP_CONFIG="${PROJECT_ROOT}/PAEP/config/paep_plug_in_usb.py" 

echo "########################################################"
echo "🚀 [Stage 1] PAEP Model (Currently Skipped/Hardcoded)..."
echo "########################################################"

# 【手动指定绝对路径】在这里填入你已经训好的 PAEP best.pt 路径
# 以后如果换了新的模型，直接改这一行就行！
PAEP_BEST_CKPT="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/ckpt/paep_plug_in_usb_0210/best.pt"

# -----------------------------------------------------------
# [被注释掉的自动训练代码] - 未来需要两阶段连跑时，解开这些注释
# -----------------------------------------------------------
# get_paep_save_dir() {
#     python3 -c "
# import importlib.util, sys
# spec = importlib.util.spec_from_file_location('cfg', '$PAEP_CONFIG')
# mod = importlib.util.module_from_spec(spec)
# spec.loader.exec_module(mod)
# print(getattr(mod, 'SAVE_DIR', ''))"
# }
# 
# PAEP_SAVE_DIR=$(get_paep_save_dir)
# PAEP_BEST_CKPT="${PAEP_SAVE_DIR}/best.pt"
# 
# unset WANDB_RUN_ID
# unset WANDB_RESUME
# export CUDA_VISIBLE_DEVICES=0
# python3 "${PAEP_SCRIPT}" --cfg "${PAEP_CONFIG}" 2>&1 | tee "${LOG_DIR}/stage1_paep_${TIMESTAMP}.log"
# 
# if [ ! -f "${PAEP_BEST_CKPT}" ]; then
#     echo "❌ Error: PAEP training finished but '${PAEP_BEST_CKPT}' not found!"
#     exit 1
# fi
# echo "✅ [Stage 1] PAEP Training Done! Best model saved at: ${PAEP_BEST_CKPT}"
# sleep 5
# -----------------------------------------------------------

# 检查你上面写死的路径文件到底存不存在，防止低级错误
if [ ! -f "${PAEP_BEST_CKPT}" ]; then
    echo "❌ Error: 找不到 PAEP 模型文件! 请检查路径是否正确: '${PAEP_BEST_CKPT}'"
    exit 1
fi
echo "✅ 使用现有的 PAEP 模型: ${PAEP_BEST_CKPT}"
echo ""


# ==========================================================
#                 STAGE 2: Diffusion Policy
# ==========================================================
# --- DP 配置 ---
DP_GPU_IDS="0,1,2,3,4,5"
DP_NUM_PROCESSES=6
DP_PORT=29505

DP_TASK="wmx_paep_real_plugin_usb_image_dp_absolute_24fps"
DP_WS_CONFIG="train_paep_diffusion_unet_real_image_workspace"
DP_DATASET="/root/workspace/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_usb_stream_downsample1_zarr"

echo "########################################################"
echo "🚀 [Stage 2] Start Training Diffusion Policy with PAEP..."
echo "########################################################"

# 1. 设置 DP 的输出目录
DP_RUN_DIR="${PROJECT_ROOT}/data/outputs/${DP_TASK}/${TIMESTAMP}"
mkdir -p "${DP_RUN_DIR}"
DP_LOG_FILE="${LOG_DIR}/stage2_dp_${TIMESTAMP}.log"

echo "Using GPUs: ${DP_GPU_IDS}"
echo "DP Dataset: ${DP_DATASET}"
echo "DP Run Dir: ${DP_RUN_DIR}"

# 2. 设置环境
export CUDA_VISIBLE_DEVICES="${DP_GPU_IDS}"
# 清理 Wandb，强制生成新 ID，防止和以前的实验混在一起
unset WANDB_RUN_ID
unset WANDB_RESUME

# 3. 启动加速训练 (Accelerate)
accelerate launch \
  --gpu_ids "${DP_GPU_IDS}" \
  --num_processes "${DP_NUM_PROCESSES}" \
  --main_process_port "${DP_PORT}" \
  --mixed_precision="bf16" \
  train.py \
  --config-name="${DP_WS_CONFIG}" \
  hydra.run.dir="${DP_RUN_DIR}" \
  task="${DP_TASK}" \
  task.dataset_path="${DP_DATASET}" \
  task.name="${DP_TASK}_${TIMESTAMP}" \
  policy.paep_ckpt="${PAEP_BEST_CKPT}" \
  logging.mode="online" \
  logging.resume=False \
  training.resume=False \
  2>&1 | tee "${DP_LOG_FILE}"

echo "########################################################"
echo "✅ All Done! Diffusion Policy training completed successfully."
echo "📜 Logs are in: ${LOG_DIR}"
echo "########################################################"