#!/usr/bin/env bash
set -e

# ================= 配置区域 =================
GPU_IDS="0,1,2,3"
NUM_PROCESSES=4
MASTER_PORT=29502

DATASET_PATH="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"
#DATASET_PATH="/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"

OURS_TASK="wmx_paep_real_wiping_board_image_dp_absolute_24fps"
WS_CONFIG="train_paep_diffusion_unet_real_image_workspace"

PAEP_CKPT="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/best.pt"
#PAEP_CKPT="/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_runs_0124/best.pt"

LOGGING_MODE="online"   # or offline
LOG_DIR="train_logs_ours"
# ===========================================

mkdir -p "${LOG_DIR}"
TIMESTAMP=$(date +%m%d%H%M%S)
LOG_FILE="${LOG_DIR}/${OURS_TASK}_${TIMESTAMP}.log"




# #修改norm.pkl找不到的问题
# BASE_NORM="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs/2026.01.21/15.52.59_train_paep_diffusion_unet_image_force_wmx_paep_real_wiping_board_image_dp_absolute_24fps_0121155245/normalizer.pkl"

RUN_DIR="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs/${OURS_TASK}/${TIMESTAMP}"
#RUN_DIR="/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs/${OURS_TASK}/${TIMESTAMP}"
mkdir -p "${RUN_DIR}"
# cp "${BASE_NORM}" "${RUN_DIR}/normalizer.pkl"


echo "Using GPUs: ${GPU_IDS}"
echo "Num processes: ${NUM_PROCESSES}"
echo "Port: ${MASTER_PORT}"
echo "Dataset: ${DATASET_PATH}"
echo "Task: ${OURS_TASK}"
echo "Workspace config: ${WS_CONFIG}"
echo "PAEP ckpt: ${PAEP_CKPT}"
echo "Log: ${LOG_FILE}"

#unset CUDA_VISIBLE_DEVICES
export CUDA_VISIBLE_DEVICES="${GPU_IDS}"


accelerate launch \
  --gpu_ids ${GPU_IDS} \
  --num_processes ${NUM_PROCESSES} \
  --main_process_port ${MASTER_PORT} \
  --mixed_precision="bf16" \
  train.py \
  --config-name=${WS_CONFIG} \
  hydra.run.dir=${RUN_DIR} \
  task=${OURS_TASK} \
  task.dataset_path=${DATASET_PATH} \
  task.name=${OURS_TASK}_${TIMESTAMP} \
  policy.paep_ckpt=${PAEP_CKPT} \
  logging.mode=${LOGGING_MODE} \
  > "${LOG_FILE}" 2>&1

echo "✅ Done. Log saved to ${LOG_FILE}"
