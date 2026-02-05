# # #!/bin/bash

# # CUDA_VISIBLE_DEVICES=0 accelerate launch train.py \
# #     --config-name=train_diffusion_unet_real_image_workspace \
# #     task=real_peel_image_gelsight_emb_dp_absolute_12fps \
# #     task.dataset_path=/home/wendi/Desktop/record_data/peel_v3_downsample2_zarr \
# #     task.name=real_peel_image_gelsight_emb_dp_absolute_12fps \
# #     logging.mode=online




# # ================= 配置区域 =================
# # 1. 物理显卡ID：只填你想用的那两张卡 (例如 6 和 7)
# # 这一步将物理卡6,7映射为逻辑卡0,1
# export CUDA_VISIBLE_DEVICES=0,1,2,3

# # 2. 进程数：必须等于上面显卡的数量 (例如 2)
# NUM_PROCESSES=4

# # 3. 端口号：防止冲突 (默认 29500)
# MASTER_PORT=29500

# # ==== 以后只改这两行 ====
# #DATASET_PATH="/home/wmx/myspace/RDP/data/plug_in_downsample1_zarr"
# DATASET_PATH="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"
# DP_TASK="wmx_real_wiping_board_image_dp_absolute_12fps"
# # ========================

# LOGGING_MODE="online"

# # 5. 日志存放目录 (会自动创建)
# LOG_DIR="dp_train_logs"
# # ===========================================

# # 准备环境
# mkdir -p "${LOG_DIR}"

# TIMESTAMP=$(date +%m%d%H%M%S)
# LOG_FILE="${LOG_DIR}/${DP_TASK}_${TIMESTAMP}.log"

# echo "Using Physical GPUs: $CUDA_VISIBLE_DEVICES"
# # echo "Logical View: 0, 1"
# echo "Num Processes: $NUM_PROCESSES"
# echo "Training Log: $LOG_FILE"


# # CUDA_VISIBLE_DEVICES=${GPU_ID} accelerate launch train.py \
# # accelerate launch --gpu_ids ${GPU_ID} train.py \
# #     --config-name=train_diffusion_unet_real_image_workspace \
# #     task=${DP_TASK} \
# #     task.dataset_path=${DATASET_PATH} \
# #     task.name=${DP_TASK}_${TIMESTAMP} \
# #     logging.mode=${LOGGING_MODE}

# accelerate launch \
#     --gpu_ids 0,1,2,3 \
#     --num_processes ${NUM_PROCESSES} \
#     --main_process_port ${MASTER_PORT} \
#     train.py \
#     --config-name=train_diffusion_unet_real_image_workspace \
#     task=${DP_TASK} \
#     task.dataset_path=${DATASET_PATH} \
#     task.name=${DP_TASK}_${TIMESTAMP} \
#     logging.mode=${LOGGING_MODE} \
#     > "${LOG_FILE}" 2>&1
#!/bin/bash
set -euo pipefail  # 增强鲁棒性，出错立即终止

# ================= 核心配置区域 (仅改这里) =================
export CUDA_VISIBLE_DEVICES=0  # 物理显卡ID
NUM_PROCESSES=1                      # 进程数=显卡数
MASTER_PORT=29500                    # 主进程端口，防冲突
DATASET_PATH="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"  # 数据集路径
DP_TASK="wmx_real_wiping_board_image_dp_absolute_12fps"  # 训练任务名
# 实际存在的normalizer.pkl路径（不要改！）
REAL_NORMALIZER_PATH="/home/ubuntu/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/data/outputs/2026.01.28/00.42.40_train_diffusion_unet_image_wmx_real_wiping_board_image_dp_absolute_12fps_0128004209/normalizer.pkl"
LOGGING_MODE="online"                # wandb模式：online/offline/disabled
# ============================================================

# 自动生成日志/任务名（无需修改）
LOG_ROOT_DIR="dp_train_logs"
TIMESTAMP=$(date +%m%d%H%M%S)
TASK_FULL_NAME="${DP_TASK}_${TIMESTAMP}"
LOG_DIR="${LOG_ROOT_DIR}/${TASK_FULL_NAME}"
LOG_FILE="${LOG_DIR}/train.log"
mkdir -p "${LOG_DIR}"  # 确保日志目录存在

# ================= 关键：创建软链接（核心修复，不改源码的关键） =================
# 代码硬编码加载的路径（新任务目录下的normalizer.pkl）
CODE_LOAD_NORMALIZER_PATH="${LOG_DIR}/normalizer.pkl"
# 创建软链接：将代码要加载的路径，映射到实际存在的文件
ln -s "${REAL_NORMALIZER_PATH}" "${CODE_LOAD_NORMALIZER_PATH}"
echo -e "✅ 已创建软链接：\n${CODE_LOAD_NORMALIZER_PATH} -> ${REAL_NORMALIZER_PATH}"
# ==================================================================================

# 打印训练配置（彩色高亮，方便核对）
echo -e "\n========================================"
echo -e "          训练配置信息"
echo -e "========================================"
echo -e "物理显卡ID: \033[32m$CUDA_VISIBLE_DEVICES\033[0m (映射为逻辑卡0-3)"
echo -e "训练进程数: \033[32m$NUM_PROCESSES\033[0m (与显卡数一致)"
echo -e "主进程端口: \033[32m$MASTER_PORT\033[0m"
echo -e "数据集路径: \033[32m$DATASET_PATH\033[0m"
echo -e "训练任务名: \033[32m$TASK_FULL_NAME\033[0m"
echo -e "实际归一化器路径: \033[32m$REAL_NORMALIZER_PATH\033[0m"
echo -e "代码加载路径(软链接): \033[32m$CODE_LOAD_NORMALIZER_PATH\033[0m"
echo -e "日志保存目录: \033[32m$LOG_DIR\033[0m"
echo -e "Wandb模式: \033[32m$LOGGING_MODE\033[0m"
echo -e "========================================"
echo -e "开始时间: \033[32m$(date +'%Y-%m-%d %H:%M:%S')\033[0m"
echo -e "========================================"
echo -e ""

# 核心训练命令（Hydra语法正确，无解析错误）
accelerate launch \
    --gpu_ids 0,1,2,3 \
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