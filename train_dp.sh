# #!/bin/bash

# CUDA_VISIBLE_DEVICES=0 accelerate launch train.py \
#     --config-name=train_diffusion_unet_real_image_workspace \
#     task=real_peel_image_gelsight_emb_dp_absolute_12fps \
#     task.dataset_path=/home/wendi/Desktop/record_data/peel_v3_downsample2_zarr \
#     task.name=real_peel_image_gelsight_emb_dp_absolute_12fps \
#     logging.mode=online

#!/bin/bash

GPU_ID=0

# ==== 以后只改这两行 ====
DATASET_PATH="/home/wmx/myspace/RDP/data/plug_in_downsample1_zarr"
DP_TASK="wmx_real_image_dp_absolute_12fps"
# ========================

LOGGING_MODE="online"
TIMESTAMP=$(date +%m%d%H%M%S)

CUDA_VISIBLE_DEVICES=${GPU_ID} accelerate launch train.py \
    --config-name=train_diffusion_unet_real_image_workspace \
    task=${DP_TASK} \
    task.dataset_path=${DATASET_PATH} \
    task.name=${DP_TASK}_${TIMESTAMP} \
    logging.mode=${LOGGING_MODE}
