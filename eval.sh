#!/bin/bash


# 1. 定义策略类型 (如果外部没传，默认为 DP)
POLICY_TYPE=${1:-"DP"}

echo "正在评估策略类型: ${POLICY_TYPE}"


# DP w. GelSight Emb. (Peeling)
python eval_real_robot_flexiv.py \
     --config-name train_diffusion_unet_real_image_workspace \
     task=wmx_real_wiping_board_image_dp_absolute_12fps \
     +task.env_runner.output_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video \
     +ckpt_path=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/DP/wiping_board/latest.ckpt

# RDP w. Force (Peeling)
# python eval_real_robot_flexiv.py \
#       --config-name train_latent_diffusion_unet_real_image_workspace \
#       task=wmx_real_wiping_board_two_cam_image_wrench_ldp_24fps \
#       at=at_peel \
#       at_load_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/RDP/wiping_board/vae_latest.ckpt \
#       +task.env_runner.output_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video \
#       +ckpt_path=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/RDP/wiping_board/latest.ckpt 
      