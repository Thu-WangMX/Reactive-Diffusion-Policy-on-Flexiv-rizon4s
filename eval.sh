#!/bin/bash

# DP w. GelSight Emb. (Peeling)
python eval_real_robot_flexiv.py \
     --config-name train_diffusion_unet_real_image_workspace \
     task=wmx_real_image_dp_absolute_12fps \
     +task.env_runner.output_dir=/home/wmx/myspace/RDP/video \
     +ckpt_path=/home/wmx/myspace/RDP/ckpt/checkpoints_20251129/data/outputs/2025.11.29/19.43.37_train_diffusion_unet_image_wmx_real_image_dp_absolute_12fps_1129194331/checkpoints/latest.ckpt

# RDP w. Force (Peeling)
# python eval_real_robot_flexiv.py \
#       --config-name train_latent_diffusion_unet_real_image_workspace \
#       task=wmx_real_plugin_two_cam_image_wrench_ldp_24fps \
#       at=at_peel \
#       at_load_dir=/home/wmx/myspace/RDP/ckpt/checkpoints_20251201/data/outputs/2025.11.30/23.38.01_train_vae_wmx_real_plugin_image_wrench_at_24fps_1130233759/checkpoints/latest.ckpt \
#       +task.env_runner.output_dir=/home/wmx/myspace/RDP/video \
#       +ckpt_path=/home/wmx/myspace/RDP/ckpt/checkpoints_20251201/data/outputs/2025.12.01/10.59.45_train_latent_diffusion_unet_image_wmx_real_plugin_two_cam_image_wrench_ldp_24fps_1201105939/checkpoints/latest.ckpt \
      