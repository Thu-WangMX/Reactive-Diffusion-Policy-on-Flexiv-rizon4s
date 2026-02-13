#!/bin/bash


# 1. 定义策略类型 (如果外部没传，默认为 DP)
POLICY_TYPE=${1:-"DP"}

echo "正在评估策略类型: ${POLICY_TYPE}"


# # #DP w. GelSight Emb. (Peeling)
# python eval_real_robot_flexiv.py \
#      --config-name train_diffusion_unet_real_image_workspace \
#      task=wmx_real_plug_in_usb_image_dp_absolute_12fps \
#      +task.env_runner.output_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video \
#      +ckpt_path=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/DP/plug_in_usb/epoch0570-train_loss0.001.ckpt


# # # RDP w. Force (Peeling)
# python eval_real_robot_flexiv.py \
#       --config-name train_latent_diffusion_unet_real_image_workspace \
#       task=wmx_real_plug_in_usb_three_cam_image_wrench_ldp_24fps \
#       at=at_peel \
#       at_load_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/RDP/plug_in_usb/at/epoch0590-train_loss0.006354.ckpt \
#       +task.env_runner.output_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video \
#       +ckpt_path=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/RDP/plug_in_usb/ldp/epoch0300-train_loss0.043.ckpt
      




# #PAEP-guided dual-gated vision-force fusion diffusion policy
python eval_ours_real_robot_flexiv.py \
  --config-name train_paep_diffusion_unet_real_image_workspace \
  task=wmx_paep_real_plugin_usb_image_dp_absolute_24fps \
  +task.env_runner.output_dir=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video \
  +diff_ckpt_path=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/OURS_v4_0128/plug_in_usb/epoch_0400.ckpt \
  +paep_ckpt_path=/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/ckpt/paep_plug_in_usb_0210/best.pt \
  +num_inference_steps=8

