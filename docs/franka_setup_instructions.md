# Franka Setup Instructions

This document provides instructions for teleoperation and policy rollout with the Franka Emika Panda robot using our VR teleoperation framework.

## Prerequisites

Before starting, ensure you have completed the following:

### Hardware
- Franka Emika Panda robot and FrankaHand
    - Configure your Franka Emika Panda Robot according to the official [libfranka documentation](https://frankaemika.github.io/docs/).  

- A desktop with realtime kernel installed apart from your own workstation.
    - This desktop is directly connected to the Franka Emika robot and is responsible for high-frequency (1000Hz) control signal transmission between the workstation and the Franka Emika.   
    - You can install realtime kernel on your desktop according to the official [libfranka documentation](https://frankaemika.github.io/docs/).  
    -  a NUC is recommended.
-  Meta Quest 3 VR headset.

### Network

- The desktop installed with the realtime kernel **must** be directly connected to the Franka Emika via Ethernet cable, ensuring a stable and fast network connection. 
  - For specific IP configuration steps and network diagnostics, please refer to the official [libfranka documentation](https://frankaemika.github.io/docs/).  
- The connection between your workstation and the desktop with realtime kernel could be wireless, but it's **encouraged** to connect directly with Ethernet cable for better connection.
- Ensure Meta Quest 3 and your workstation under the same network segment. 

### Software

#### Workstation Setup

1. We use **Polymetis** to write policy for Franka teleoperation. Polymetis supports Python3.8. It's recommended to create a Conda environment and directly install Polymetis from Conda.

```bash
conda create -n polymetis python=3.8
conda activate polymetis
conda install -c pytorch -c fair-robotics -c aihabitat -c conda-forge polymetis
```

For more information about Polymetis, please refer to the Polymetis [GitHub repository](https://github.com/facebookresearch/Polymetis) and the [official documentation](https://deepwiki.com/facebookresearch/fairo/5.3.3-polymetis-installation-and-configuration) for more details and advanced setup instructions.

**Note**: You should install Polymetis on both of your workstation and the desktop with realtime kernel!
 

## Teleoperation Setup

### Desktop with realtime kernel

1. Refer to the official [libfranka documentation](https://frankaemika.github.io/docs/) to power on the Franka Emika robot:  
    - Turn on the Franka Emika power switch.
    - Wait until the indicator light on the robot base turns yellow and stops blinking.
    - In your browser, enter the configured Franka Emika IP address to access the Franka Desk.
    - Confirm that both the Franka arm and gripper are properly connected.

2. On the Franka Desk, unlock the joints and wait until the indicator light on the robot base turns blue.

3. Click "Activate FCI" and keep this page open.

4. In two separate terminals, enter the following commands respectively:

```bash
# Start the polymetis robot interface server
launch_robot.py robot_client=franka_hardware

# Start the polymetis robot interface server
launch_gripper.py gripper=<franka_hand|robotiq_2f>
```
Wait for these two servers to ready.

### Workshop setup

The environment and the task have to be configured first and
then start service for teleoperating robots.

1. Environment and Task Configuration.
- **Environment Configuration.**
Edit [reactive_diffusion_policy/config/task/real_franka_env.yaml](../reactive_diffusion_policy/config/task/real_franka_env.yaml)
to configure the environment settings including `host_ip`, `robot_ip`, `vr_server_ip` and `calibration_path`.
- **Task Configuration.**
Create task config file which assigns the camera and sensor to use.
You can take [reactive_diffusion_policy/config/task/real_franka_env.yaml](../reactive_diffusion_policy/config/task/real_franka_env.yaml)
as an example.
Refer to [docs/customized_deployment_guide.md](../docs/customized_deployment_guide.md) for more details.
Edit [reactive_diffusion_policy/config/task/real_world_env.yaml](../reactive_diffusion_policy/config/real_world_env.yaml) by replacing the default task with your own task config.(e.g. real_franka_env)

1. Start service for teleoperation
   
```bash
# start teleoperation server
python teleop.py task=[task_config_file_name]
```

### Meta Quest 3

Follow the [user guide in our Unity Repo](https://github.com/xiaoxiaoxh/TactAR_APP/blob/master/Docs/User_Guide.md) to run the TactAR APP.

## Additional Notes

- We use the Franka Research 3 with the FrankaHand gripper. Other hardwares should also work in principle. 
- Theoretically, the closer the control signal frequency sent by the FrankaServer is to 1000Hz, the more controllable the policy will be. If your hardware allows, you may increase the "frequency" parameter in [franka_server.py](rreactive_diffusion_policy/real_world/robot/franka_server.py).
  

For troubleshooting and advanced usage, please refer to the full documentation or contact the maintainers.
