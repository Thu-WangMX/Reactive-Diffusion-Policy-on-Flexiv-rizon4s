
# Reactive Diffusion Policy on Flexiv Rizon4s

This repository is based on **Reactive Diffusion Policy (RDP)** and extends it to a **Flexiv Rizon4s** robot with a Flexiv-GN01 gripper and dual Intel RealSense cameras. It supports VR-based teleoperation and data collection for downstream policy learning.

This README focuses on **how to collect teleoperation data (demonstrations)** using:

- Flexiv Rizon4s robot
- A Linux PC (Ubuntu)
- Two Intel RealSense cameras
- A VR headset running the TactAR app

---

## 1. Hardware & Network Setup

1. **Robot ↔ PC (wired)**  
   - Connect the PC to the Flexiv controller using an Ethernet cable.  
   - Typical IPs (example):  
     - PC (wired): `192.168.2.101`  
     - Flexiv robot: `192.168.2.100`

2. **PC ↔ VR (Wi-Fi)**  
   - Connect both the **PC** and the **VR headset** to the **same Wi-Fi network**.  
   - The PC will receive VR tracking data over this Wi-Fi.

3. **RealSense cameras**  
   - Connect two Intel RealSense cameras to the PC via USB.
   - Use `realsense-viewer` to confirm that both cameras output RGB + Depth at **640×480** resolution.

---

## 2. Python Environment

All teleoperation and data collection scripts should run in the `rdp_venv` Python virtual environment (not a conda environment).

In **each terminal**, run:

```bash
conda deactivate          # make sure conda env is not active
source rdp_venv/bin/activate
cd ~/reactive_diffusion_policy
```

Replace `~/reactive_diffusion_policy` with your actual repo path if different.

---

## 3. Real Robot Configuration (real_robot_env.yaml)

The key configuration file is:

```text
reactive_diffusion_policy/reactive_diffusion_policy/config/task/real_robot_env.yaml
```

A typical configuration looks like:

```yaml
robot_name: 'flexiv_rizon'

robot_server:
  host_ip: "192.168.2.101"
  port: 8092
  left_robot_ip: "192.168.2.100"
  robot_sn: "Rizon4s-062958"
  gripper_name: "Flexiv-GN01"
  right_robot_ip: "192.168.2.111"
  bimanual_teleop: &bimanual_teleop False

publisher:
  # vr_server_ip: &vr_server_ip '192.168.3.20' # example for another Wi-Fi
  vr_server_ip: &vr_server_ip '192.168.43.201'
  vr_server_port: 10002
  robot_publisher:
    robot_server_ip: ${task.robot_server.host_ip}
    robot_server_port: ${task.robot_server.port}
    fps: 120
    vr_server_ip: *vr_server_ip
    vr_server_tcp_port: 10001
    vr_server_force_port: 10005
    bimanual_teleop: *bimanual_teleop

teleop_server:
  # host_ip: "192.168.2.169"
  host_ip: "0.0.0.0"
  port: 8082
  fps: 60
  use_force_control_for_gripper: False
  max_gripper_width: 0.05
  min_gripper_width: 0.0
  grasp_force: 7.0
  gripper_control_time_interval: 30  # 30 for Flexiv Gripper, 20 for PGC-50 gripper
  gripper_control_width_precision: 0.01  # 0.01 for PGC-50, 0.01 for Flexiv Grav Gripper
  bimanual_teleop: *bimanual_teleop

device_mapping_server:
  host_ip: '127.0.0.1'
  port: 8062

transforms:
  calibration_path: 'data/calibration/v6'

logger:
  level: DEBUG
```

### 3.1 Set the VR server IP (`vr_server_ip`)

1. Power on the VR headset and connect it to the same Wi-Fi as the PC.
2. Check the **VR IP address** (e.g., `192.168.43.201`).
3. Edit `real_robot_env.yaml` and set:

   ```yaml
   publisher:
     vr_server_ip: &vr_server_ip 'YOUR_VR_IP_HERE'
   ```

4. Save the file.

---

## 4. TactAR App Setup (VR Side)

On the VR headset, open **TactAR APP**:

1. In the TactAR settings, set the **first IP field** to the **PC's Wi-Fi IP** (not the wired IP).
2. Click **“Refresh IP”**.  
   - After this, the VR should start streaming pose and controller data to the PC at the specified IP and ports.

You can verify connectivity later by checking logs from `teleop.py`.

---

## 5. VR–Robot Coordinate Calibration

You must calibrate the VR coordinate frame with the robot:

1. Follow the instructions from the TactAR repository (text + video):  
   <https://github.com/xiaoxiaoxh/TactAR_APP?tab=readme-ov-file>
2. In brief:
   - Align the large **white marker** in VR with the robot **base frame origin**.
   - Align the **black-and-white marker** with the robot **TCP (end-effector)**.
3. Confirm and save the calibration so that the VR motion correctly maps to robot end-effector motion.

---

## 6. Launching Teleoperation and Data Recording

You need **3 terminals**, all with `rdp_venv` activated and working directory set to the repository.

### 6.1 Terminal 1: Start Teleoperation Server

```bash
cd ~/reactive_diffusion_policy
source rdp_venv/bin/activate
python teleop.py task=wmx_data_collection
```

This:

- Starts the teleoperation server.
- Receives VR tracking and button data.
- Sends TCP and gripper commands to the Flexiv robot via the robot server.

---

### 6.2 Terminal 2: Start Camera Node Launcher

```bash
cd ~/reactive_diffusion_policy
source rdp_venv/bin/activate
python camera_node_launcher.py task=wmx_data_collection
```

This:

- Starts RealSense camera nodes.
- Publishes color images to ROS2 topics:
  - `/external_camera/color/image_raw`
  - `/left_wrist_camera/color/image_raw`
- These topics are later synchronized with robot states for data recording.

---

### 6.3 Terminal 3: Start Data Recorder

```bash
cd ~/reactive_diffusion_policy
source rdp_venv/bin/activate
python record_data.py seq_1.pkl
```

Notes:

- `seq_1.pkl` is the filename for the current recording session.
- While running, the recorder:
  - Subscribes to camera topics and robot state topics.
  - Uses ROS2 message_filters to temporally synchronize:
    - External and wrist images
    - TCP pose / velocity / wrench
    - Gripper width / force
    - Joint positions (q), torques (tau), and external torques (tau_ext)
  - Buffers data and saves them at the end of the session.

---

## 7. Data Collection Procedure

For each demonstration (one complete teleoperated task):

1. **Start the three processes**  
   - `teleop.py task=wmx_data_collection`
   - `camera_node_launcher.py task=wmx_data_collection`
   - `record_data.py seq_1.pkl` (or another filename)

2. **Perform the task using VR teleoperation**  
   - Use the VR controllers (TactAR) to move the robot end-effector and control the gripper.
   - Execute a meaningful task: e.g., reach, grasp, move, place, etc.

3. **Stop recording once the task is finished**
   - Go to **Terminal 3** (Data Recorder).
   - Press `Ctrl+C` to stop `record_data.py`.
   - Wait until the log shows something like:
     ```text
     Saved sensor messages to /path/to/seq_1.pkl
     ```
   - This indicates that the recording has been successfully saved.

4. **(Optional) Stop other processes**
   - You can keep `teleop.py` and `camera_node_launcher.py` running if you plan to quickly record another sequence.
   - Or you can stop them with `Ctrl+C` in their respective terminals.
   - If you keep teleop running, be careful **not to accidentally press the side trigger** or other VR buttons, to avoid unexpected robot motion.

5. **Record a new sequence**
   - For the next demonstration, choose a new file name, e.g.:
     ```bash
     python record_data.py seq_2.pkl
     python record_data.py seq_3.pkl
     ```
   - Repeat steps 2–4.

---

## 8. Network & Latency Tips

VR-based teleoperation is sensitive to network latency and jitter:

- Keep the PC and VR headset **close to the Wi-Fi router** and avoid obstacles.
- If the company Wi-Fi is unstable (ping spikes to hundreds or thousands of ms):
  - Try using your **phone’s hotspot** as Wi-Fi for both PC and VR.
  - Or temporarily disconnect unnecessary network interfaces on the PC to simplify routing.
- You can check the VR link quality with:

  ```bash
  ping <VR_IP>
  ```

  - Most pings should be within ~10 ms.
  - Occasional spikes are acceptable, but frequent large spikes will cause noticeable lag.

---

## 9. Safety & Data Alignment

### 9.1 Safety

- Always ensure a **safe workspace** around the robot:
  - No humans in the immediate workspace when teleoperation is active.
  - Emergency stop should be within easy reach.
- Verify teleoperation behavior with small motions before performing larger movements.

### 9.2 Data Alignment

- The Data Recorder uses ROS2 message_filters (ApproximateTimeSynchronizer) to synchronize topics.
- Each recorded timestep includes:
  - External and wrist camera images
  - TCP pose, velocity, wrench
  - Gripper width and force
  - Joint position (q), torque (tau), and external torque (tau_ext)
- Therefore, **all modalities in a single timestep are time-aligned**; there is no case where only images are recorded without corresponding robot state.

---

## 10. Recommended Shutdown Order

For each recording session:

1. Finish teleoperation for the current demonstration.
2. In the **Data Recorder terminal**:
   - Press `Ctrl+C` to stop `record_data.py`.
   - Wait for the “Saved sensor messages to ...” log.
3. Optional: stop other processes:
   - `teleop.py` (teleoperation server)
   - `camera_node_launcher.py` (RealSense publisher)

This order minimizes the risk of losing data or leaving teleoperation active by accident.

---

If you extend this repository (e.g., new tasks, additional sensors, or different grippers), it is recommended to update this README with any changes in configuration files, network topology, or data formats.
