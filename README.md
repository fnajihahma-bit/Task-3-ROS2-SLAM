# Task-3-ROS2-SLAM
(Simultaneous Localization and Mapping) with TurtleBot3

## 🙋 Submitted By

- **Name:** Fatin Najihah Binti Mat Ali  
- **Student ID:** 2024853488

## 🗂️ Repo Structure

```markdown
/week3_task_ros2/
├─ README.md # Steps for SLAM in simulation & real robot
├─ img/
│ ├─ sim_slam.png
│ ├─ sim_map.png
│ ├─ real_slam.png
│ └─ real_map.png
├─ maps/
│ ├─ sim_map.yaml
│ ├─ sim_map.pgm
│ ├─ real_map.yaml
│ └─ real_map.pgm
├─ video/
│ ├─ sim_slam_demo.mp4
│ └─ real_slam_demo.mp4
└─ reflection.pdf # Short reflection (200–300 words)
```
---
## A) Simulation (Gazebo)

### 🧪 Mapping with SLAM in Simulation (Gazebo + Cartographer)

### 🐢 Requirements

Make sure your environment is properly set up:
```bash
export TURTLEBOT3_MODEL=burger
```

📌 Step-by-Step Instructions
1. Launch Gazebo Simulation

Start the TurtleBot3 in a virtual world:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Run SLAM with Cartographer

Launch the SLAM node using Cartographer
(Note: turtlebot3_slam is not available in ROS 2 Humble):

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
use_sim_time:=True ensures SLAM uses Gazebo's simulation clock.

3. Teleoperate the Robot

Use keyboard control to move the robot and build the map:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
⌨️ Drive the robot around the environment slowly to allow Cartographer to generate an accurate map.

4. Save the Map

Once the environment is fully mapped, save the map to a file:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/sim_map
```
This will generate:

~/sim_map.pgm — the occupancy grid map image

~/sim_map.yaml — the metadata file used for localization

---

### B) Real Robot (Lab Environment)

### 🐢 TurtleBot3 Bringup with ROS 2 Humble

This repository documents the step-by-step setup and bringup process for using a TurtleBot3 robot with **ROS 2 Humble**, across a **Remote PC** and a **TurtleBot3 SBC** (e.g., Raspberry Pi).

---
### 📋 Table of Contents

```markdown

1. [Requirements](#requirements)
2. [Network Configuration](#network-configuration)
3. [ROS 2 Environment Setup (`.bashrc`)](#ros-2-environment-setup-bashrc)
4. [Real Robot Bringup (Lab Environment)](#real-robot-bringup-lab-environment)
5. [SLAM with Cartographer](#slam-with-cartographer)
6. [Teleoperation](#teleoperation)
7. [Saving the Map](#saving-the-map)

```
---

## ✅ Requirements

- ROS 2 **Humble** installed on:
  - 🖥️ Remote PC (Ubuntu 22.04)
  - 🐢 TurtleBot3 SBC (Raspberry Pi 4)
- `turtlebot3` ROS 2 packages installed:
  - `turtlebot3_bringup`
  - `turtlebot3_cartographer`
  - `turtlebot3_teleop`
- Devices connected to the **same Wi-Fi**
- SSH access from Remote PC to robot

---

## 🌐 Network Configuration

1. Ensure both SBC and Remote PC are on the same Wi-Fi.
2. Check IP of SBC (run on SBC):

```bash
hostname -I
```
Check IP, this IP is needed to SSH from Remote PC.

### ⚙️ ROS 2 Environment Setup (.bashrc)

### 🐢 TurtleBot3 SBC

On the SBC (e.g., Raspberry Pi):

1. Open .bashrc:

```bash
nano ~/.bashrc
```

2. Add the following at the bottom:

# ==== TurtleBot3 SBC ROS 2 Setup ====

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LDS_MODEL=LDS-02            # or LDS-01
export TURTLEBOT3_MODEL=waffle     # or burger
```

3. Save and apply:

```bash
source ~/.bashrc
```

### 💻 Remote PC

On your ROS 2 workstation (Remote PC):

1. Open .bashrc:

```bash
nano ~/.bashrc
```

2. Add the following at the bottom:

# ==== Remote PC ROS 2 Setup ====

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TURTLEBOT3_MODEL=waffle     # Must match SBC
```
⚠️ Do not set LDS_MODEL on the Remote PC.

3. Save and apply:

```bash
source ~/.bashrc
```

### 🚀 Launching TurtleBot3 (Bringup)

### 🔧 These steps must be done on the TurtleBot3 SBC.

1. SSH into the SBC from your Remote PC:

```bash
ssh uitm@10.9.10.153
```

2. Run the bringup:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
If .bashrc is correctly set, this will launch all essential TurtleBot3 nodes (sensors, motors, publishers, etc.).

### 🧭 SLAM with Cartographer

✅ Run this on your Remote PC after bringup is running on SBC.

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

This will:

- Start SLAM using LiDAR and odometry
- Open RViz2 with live map building



### 📡 Testing Communication (Remote PC)

After bringup is running on the SBC:

1. On the Remote PC, list active topics:

```bash
ros2 topic list
```
Expected output:

```bash
/battery_state
/cmd_vel
/imu
/odom
/scan
/joint_states
/tf
/tf_static
...
```

2. Echo a topic (e.g., LiDAR):

```bash
ros2 topic echo /scan
```

### 🎮 Teleoperation (Optional)

To control the robot from the Remote PC using your keyboard:

1. Install the teleop package (if not already):

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

2. Launch teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
You can now control your robot using W/A/S/D keys.


### 💾 Saving the Map

After you’ve finished exploring:

Run this on the Remote PC in a new terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/real_map
```

This will save two files in your home directory:
- real_map.yaml
- real_map.pgm

You can use these later for navigation.
