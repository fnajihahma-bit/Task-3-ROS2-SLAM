# Task-3-ROS2-SLAM
(Simultaneous Localization and Mapping) with TurtleBot3

## 🗂️ Repo Structure
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

---

## 🧪 Mapping with SLAM in Simulation (Gazebo + Cartographer)

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
