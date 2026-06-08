# 🚗 Legxy

### ROS 2 Autonomous Mapping & Navigation Robot

A Raspberry Pi 5 powered autonomous mobile robot developed using ROS 2 Jazzy, RPLIDAR C1, and MPU6500 IMU. The platform is capable of real-time teleoperation, LiDAR-based SLAM mapping, laser odometry, sensor integration, and autonomous navigation development.

---

## 📌 Overview

Legxy The Sexyy is a differential-drive robotic platform built for learning and developing modern robotics technologies using ROS 2.

The robot combines:

* Raspberry Pi 5 computing
* LiDAR-based environment perception
* IMU-based orientation sensing
* PID-based heading control
* Real-time mapping using SLAM Toolbox
* Foxglove visualization and monitoring

The project serves as a foundation for future autonomous navigation, path planning, obstacle avoidance, and outdoor robotic applications.

## 🛠 Hardware Components

| Component        | Model                |
| ---------------- | -------------------- |
| Main Controller  | Raspberry Pi 5 (32GB) |
| Operating System | Ubuntu 24.04         |
| ROS Version      | ROS 2 Jazzy          |
| LiDAR            | RPLIDAR C1           |
| IMU              | MPU6500              |
| Motor Driver     | L298N                |
| Motors           | DC Geared Motors     |
| Communication    | Wi-Fi                |
| Visualization    | Foxglove Studio      |

---

## 🧠 Software Stack

* ROS 2 Jazzy
* SLAM Toolbox
* Foxglove Bridge
* RPLIDAR ROS2 Driver
* RF2O Laser Odometry
* TF2
* URDF/Xacro
* Python ROS Nodes
* Teleop Twist Keyboard

---

## ✨ Features

### Completed

* ✅ ROS 2 Jazzy Setup
* ✅ Raspberry Pi 5 Integration
* ✅ RPLIDAR C1 Integration
* ✅ MPU6500 IMU Integration
* ✅ Motor Driver Control
* ✅ Teleoperation
* ✅ PID Heading Correction
* ✅ Laser Odometry
* ✅ SLAM Mapping
* ✅ Occupancy Grid Generation
* ✅ Foxglove Visualization
* ✅ Remote SSH Development
* ✅ Real-Time Sensor Monitoring

---

## 📂 Workspace Structure

```text
rc_car_ws/src/
├── rc_car_bringup/
├── rc_car_driver/
├── rc_car_imu/
├── rc_car_control/
├── rc_car_description/
├── sllidar_ros2/
└── rf2o_laser_odometry/
```

---

## 🚀 Build

```bash
cd ~/rc_car_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## 🚀 Launch Full SLAM System

```bash
ros2 launch rc_car_bringup slam.launch.py
```

Open another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args --remap /cmd_vel:=/cmd_vel_raw
```

Open Foxglove Bridge:

```bash
ros2 run foxglove_bridge foxglove_bridge
```

Connect using:

```text
ws://<RASPBERRY_PI_IP>:8765
```

---

## 🗺 Mapping Results

The robot successfully generates occupancy grid maps using:

* RPLIDAR C1
* RF2O Laser Odometry
* SLAM Toolbox

Generated maps can be saved using:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
```

---

## 🔧 Debugging Tools

Useful ROS commands:

```bash
ros2 node list

ros2 topic list

ros2 topic hz /scan

ros2 topic echo /imu/data_raw
```

Check TF transforms:

```bash
ros2 run tf2_ros tf2_echo map odom
```

---

## 📈 Future Development

* ⬜ Autonomous Navigation (Nav2)
* ⬜ Path Planning
* ⬜ Obstacle Avoidance
* ⬜ GPS Integration
* ⬜ Outdoor Mapping
* ⬜ Autonomous Patrol Missions

---

## 🎯 Learning Objectives

This project was developed to gain hands-on experience in:

* Mobile Robotics
* ROS 2 Development
* Sensor Fusion
* Robot Localization
* Mapping and Navigation
* Autonomous Systems
* Embedded Robotics

---

## 👨‍💻 Author

### Jenil Patel

Robotics & Automation Engineering Student

GitHub: https://github.com/unknownLEGEND27

---

## 📜 License

This project is released under the MIT License.
