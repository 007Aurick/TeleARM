# 🦾 TeleARM — Autonomous Perception-Guided Mobile Manipulator

> An autonomous mobile robot that detects objects with a camera, navigates to them while avoiding obstacles with a depth camera, and picks them up using a mounted robotic arm — all visualized live in RViz2.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange?logo=gazebo&logoColor=white)
![RViz2](https://img.shields.io/badge/RViz2-Visualization-9cf)
![SLAM](https://img.shields.io/badge/SLAM-slam__toolbox-green)
![Nav2](https://img.shields.io/badge/Nav2-Autonomous%20Navigation-blueviolet)
![Status](https://img.shields.io/badge/status-in%20development-yellow)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

---

## 📖 Overview

**TeleARM** combines perception, autonomous navigation, and manipulation into one working robot:

- 📷 **Camera-based object detection** — spot a target object in the environment
- 🗺️ **Autonomous navigation** using `Nav2` + Depth Camera, driving to the detected object while avoiding obstacles
- 🦾 **Automatic pick-and-place** using a mounted robotic arm, triggered on arrival

No keyboard, no manual driving, no manual goal-setting — the robot decides where to go and what to do based on what it sees.

---

## ✨ Features

- 📷 Real-time object detection from a mounted RGB(-D) camera
- 📍 Detected object position converted into a navigation goal in the map frame
- 🧭 Fully autonomous point-to-point navigation via `Nav2`, using Depth Camera-based obstacle avoidance
- 🌲 Full TF tree: `map → odom → base_link → arm_base_link → ... → gripper`
- 🦾 Fixed-mount robotic arm riding on top of the mobile base
- 🤖 Automatic pick-and-place sequence triggered on goal arrival (no keypress needed)
- 🖥️ Fully visualized in RViz2 — robot model, TF frames, Depth Camera scan, occupancy grid map, camera feed, and detected object markers

---

## 🧠 How It Works
📷 Camera ──► Object Detection ──► Object Position (pixel + depth)
│
▼
Position ──► Transformed into Map Frame
│
▼
Goal Pose ──► Nav2 ──► Path Planning + Obstacle Avoidance
│ ▲
│ │
│ 📡 Depth Camera Scan
▼
Robot Drives Autonomously to Object
│
▼
Goal Reached ──► Arm Node ──► Pick-and-Place Sequence
│
▼
🖥️ RViz2 renders everything together

- **Perception** (camera) decides *where* the robot should go.
- **Navigation** (`Nav2` + Depth Camera) decides *how* to get there safely.
- **Manipulation** (arm) executes *what* happens once the robot arrives.
- Each subsystem is modular — detection, navigation, and arm control communicate only through well-defined topics/goals, not tight coupling.

---

## 🛠️ Tech Stack

| Component            | Tool/Package                        |
|-----------------------|--------------------------------------|
| Middleware            | ROS2 (Humble)                        |
| Simulation             | Gazebo                              |
| Perception              | Camera (RGB or RGB-D) + detection node |
| Navigation              | `Nav2` (planner, controller, costmaps) |
| Localization/Mapping    | `slam_toolbox` (pre-built or live map) |
| Visualization           | RViz2                               |
| Robot Description      | URDF / Xacro                        |
| Arm Control             | Custom joint-state publisher node   |

---

## 📂 Project Structure
telearm_ws/
├── telearm_description/ # URDF/Xacro files for base + arm
│ ├── urdf/
│ └── meshes/
├── telearm_bringup/ # Launch files (sim, SLAM, Nav2, RViz configs)
├── telearm_perception/ # Camera-based object detection node(s)
├── telearm_arm_control/ # Pick-and-place trigger + pose sequence node
└── README.md

---

## 🚀 Getting Started

### Prerequisites
- ROS2 Humble
- Gazebo
- `slam_toolbox`
- `Nav2`
- OpenCV (or equivalent) for object detection

### Build
```bash
cd ~/telearm_ws
colcon build --symlink-install
source install/setup.bash
```

### Run
```bash
# Terminal 1 — launch simulation + robot
ros2 launch telearm_bringup telearm_sim.launch.py

# Terminal 2 — launch Nav2 with a saved map
ros2 launch telearm_bringup navigation.launch.py

# Terminal 3 — start object detection
ros2 run telearm_perception object_detector

# Terminal 4 — arm control node
ros2 run telearm_arm_control pick_place_trigger
```

The robot detects an object with its camera, autonomously navigates to it while avoiding obstacles, and triggers the arm's pick-and-place sequence on arrival. 🦾📦

---

## 🎯 Roadmap

- [x] Robot model + simulation environment
- [x] Depth Camera-based SLAM mapping
- [x] Nav2 autonomous navigation to a manual goal
- [x] 📷 Camera-based object detection
- [ ] 📍 Detection → navigation goal conversion
- [ ] 🤖 Auto-trigger pick-and-place on arrival
- [ ] 🎨 Color/class-based sorting into multiple bins
- [ ] 🧠 MoveIt2 integration for real motion planning of the arm

---

## 🤝 Contributing

Pull requests, issues, and suggestions are welcome! This is a learning project, so feedback that helps simplify or clarify things is especially appreciated. 🙌

---

## 📜 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)
- [ROBOTIS OpenManipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2](https://github.com/ros-navigation/navigation2)

---
