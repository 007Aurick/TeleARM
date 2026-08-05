# 🦾 TeleARM — Autonomous Warehouse Forklift

> A ROS 2 Humble + Gazebo Classic mobile forklift that wanders a warehouse, detects colored boxes from simulation state, picks them with a claw, and delivers each one to the matching color drop pad — no keyboard teleop required.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange?logo=gazebo&logoColor=white)
![RViz2](https://img.shields.io/badge/RViz2-Visualization-9cf)
![SLAM](https://img.shields.io/badge/SLAM-slam__toolbox-green)
![Nav2](https://img.shields.io/badge/Nav2-Optional-blueviolet)
![ros2_control](https://img.shields.io/badge/ros2__control-diff__drive%20%2B%20gripper-critical)
![Status](https://img.shields.io/badge/status-working%20sim%20mission-brightgreen)
![License](https://img.shields.io/badge/license-MIT-lightgrey)
![Platform](https://img.shields.io/badge/platform-WSL2%20%2F%20Linux-informational)

**Tags:** `#ROS2` `#Humble` `#Gazebo` `#ros2_control` `#DiffDrive` `#Forklift` `#Warehouse` `#PickAndPlace` `#Lidar` `#Nav2` `#SLAM` `#RViz2` `#MobileManipulator` `#Simulation`

---

## 📖 Overview

**TeleARM** is a simulated warehouse forklift that:

- 🚗 **Wanders** open space with lidar-based obstacle avoidance
- 📦 **Detects** colored boxes (`orange` / `blue` / `green` / `red`) from `/gazebo/model_states`
- 🤏 **Picks** boxes by driving into the fork pocket and closing the claw
- 🎯 **Delivers** each box to the matching corner drop pad
- 🔁 **Repeats** until the warehouse is cleared

Primary mission path uses a custom state machine (`mission.py`). Optional `slam_toolbox` + `Nav2` launches are still included for mapping / navigation experiments.

---

## ✨ Features

- 🏭 Large storage warehouse world (walls, racks, colored boxes, corner pads)
- 🚚 Scaled differential-drive forklift with a front claw (open / close is visually obvious)
- 📡 360° lidar (`/scan`) for wander avoidance and approach safety
- 🧠 Full mission FSM: `searching → approaching → gripping → delivering → dropping`
- 🎨 Color-matched delivery pads (boxes land on-pad; later drops use an on-pad grid)
- ⬛ Cubes keep full face color with **black edge frames** so they don’t look like pad markers
- 🎮 `ros2_control` diff-drive + gripper position controller
- 🗺️ Optional SLAM (`slam_toolbox`) and Nav2 bringup launch files
- 🖥️ Optional RViz / display launch for model inspection

---

## 🧠 How It Works

```text
Lidar (/scan) ──► Wander (obstacle avoidance) while searching
                         │
                         ▼
/gazebo/model_states ──► Best visible colored box (FOV + range)
                         │
                         ▼
              Approach box → align → drive into forks
                         │
                         ▼
              Close claw → seat box → carry to color pad
                         │
                         ▼
              Plant box on pad → open claw → reverse → turn
                         │
                         ▼
                    Resume wander (repeat)
```

| Subsystem | Role |
|-----------|------|
| **Wander** (`diff_drive_publisher.py`) | Drives while searching; silenced when mission takes over |
| **Mission** (`mission.py`) | Owns approach, grip, delivery, drop sequencing |
| **Gripper** (`gripper_publisher.py`) | Publishes open/close joint commands |
| **Gazebo state** | Ground-truth box / robot poses (reliable on WSL; RGB camera is optional/unreliable) |
| **Lidar** | Avoid walls/racks; don’t treat the carried box as an obstacle |

### Drop pads

| Color | Pad world pose `(x, y)` |
|-------|-------------------------|
| Orange | `(-22, 15)` |
| Blue | `(22, 15)` |
| Green | `(-22, -15)` |
| Red | `(22, -15)` |

---

## 🛠️ Tech Stack

| Component | Tool / Package |
|-----------|----------------|
| Middleware | ROS 2 Humble |
| Simulation | Gazebo Classic (`gazebo_ros`) |
| Robot description | URDF / Xacro + STL meshes |
| Controllers | `ros2_control` — `diff_drive_controller` + gripper position controller |
| Perception (mission) | `/gazebo/model_states` box lookup |
| Perception (optional) | RGB camera + `box_detector` / `sim_box_detector` |
| Avoidance | Lidar `LaserScan` (`/scan`) |
| Mapping (optional) | `slam_toolbox` |
| Navigation (optional) | `Nav2` + `cmd_vel_relay` |
| Visualization | Gazebo GUI / RViz2 |

---

## 📂 Project Structure

```text
TeleARM/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── diff_drive_controller.yaml   # diff-drive + gripper controllers
│   ├── nav2_params.yaml             # optional Nav2
│   └── *.rviz
├── launch/
│   ├── gazebo.launch.py             # main sim bringup
│   ├── display.launch.py            # model / RViz
│   ├── online_async_launch.py       # slam_toolbox
│   └── navigation.launch.py         # Nav2
├── urdf/
│   └── telearm.urdf.xacro           # forklift + claw + sensors
├── meshes/
│   ├── body.STL
│   ├── wheel.STL
│   └── window.STL
├── worlds/
│   └── storage_warehouse.world      # warehouse, boxes, pads
├── scripts/
│   ├── mission.py                   # main autonomous FSM
│   ├── diff_drive_publisher.py      # wander / avoidance
│   ├── gripper_publisher.py         # claw open/close
│   ├── sim_box_detector.py          # optional sim detector
│   └── box_detector.py              # optional RGB detector
├── TeleARM_control/
│   ├── cmd_vel_relay.py             # Nav2 Twist → TwistStamped
│   └── cleanup_controller.py
└── rviz/
```

---

## 🚀 Getting Started

### Prerequisites

- ROS 2 Humble
- Gazebo Classic + `gazebo_ros` packages
- `ros2_control` / `gazebo_ros2_control`
- `xacro`, `robot_state_publisher`
- (Optional) `slam_toolbox`, `nav2_bringup`, `rviz2`, `foxglove_bridge`

### Build

From the workspace that contains this package (example: `~/ros2_ws`):

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select TeleARM --symlink-install
source install/setup.bash
```

If this repo is its own colcon workspace under `~/ros2_ws/TeleARM`:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/TeleARM
colcon build --symlink-install
source install/setup.bash
```

### Run — autonomous warehouse mission (recommended)

**Terminal 1 — Gazebo + robot + controllers + wander/gripper pubs**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash   # or ~/ros2_ws/TeleARM/install/setup.bash
ros2 launch TeleARM gazebo.launch.py gui:=true
```

For better FPS / physics on WSL:

```bash
ros2 launch TeleARM gazebo.launch.py gui:=false
```

**Terminal 2 — mission**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run TeleARM mission.py
```

That’s it for the main demo. Do **not** also run `sim_box_detector` for the default mission path — `mission.py` already finds boxes from model states.

### Run — optional SLAM / Nav2

```bash
# SLAM
ros2 launch TeleARM online_async_launch.py

# Nav2 (uses cmd_vel_relay when wired in)
ros2 launch TeleARM navigation.launch.py
```

### Run — model display

```bash
ros2 launch TeleARM display.launch.py
```

---

## 📡 Key Topics / Services

| Name | Type | Notes |
|------|------|--------|
| `/diff_drive_base_controller/cmd_vel` | `geometry_msgs/TwistStamped` | Mission + wander command interface |
| `/enable_wander` | `std_msgs/Bool` | Mission enables wander only while searching |
| `/gripper_command` | `std_msgs/String` | `"open"` / `"close"` |
| `/gripper_controller/commands` | `std_msgs/Float64MultiArray` | Finger joint positions |
| `/scan` | `sensor_msgs/LaserScan` | Lidar avoidance |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Robot + box poses |
| `/gazebo/set_entity_state` | service | Seat / plant boxes during carry & drop |
| `/camera/image_raw` | `sensor_msgs/Image` | Optional RGB (can be flaky on WSL) |

---

## 🤖 Mission States

| State | Behavior |
|-------|----------|
| `searching` | Wander enabled, claw open, look for nearest in-FOV box |
| `approaching` | Align and drive until box is in the fork pocket |
| `gripping` | Close claw, seat box, verify pocket |
| `delivering` | Carry to matching color pad slot |
| `dropping` | Plant box on pad, open claw, reverse, turn, resume search |

---

## 🎯 Roadmap

- [x] Forklift URDF + Gazebo warehouse world
- [x] `ros2_control` diff-drive + gripper
- [x] Lidar wander / obstacle avoidance
- [x] Autonomous pick → color-pad delivery loop
- [x] On-pad drop slots (no pad-marker confusion; black cube edges)
- [x] Optional `slam_toolbox` + Nav2 launch files
- [x] More robust physics-only grasp (less `set_entity_state` assist)
- [x] Reliable RGB detection path on WSL / GPU setups
- [x] Full Nav2 handoff for long-range delivery
- [x] MoveIt2 / articulated arm variant

---

## ⚠️ Notes (WSL / performance)

- Gazebo GUI can drop to very low FPS on WSL; use `gui:=false` when testing the mission.
- Low real-time factor makes physics “spazzy”; prefer headless for stability.
- RGB camera plugins have historically been unreliable on some WSL setups — the mission uses model states on purpose.
- After URDF / world changes, fully restart Gazebo (don’t just respawn the mission node).

---

## 🤝 Contributing

Pull requests, issues, and suggestions are welcome. This is a learning / demo project — feedback that simplifies the stack or hardens the pick/drop loop is especially appreciated.

---

## 📜 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details (add one if it isn’t in the repo yet).

---

## 🙏 Acknowledgments

- [ros2_control](https://github.com/ros-controls/ros2_control) / [gazebo_ros2_control](https://github.com/ros-controls/gazebo_ros2_control)
- [gazebo_ros](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2](https://github.com/ros-navigation/navigation2)
- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)

---

<p align="center">
  <b>TeleARM</b> — wander · detect · pick · sort · repeat 🚚📦
</p>
