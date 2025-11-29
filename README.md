# ROS 2 Warehouse Simulation & Consensus-Based task distributer

A robust **ROS 2 Humble** and **Gazebo Fortress** simulation for multi-robot warehouse automation. This project focuses on decentralized path planning (Nav2) combined with a centralized consensus-based task allocation system.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)
![License](https://img.shields.io/badge/License-Apache_2.0-green)

---

## Workspace Architecture

This project follows a professional "Separation of Concerns" architecture:

```text
src/
├── bringup_sim/
│   ├── launch/                  # Master launch files (Sim + RViz + Bridge)
│   ├── config/                  # RViz, Bridge YAML, and Nav2 params
│   └── maps/                    # Generated SLAM maps
│
├── fleet_manager/
│   ├── fleet_manager/           # Python nodes for Task Allocation
│   └── ...                      # Consensus algorithm logic
│
├── gazebo_sim/                  # GZ world
│   ├── worlds/
│   └── models/
│
└── warehouse_robot_description/
    ├── urdf/
    └── meshes/
```

---

## Quick Start
1. Prerequisites
Ensure you have Ubuntu 22.04 and ROS 2 Humble installed. You will also need Gazebo Fortress and the ROS-GZ bridge.
```text
# Install Gazebo Fortress and ROS 2 Bindings
sudo apt-get install ros-humble-ros-gz \
                     ros-humble-slam-toolbox \
                     ros-humble-navigation2 \
                     ros-humble-nav2-bringup \
                     ros-humble-xacro \
                     ros-humble-robot-state-publisher
```

2. Build the Workspace
```text
# Create a workspace directory (if you haven't already)
mkdir -p ~/warehouse_ws/src
cd ~/warehouse_ws/src

# Clone this repository
git clone https://github.com/andytsai104/ros2_warehouse_simulation.git

# Build packages
cd ~/warehouse_ws
colcon build --symlink-install
source install/setup.bash
```

3. Launch Simulation
We use a modular launch file that handles the Robot, Gazebo, RViz, and the ROS-GZ Bridge automatically.
Basic Simulation (Drive Around):
```text
ros2 launch bringup_sim gazebo_control.launch.py
```

Simulation with Mapping (SLAM):
```text
ros2 launch bringup_sim gazebo_control.launch.py slam:=True
```

4. Control the Robot
The robot uses a differential drive controller. Open a new terminal to drive it manually:
```text
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```



