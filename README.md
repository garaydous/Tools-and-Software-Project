# Tools-and-Software-Project

## UR5e ArUco Control System

Vision-based teleoperation system for UR5e robot using ArUco markers for intuitive control.

## Requirements

### Hardware
- USB Webcam (for ArUco marker detection)

### Software
- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.10+
- Required ROS2 packages:
  - `ur_robot_driver`
  - `ur_controllers`
  - `joint_state_publisher`
  - `robot_state_publisher`
  - `rviz2`
- Python packages:
  - `opencv-python`
  - `cv_bridge`
  - `numpy`

## Installation

### 1. Install Docker (for URSim)

```bash
# Install Docker
sudo apt update
sudo apt install -y docker.io docker-compose

# Add your user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### 2. Install ROS2 Jazzy

Follow the official ROS2 installation guide: https://docs.ros.org/en/jazzy/Installation.html

### 3. Install Dependencies

```bash
# Install ROS2 packages
sudo apt update
sudo apt install -y \
  ros-jazzy-ur-robot-driver \
  ros-jazzy-ur-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  python3-opencv \
  v4l-utils

# Source ROS2
source /opt/ros/jazzy/setup.bash
```

### 4. Setup URSim Simulator

```bash
# Create directories for URSim data
mkdir -p ~/.ursim/urcaps
mkdir -p ~/.ursim/programs
cd ~/.ursim/urcaps
wget https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/raw/main/ur_robot_driver/resources/externalcontrol-1.0.5.urcap

```

**URSim Access:**
- Web interface: http://localhost:6080

**URSim Setup Steps:**
1. Access the web interface at http://localhost:6080
2. Power on the robot (press "ON" button)
3. Release brakes (press "Start" button)
4. Go to Installation → URCaps and install the external_control URCap
5. Restart the robot after installing URCap
6. Load a program or use the teach pendant

### 5. Setup Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/garaydous/Tools-and-Software-Project.git
cd ~/ros2_ws

# Build the package
colcon build --packages-select image_click_teleop

# Source the workspace
source install/setup.bash
```

##  Usage

### Prerequisites

```bash
# Start URSim in a separate terminal
docker run --rm -it \
  -p 5900:5900 \
  -p 6080:6080 \
  -p 30001-30004:30001-30004 \
  -v ${HOME}/.ursim/urcaps:/urcaps \
  -v ${HOME}/.ursim/programs:/ursim/programs \
  --name ursim \
  universalrobots/ursim_e-series

# Access URSim web interface: http://localhost:6080
# 1. Power on robot (ON button)
# 2. Go to Installation - URCaps - change parameters if necessary
# 3. Program - create new program- External Control node
# 4. Play

Restart the robot after installing URCap
```


### Quick Start

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch the complete system
ros2 launch image_click_teleop click_teleop.launch.py
```

## How It Works

### ArUco Markers

The system uses ArUco markers (dictionary `DICT_4X4_50`) to control the robot:

- **Marker ID 0**: Primary control marker
- **Green Center Line**: Divides the screen into two control zones


### Robot Control

1. **Upper Zone** (above green line):
   - Marker detected → Base rotates +0.5 rad (counterclockwise)
   
2. **Lower Zone** (below green line):
   - Marker detected → Base rotates -0.5 rad (clockwise)

### Visual Interface

The "ArUco Control UR5e" window displays:
- Green horizontal line (zone divider)
- Robot status (READY / WAITING)
- Current base joint angle
- Detected ArUco markers (highlighted with borders)
- Marker center point (blue circle)

## Authors
Burhan Ali
Gonçalo Paulino
Maria Garrido
