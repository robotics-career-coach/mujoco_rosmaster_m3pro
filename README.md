# Yahboom ROSMASTER M3 Pro MuJoCo Simulation 

MuJoCo physics simulation of the [Yahboom ROSMASTER M3 Pro](https://category.yahboom.net/products/rosmaster-m3-pro) robot with ROS2 Jazzy integration via `ros2_control`. The simulation publishes sensor data and accepts velocity commands on standard ROS topics, enabling development of navigation, perception, and control algorithms without the physical robot.

## Robot Features Simulated

- **Mecanum drive base** — 4 mecanum wheels with anisotropic friction, omnidirectional motion
- **6-DOF robotic arm** — Position-controlled with gripper (mimic-joint coupled fingers)
- **Depth camera** — RGB + depth image streams
- **Dual LiDAR** — Front-left and rear-right TOF rangefinder arrays (T-mini Plus, 0.05–12m range)
- **9-axis IMU** — Orientation, angular velocity, linear acceleration
- **Wheel encoders** — Joint position and velocity feedback

## Repository Structure

```
src/
├── m3pro_description/       # Robot model package
│   ├── mjcf/m3pro.xml       # MuJoCo model (primary simulation model)
│   ├── urdf/                # URDF xacro files for robot_state_publisher & ros2_control
│   ├── meshes/              # STL mesh files from Yahboom CAD
│   ├── rviz/                # RViz display config
│   └── launch/              # display.launch.py (RViz-only visualization)
│
└── m3pro_mujoco_sim/        # Simulation launch & config package
    ├── config/
    │   ├── controllers.yaml       # ros2_control controller definitions
    │   ├── mujoco_plugins.yaml    # Camera & LiDAR sensor plugin config
    │   └── mujoco_sim.yaml        # MuJoCo simulation parameters
    ├── launch/
    │   ├── sim.launch.py          # Headless simulation
    │   └── sim_with_rviz.launch.py
    └── worlds/                    # Scene files with obstacles

meshes/    # Original mesh files from the robot
urdf/      # Original Yahboom URDF and prior MuJoCo XML conversion attempts
```

## ROS Topics

### Control Inputs

| Topic | Type | Description |
|-------|------|-------------|
| `/mecanum_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | Base velocity (vx, vy, omega) |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Arm trajectory commands |
| `/gripper_controller/commands` | `std_msgs/Float64MultiArray` | Gripper position |

### Sensor Outputs

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | All joint positions and velocities |
| `/mecanum_drive_controller/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | IMU data |
| `/depth_camera/color/image_raw` | `sensor_msgs/Image` | RGB camera image |
| `/depth_camera/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/lidar_front/scan` | `sensor_msgs/LaserScan` | Front LiDAR scan |
| `/lidar_rear/scan` | `sensor_msgs/LaserScan` | Rear LiDAR scan |

## Prerequisites

- **ROS2 Jazzy** — [Installation guide](https://docs.ros.org/en/jazzy/Installation.html)
- **MuJoCo** — Installed automatically as a dependency of `mujoco_ros2_control`

## Setup

### 1. Install ROS2 dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard
```

### 2. Install mujoco_ros2_control

Check if a binary package is available:

```bash
sudo apt install ros-jazzy-mujoco-ros2-control
```

If not available via apt, build from source:

```bash
cd src/
git clone -b main https://github.com/ros-controls/mujoco_ros2_control.git
cd ..
```

### 3. Install remaining dependencies via rosdep

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

### Launch simulation with MuJoCo viewer

```bash
ros2 launch m3pro_mujoco_sim sim.launch.py
```

### Launch simulation with RViz

```bash
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py
```

### Drive the robot with keyboard teleop

In a separate terminal:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mecanum_drive_controller/cmd_vel_unstamped
```

### View URDF in RViz (no simulation)

```bash
ros2 launch m3pro_description display.launch.py
```

## Architecture

The simulation uses `ros-controls/mujoco_ros2_control` as the bridge between MuJoCo and ROS2:

```
┌─────────────────────────────────────────────┐
│  MuJoCo Physics Engine                      │
│  (m3pro.xml — MJCF model)                   │
└──────────────┬──────────────────────────────┘
               │ MujocoSystemInterface
┌──────────────┴──────────────────────────────┐
│  ros2_control Controller Manager            │
│  ├── mecanum_drive_controller               │
│  ├── joint_state_broadcaster                │
│  ├── imu_sensor_broadcaster                 │
│  ├── arm_controller (JointTrajectory)       │
│  └── gripper_controller                     │
└──────────────┬──────────────────────────────┘
               │ ROS2 Topics
┌──────────────┴──────────────────────────────┐
│  Your Application                           │
│  (nav2, MoveIt2, custom nodes, etc.)        │
└─────────────────────────────────────────────┘
```
