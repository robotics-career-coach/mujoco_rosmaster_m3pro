# Yahboom ROSMASTER M3 Pro MuJoCo Simulation 

MuJoCo physics simulation of the [Yahboom ROSMASTER M3 Pro](https://category.yahboom.net/products/rosmaster-m3-pro) robot with ROS2 Humble integration via `ros2_control`. The simulation publishes sensor data and accepts velocity commands on standard ROS topics, enabling development of navigation, perception, and control algorithms without the physical robot.

![MuJoCo simulation of the ROSMASTER M3 Pro](resources/mujoco_rosmaster_m3pro.png)

## Robot Features Simulated

- **Mecanum drive base** — 4 mecanum wheels with physically modeled passive rollers, omnidirectional motion
- **6-DOF robotic arm** — Position-controlled with gripper (mimic-joint coupled fingers)
- **Depth camera** — RGB + depth image streams
- **Dual LiDAR** — Front-left and rear-right TOF rangefinder arrays (T-mini Plus, 0.05–12m range)
- **9-axis IMU** — Orientation, angular velocity, linear acceleration
- **Wheel encoders** — Joint position and velocity feedback

## Repository Structure

```
src/
├── m3pro_description/       # Robot model package
│   ├── mjcf/
│   │   ├── scene_<scene>_<base>.xml # Top-level file the sim loads, e.g. scene_empty_mecanum.xml
│   │   ├── m3pro_robot_<base>.xml   # Per-base assembler (mecanum/friction/kinematic)
│   │   ├── common/                  # Shared fragments (arm, gripper, camera, wheel actuators...)
│   │   ├── bases/<base>/            # Base-specific wheel bodies/contacts/tendons
│   │   └── environments/            # Shared lights/ground/object fragments (empty, basic)
│   ├── urdf/                # Plain URDF for robot_state_publisher & ros2_control
│   ├── meshes/              # STL mesh files from Yahboom CAD
│   ├── rviz/                # RViz display config
│   └── launch/              # display.launch.py (RViz-only visualization)
│
├── m3pro_mujoco_sim/        # Simulation launch & config package
│   ├── config/
│   │   ├── controllers.yaml       # ros2_control controller definitions
│   │   └── mujoco_plugins.yaml    # Camera & LiDAR sensor plugin config
│   ├── launch/
│   │   ├── sim.launch.py          # Headless simulation
│   │   └── sim_with_rviz.launch.py
│   └── worlds/                    # Scene files with obstacles
│
└── m3pro_arm_teleop/        # Task-space keyboard teleop for the arm + gripper
    └── scripts/
        └── arm_teleop_keyboard    # Standalone rclpy node (PyKDL-based IK, no MoveIt)
```

## ROS Topics

### Control Inputs

| Topic | Type | Description |
|-------|------|-------------|
| `/mecanum_drive_controller/reference_unstamped` | `geometry_msgs/Twist` | Base velocity (vx, vy, omega) |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Arm trajectory commands |
| `/gripper_controller/commands` | `std_msgs/Float64MultiArray` | Gripper position |

### Sensor Outputs

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | All joint positions and velocities |
| `/mecanum_drive_controller/odometry` | `nav_msgs/Odometry` | Wheel odometry |
| `/imu/data` | `sensor_msgs/Imu` | IMU data (via MuJoCo sensor plugin) |
| `/depth_camera/color/image_raw` | `sensor_msgs/Image` | RGB camera image |
| `/depth_camera/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/lidar_front/scan` | `sensor_msgs/LaserScan` | Front LiDAR scan |
| `/lidar_rear/scan` | `sensor_msgs/LaserScan` | Rear LiDAR scan |

## Quick Start (Dev Container)

The easiest way to get started on any OS — all dependencies are pre-installed:

1. Install [Docker](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Clone this repo and open it in VS Code
3. When prompted, click **"Reopen in Container"** (or run `Dev Containers: Reopen in Container` from the command palette)
4. The container builds and installs all dependencies automatically. Once ready:

```bash
source install/setup.bash
ros2 launch m3pro_mujoco_sim sim.launch.py                # empty scene (default)
ros2 launch m3pro_mujoco_sim sim.launch.py scene:=basic   # scene with objects
ros2 launch m3pro_mujoco_sim sim.launch.py base:=friction # cheaper anisotropic-friction wheels
```

> **GUI note:** For MuJoCo viewer and RViz, you need X11 forwarding. On Linux/WSL2 this works out of the box. On macOS, install [XQuartz](https://www.xquartz.org/) and run `xhost +local:docker` first.

> **GPU note:** `.devcontainer/devcontainer.json` requests `--gpus=all`, which requires an NVIDIA GPU with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed on the host (Docker Desktop on Windows/WSL2 sets this up for you if you have an NVIDIA GPU). If your machine has no NVIDIA GPU, container startup will fail with a "could not select device driver" error — remove the `--gpus=all` entry (and the `NVIDIA_VISIBLE_DEVICES`/`NVIDIA_DRIVER_CAPABILITIES` container env vars) from `devcontainer.json` to run on CPU-only rendering instead.

### Git credentials in the dev container

VS Code Dev Containers automatically forwards your host's git credentials and `.gitconfig` (name/email) into the container, so `git push`/`pull`/etc. work without leaving the container — no repo configuration needed. It just requires a one-time setup on your host machine first:

**Option A: HTTPS with a credential helper (recommended — same steps work on all three OSes)**

| Platform | Setup |
|---|---|
| Windows | Already set up — Git for Windows ships with Git Credential Manager (GCM) by default |
| macOS | `brew install git-credential-manager` then `git-credential-manager configure` |
| Ubuntu | `sudo apt install git-credential-manager` then `git-credential-manager configure` |

After configuring, sign in once (`git pull` from any repo will prompt a browser login). VS Code transparently proxies credential requests from inside the container to this host helper.

**Option B: SSH keys**

Make sure an `ssh-agent` is running on your host with your key loaded:

- **macOS**: `ssh-agent` runs by default — just run `ssh-add ~/.ssh/id_ed25519` (or your key path)
- **Ubuntu**: `eval "$(ssh-agent -s)" && ssh-add`, and add that to `~/.bash_profile` so it persists across shells
- **Windows**: in an admin PowerShell, `Set-Service ssh-agent -StartupType Automatic; Start-Service ssh-agent`, then `ssh-add` your key

VS Code detects the running agent and forwards it into the container automatically, so `git@github.com:...` SSH remotes work as-is.

> Don't try to hand-roll this with a `mounts` entry in `devcontainer.json` (like the X11 socket mount above) — an SSH agent is a Unix socket on Linux/macOS but a named pipe on Windows, so a hardcoded mount path won't work across all three platforms. Let VS Code's built-in forwarding handle it.

## Manual Setup (Ubuntu 22.04)

### Prerequisites

- **Ubuntu 22.04** (native or WSL2)
- **ROS2 Humble** — [Installation guide](https://docs.ros.org/en/humble/Installation.html)

### 1. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-mujoco-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-python-orocos-kdl-vendor \
  ros-humble-urdfdom-py
pip install mujoco
```

### 2. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

### Launch simulation with MuJoCo viewer

```bash
ros2 launch m3pro_mujoco_sim sim.launch.py                # empty scene (default)
ros2 launch m3pro_mujoco_sim sim.launch.py scene:=basic   # scene with objects
```

### Launch simulation with RViz

```bash
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py                # empty scene (default)
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py scene:=basic   # scene with objects
```

The `scene` and `base` arguments together map to `mjcf/scene_<scene>_<base>.xml` in `m3pro_description` — see [Repository Structure](#repository-structure) above. `scene` picks the environment (`empty` or `basic`); `base` picks how the wheel/ground physics is modeled:

| `base` value | Description |
|---|---|
| `mecanum` (default) | Passive-roller mecanum wheels — highest fidelity, matches the real robot's roller geometry most closely |
| `friction` | Single anisotropic-friction capsule per wheel — cheaper, smoother straight-line driving; strafe/rotate have known cross-coupling issues, see [CLAUDE.md](CLAUDE.md#base-variants) |
| `kinematic` | No wheel-ground contact at all — a holonomic "ghost" base driven by an exact kinematic constraint; zero jitter, no traction/slip modeling |

```bash
ros2 launch m3pro_mujoco_sim sim.launch.py base:=kinematic
```

### Drive the robot with keyboard teleop

In a separate terminal:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mecanum_drive_controller/reference_unstamped
```

To verify commands are actually reaching the controller, watch the topic in a third terminal while pressing keys:

```bash
ros2 topic echo /mecanum_drive_controller/reference_unstamped
```

#### Keyboard bindings

The mecanum base is holonomic, so it can strafe sideways as well as drive and turn. Hold **Shift** on the letter keys below to strafe instead of turning.

| Key | Motion | Shift+Key | Motion |
|-----|--------|-----------|--------|
| `i` | Forward | `I` | Forward |
| `,` | Backward | `<` | Backward |
| `j` | Turn left (CCW) | `J` | Strafe left |
| `l` | Turn right (CW) | `L` | Strafe right |
| `u` | Forward + turn left | `U` | Forward + strafe left |
| `o` | Forward + turn right | `O` | Forward + strafe right |
| `m` | Backward + turn right | `M` | Backward + strafe left |
| `.` | Backward + turn left | `>` | Backward + strafe right |
| `t` | Up (+z, unused on ground base) | | |
| `b` | Down (-z, unused on ground base) | | |
| any other key | Stop | | |

Speed adjustment:

| Key | Effect |
|-----|--------|
| `q` / `z` | Increase / decrease both linear and angular speed by 10% |
| `w` / `x` | Increase / decrease linear speed only by 10% |
| `e` / `c` | Increase / decrease angular speed only by 10% |

Default speed is 0.5 m/s linear, 1.0 rad/s angular. `Ctrl-C` quits and publishes a zero-velocity command on exit.

### Jog the arm and gripper with task-space keyboard teleop

In a separate terminal:

```bash
source install/setup.bash
ros2 run m3pro_arm_teleop arm_teleop_keyboard
```

This node (`m3pro_arm_teleop`, see its own [README](src/m3pro_arm_teleop/README.md) for full details) solves inverse kinematics with PyKDL to let you jog the gripper's Cartesian pose directly, rather than moving one arm joint at a time — no MoveIt/SRDF setup required.

| Axis | + | - |
|---|---|---|
| x (forward/back, base frame) | `w` | `s` |
| y (left/right, base frame) | `a` | `d` |
| z (up/down, base frame) | `r` | `f` |
| pitch | `t` | `g` |
| yaw | `y` | `h` |
| gripper (open/close) | `u` | `j` |

`0` returns the arm to its home pose. `Ctrl-C` quits and holds the last commanded pose.

> **Zero-effort alternative:** `ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller` gives GUI sliders for direct per-joint positioning. It ships transitively with `ros2_controllers` (already a dependency of this repo), so no new code or install is needed — handy for precise joint-level testing, though it doesn't do task-space jogging.

### View URDF in RViz (no simulation)

```bash
ros2 launch m3pro_description display.launch.py
```

## Architecture

The simulation uses `ros-controls/mujoco_ros2_control` as the bridge between MuJoCo and ROS2:

```
┌─────────────────────────────────────────────┐
│  MuJoCo Physics Engine                      │
│  (scene_<scene>_<base>.xml — MJCF)          │
└──────────────┬──────────────────────────────┘
               │ MujocoSystemInterface
┌──────────────┴──────────────────────────────┐
│  ros2_control Controller Manager            │
│  ├── mecanum_drive_controller               │
│  ├── joint_state_broadcaster                │
│  ├── arm_controller (JointTrajectory)       │
│  └── gripper_controller                     │
└──────────────┬──────────────────────────────┘
               │ ROS2 Topics
┌──────────────┴──────────────────────────────┐
│  Your Application                           │
│  (nav2, MoveIt2, custom nodes, etc.)        │
└─────────────────────────────────────────────┘
```

### Mecanum wheel modeling

The wheel/ground physics is configurable via the `base` launch argument (see
above) independent of everything else in the simulation — same ROS topics,
joints, and controllers regardless of which one is loaded.

By default (`base:=mecanum`), each wheel is a hub cylinder surrounded by 12
passive, free-spinning roller bodies (sphere geoms on undriven hinge joints)
arranged around its rim, rather than a single geom with hand-tuned directional
friction — this is what produces smooth holonomic motion in MuJoCo (generated
by `m3pro_description/scripts/generate_mecanum_rollers.py`). See
[docs/mecanum_drive_modeling_in_mujoco.md](docs/mecanum_drive_modeling_in_mujoco.md)
for a broader survey of MuJoCo mecanum-modeling approaches and their tradeoffs,
and [CLAUDE.md](CLAUDE.md#base-variants) for the other two `base` options
(`friction`, `kinematic`) and their known limitations.
