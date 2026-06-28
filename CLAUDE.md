# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

MuJoCo simulation of the Yahboom ROSMASTER M3 Pro robot integrated with ROS2 Humble via `ros-controls/mujoco_ros2_control`. The simulation exposes standard ROS topics for a mecanum-drive mobile base with a 6-DOF arm, dual LiDAR, depth camera, and IMU.

## Build and Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Run simulation
ros2 launch m3pro_mujoco_sim sim.launch.py          # headless
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py # with RViz

# Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mecanum_drive_controller/cmd_vel_unstamped

# View URDF only (no simulation)
ros2 launch m3pro_description display.launch.py
```

Standalone MuJoCo model validation (no ROS needed):
```bash
python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('src/m3pro_description/mjcf/m3pro.xml'); print('OK')"
```

## Architecture

Two ROS2 ament_cmake packages, no custom C++ or Python nodes — everything is wired through configuration:

**`m3pro_description`** — Robot model. Contains two parallel representations:
- `mjcf/m3pro.xml` — The MuJoCo MJCF model used by the physics engine. This is the authoritative simulation model. Meshes are referenced relative to `../meshes/` via the `meshdir` compiler directive.
- `urdf/m3pro.urdf` — Plain URDF with `<ros2_control>` tags for `robot_state_publisher` (TF tree) and `MujocoSystemInterface` hardware plugin declaration. Not used for physics. No xacro dependency.

**`m3pro_mujoco_sim`** — Launch files and controller/plugin configs. No code, only YAML and Python launch files.

The data flow is: `m3pro.xml` (MJCF) → MuJoCo engine → `MujocoSystemInterface` → ros2_control controller manager → ROS topics.

## Key Design Decisions

- **Mesh scale**: STL files from Yahboom are in millimeters. All mesh references use `scale="0.001 0.001 0.001"`.
- **Mecanum wheels**: Modeled as cylinder collision geoms with `condim="4"` anisotropic friction under an `elliptic` friction cone. Visual meshes are separate (class="visual", no collision). The actual omnidirectional kinematics are handled by the `mecanum_drive_controller`, not the physics friction — friction values primarily need to provide enough traction for the controller to work.
- **Body hierarchy**: Everything is under a single `base_link` body with a `freejoint`. The prior URDF-to-MJCF conversions in `urdf/` had wheels as siblings of the arm in `worldbody` (broken for mobile simulation) — the `src/` model fixes this.
- **Gripper**: Uses MuJoCo equality constraints to couple all finger joints to `rlink1_joint` (mimic joints from the original URDF).
- **Dual model files**: The MJCF and URDF must stay in sync for joint names. The MJCF is the physics source of truth; the URDF exists only because `robot_state_publisher` requires URDF for TF.

## Joint Name Mapping

Wheel joints: `lwheel1_joint` (FL), `rwheel1_joint` (FR), `lwheel2_joint` (RL), `rwheel2_joint` (RR)
Arm joints: `arm1_joint` through `arm5_joint`
Gripper: `rlink1_joint` (primary), `llink1_joint`, `rlink2_joint`, `llink2_joint`, `rlink3_joint`, `llink3_joint` (coupled via equality constraints)

These names must match across `m3pro.xml`, `m3pro.urdf`, and `controllers.yaml`.

## Files in urdf/ (Root)

The `urdf/` directory at repository root contains the **original** Yahboom URDF and several prior MuJoCo XML conversion attempts (some with Windows absolute paths, some with incorrect body hierarchy). These are kept for reference only. The active simulation model is `src/m3pro_description/mjcf/m3pro.xml`.
