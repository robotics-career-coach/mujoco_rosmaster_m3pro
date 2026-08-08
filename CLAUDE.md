# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

MuJoCo simulation of the Yahboom ROSMASTER M3 Pro robot integrated with ROS2 Humble via `ros-controls/mujoco_ros2_control`. The simulation exposes standard ROS topics for a mecanum-drive mobile base with a 6-DOF arm, dual LiDAR, depth camera, and IMU.

## Build and Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Run simulation (default: empty scene)
ros2 launch m3pro_mujoco_sim sim.launch.py                        # headless, empty scene
ros2 launch m3pro_mujoco_sim sim.launch.py scene:=basic           # with objects
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py              # with RViz
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py scene:=basic # RViz + objects

# Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mecanum_drive_controller/reference_unstamped

# View URDF only (no simulation)
ros2 launch m3pro_description display.launch.py
```

Standalone MuJoCo model validation (no ROS needed):
```bash
python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('src/m3pro_description/mjcf/scene_empty.xml'); print('OK')"
```

## Architecture

Two ROS2 ament_cmake packages, no custom C++ or Python nodes — everything is wired through configuration:

**`m3pro_description`** — Robot model. Contains two parallel representations:
- `mjcf/m3pro_robot.xml` — The robot definition (body tree, actuators, sensors, constraints). Included by scene files via `<include>`.
- `mjcf/scene_*.xml` — Scene files that `<include>` the robot and add environment (ground, lights, objects). The `scene` launch argument selects which one to load (e.g., `scene:=basic` loads `scene_basic.xml`).
- `urdf/m3pro.urdf` — Plain URDF with `<ros2_control>` tags for `robot_state_publisher` (TF tree) and `MujocoSystemInterface` hardware plugin declaration. Not used for physics. No xacro dependency.

**`m3pro_mujoco_sim`** — Launch files and controller/plugin configs. No code, only YAML and Python launch files.

The data flow is: `scene_*.xml` (MJCF + includes) → `mjcf_publisher` (resolves includes) → MuJoCo engine → `MujocoSystemInterface` → ros2_control controller manager → ROS topics.

## Key Design Decisions

- **Mesh scale**: STL files from Yahboom are in meters (SI units). No scale factor is needed on mesh references.
- **Mecanum wheels**: Each wheel is a hub cylinder plus 12 free-spinning sphere "rollers" on tilted hinge joints (real roller geometry, not a friction hack — see `MECANUM_MODELING_NOTES.md` and `src/m3pro_description/mjcf/generate_mecanum_wheel.py`, the generator used to derive the roller bodies spliced into `m3pro_robot.xml`). Hub and roller geoms use `contype="1" conaffinity="0"` so they collide with the ground but not each other or the chassis (`base_collision` uses the same filtering so wheel rollers don't wedge against it). Wheel joints all use `<motor>` (effort) actuators, not `<velocity>` — a `<velocity>` actuator fights the roller contact physics. `mecanum_drive_controller` still commands wheel joints through the `velocity` command_interface; `mujoco_ros2_control`'s hardware interface converts that into effort via a per-joint velocity PID configured through `pids_config_file` (see `m3pro_mujoco_sim/config/wheel_pids.yaml`, referenced from `m3pro_ros2_control.urdf.xacro`/`m3pro.urdf`). That config must define a `pid_gains.position.<joint>` block too (with finite but otherwise unused values) — the installed `ros-humble-mujoco-ros2-control` build has a readiness-check bug that reads the position PID's gains instead of the velocity PID's when deciding whether the velocity PID is configured. Visual meshes remain separate (class="visual", no collision). All four wheel joints share the same local hinge axis; the two right-side wheel bodies use a mirrored `quat` relative to the left side, so their axis must be sign-flipped (`axis="0 0 1"` vs `axis="0 0 -1"` on the left) for a positive `qvel` to push the chassis the same way on both sides — verified via isolated single-wheel drive tests in standalone MuJoCo before wiring up ROS.
- **Body hierarchy**: Everything is under a single `base_link` body with a `freejoint`. The prior URDF-to-MJCF conversions in `urdf/` had wheels as siblings of the arm in `worldbody` (broken for mobile simulation) — the `src/` model fixes this.
- **Gripper**: Uses MuJoCo equality constraints to couple all finger joints to `rlink1_joint` (mimic joints from the original URDF).
- **Dual model files**: The MJCF and URDF must stay in sync for joint names. The MJCF is the physics source of truth; the URDF exists only because `robot_state_publisher` requires URDF for TF.

## Joint Name Mapping

Wheel joints: `lwheel1_joint` (FL), `rwheel1_joint` (FR), `lwheel2_joint` (RL), `rwheel2_joint` (RR)
Arm joints: `arm1_joint` through `arm5_joint`
Gripper: `rlink1_joint` (primary), `llink1_joint`, `rlink2_joint`, `llink2_joint`, `rlink3_joint`, `llink3_joint` (coupled via equality constraints)

These names must match across `m3pro_robot.xml`, `m3pro.urdf`, and `controllers.yaml`.
