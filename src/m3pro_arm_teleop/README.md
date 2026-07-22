# m3pro_arm_teleop

Task-space (Cartesian) keyboard jog teleoperation for a 5-DOF arm + 1-DOF
gripper controlled through standard `ros2_control` topics. Originally built
for the Yahboom ROSMASTER M3 Pro, but the node is deliberately decoupled from
any simulation- or robot-specific package: it gets the robot's kinematic
chain from the `/robot_description` topic (published by `robot_state_publisher`
in both simulation and on real hardware) and streams position targets to a
`JointTrajectoryController` and a `JointGroupPositionController`. This
directory has no dependency on any other package in its parent workspace, so
it can be copied into its own repository and reused as-is against any arm
that exposes the same kind of `ros2_control` interface.

## Why this exists

There is no off-the-shelf ROS package that does keyboard-jog, task-space
teleop for an arbitrary `ros2_control` arm without a full MoveIt
(`moveit_config`/SRDF) setup. This package fills that gap with a single small
node built on **PyKDL** (`orocos_kdl_vendor`), an official, apt-installable
ROS2 kinematics library — no SRDF or MoveIt required. `urdfdom_py` is used to
parse joint types/axes/limits straight out of the plain URDF.

## How it works

1. On startup, the node waits for `/robot_description` and builds a
   `PyKDL.Chain` from a configurable `root_link` to `tip_link` (default
   `base_link` → `gripping_housing`), plus a table of each joint's URDF
   position limits.
2. It waits for `/joint_states` and seeds its internal joint/pose state from
   the arm's actual current position (never assumes a zero pose).
3. Each keypress nudges an internal target Cartesian pose (translation in the
   `root_link` frame, or pitch/yaw rotation) and runs `PyKDL.ChainIkSolverPos_LMA`,
   warm-started from the previous solution, to solve for joint angles.
4. The IK solution is accepted only if every joint stays within its URDF
   limit **and** the achieved pose (via forward kinematics on the solution)
   is within `ik_position_tolerance`/`ik_orientation_tolerance` of what was
   requested. Otherwise the move is rejected and the arm holds its last good
   pose. (KDL's own internal convergence flag is intentionally not used as
   the accept/reject signal — see the comment in `build_kinematics()` in the
   script for why: a 5-joint arm cannot exactly satisfy an arbitrary 6-DOF
   pose, so that flag is essentially always "not converged" even when the
   achieved pose is a fraction of a millimeter off, which is why an explicit
   tolerance check on the *achieved* pose is used instead.)
5. Accepted solutions are published as a single-point `JointTrajectory` to
   the arm controller. The gripper is handled separately and purely in joint
   space (it's a single independent joint; no IK involved).

There is deliberately no independent "roll" control: a 5-joint arm cannot
track an arbitrary full 6-DOF pose, so the target orientation's roll
component is always re-synced from the actual achieved pose after each
solve rather than driven to an arbitrary keyboard-commanded value. This
keeps the 6-vector pose target always consistent with something the arm can
actually reach.

## Running

In a separate terminal, with the simulation (or real robot's `ros2_control`
stack) already running:

```bash
ros2 run m3pro_arm_teleop arm_teleop_keyboard
```

## Keymap

| Axis | + | - |
|---|---|---|
| x (forward/back, base frame) | `w` | `s` |
| y (left/right, base frame) | `a` | `d` |
| z (up/down, base frame) | `r` | `f` |
| pitch | `t` | `g` |
| yaw | `y` | `h` |
| gripper (open/close) | `u` | `j` |

`0` = return to home pose (all arm joints to 0). `Ctrl-C` = quit; the arm and
gripper hold their last commanded position.

Holding a key down relies on the terminal's own key-repeat to give a
continuous jogging feel — the node does not run a background republish
timer, since both controllers hold their last commanded target indefinitely.

## Parameters

All overridable via `--ros-args -p name:=value`; defaults work out of the box
against this repository's simulation.

| Parameter | Default | Meaning |
|---|---|---|
| `root_link` | `base_link` | KDL chain root |
| `tip_link` | `gripping_housing` | KDL chain tip (tool frame) |
| `gripper_joint_name` | `rlink1_joint` | Independently-actuated gripper joint |
| `translation_step` | `0.01` (m) | Per-keypress translation increment |
| `rotation_step` | `0.05` (rad) | Per-keypress pitch/yaw increment |
| `gripper_step` | `0.05` (rad) | Per-keypress gripper increment |
| `time_from_start` | `0.2` (s) | Trajectory point duration |
| `ik_position_tolerance` | `0.005` (m) | Max accepted position error |
| `ik_orientation_tolerance` | `0.1` (rad) | Max accepted orientation error |
| `arm_trajectory_topic` | `/arm_controller/joint_trajectory` | |
| `gripper_command_topic` | `/gripper_controller/commands` | |
| `joint_states_topic` | `/joint_states` | |
| `robot_description_topic` | `/robot_description` | |
| `home_gripper_position` | `0.0` (rad) | Gripper position on home |
