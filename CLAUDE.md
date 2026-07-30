# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

MuJoCo simulation of the Yahboom ROSMASTER M3 Pro robot integrated with ROS2 Humble via `ros-controls/mujoco_ros2_control`. The simulation exposes standard ROS topics for a mecanum-drive mobile base with a 6-DOF arm, dual LiDAR, depth camera, and IMU.

## Build and Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Run simulation (default: empty scene, mecanum base)
ros2 launch m3pro_mujoco_sim sim.launch.py                        # headless, empty scene
ros2 launch m3pro_mujoco_sim sim.launch.py scene:=basic           # with objects
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py              # with RViz
ros2 launch m3pro_mujoco_sim sim_with_rviz.launch.py scene:=basic # RViz + objects

# Switch the base's wheel/ground-contact model (see "Base variants" below)
ros2 launch m3pro_mujoco_sim sim.launch.py base:=friction         # anisotropic-friction capsule wheels
ros2 launch m3pro_mujoco_sim sim.launch.py base:=kinematic        # no wheel-ground contact (holonomic ghost base)

# Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mecanum_drive_controller/reference_unstamped

# Jog the arm/gripper in task space with keyboard
ros2 run m3pro_arm_teleop arm_teleop_keyboard

# View URDF only (no simulation)
ros2 launch m3pro_description display.launch.py
```

Standalone MuJoCo model validation (no ROS needed):
```bash
python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('src/m3pro_description/mjcf/scene_empty_mecanum.xml'); print('OK')"
```

## Architecture

Three ROS2 ament_cmake packages. The first two are wired entirely through configuration (no custom nodes beyond `mjcf_publisher`); the third adds one standalone Python node:

**`m3pro_description`** — Robot model. Contains two parallel representations:
- `mjcf/scene_<scene>_<base>.xml` — The top-level file the running simulation actually loads (e.g. `scene_empty_mecanum.xml`). One per `scene` (`empty`/`basic`) × `base` (`mecanum`/`friction`/`kinematic`) combination — see "Base variants" below. Each `<include>`s an `m3pro_robot_<base>.xml` assembler plus shared `environments/*.xml` fragments for lights/ground/objects.
- `mjcf/m3pro_robot_<base>.xml` — Per-base assembler: compiler/default blocks, then `<include>`s of the fragments in `mjcf/common/` (assets, chassis core, arm/camera/gripper, gripper equality, wheel actuators/sensors — identical across all three bases) and `mjcf/bases/<base>/` (the base-specific wheel bodies, and for `kinematic`, the FK tendon/equality constraints too).
- `mjcf/common/` and `mjcf/bases/<base>/` — `<mujocoinclude>` fragments, not standalone valid MJCF; only meaningful spliced into an assembler file.
- `urdf/m3pro.urdf` — Plain URDF with `<ros2_control>` tags for `robot_state_publisher` (TF tree) and `MujocoSystemInterface` hardware plugin declaration. Not used for physics. No xacro dependency.

**`m3pro_mujoco_sim`** — Launch files and controller/plugin configs. No code, only YAML and Python launch files.

**`m3pro_arm_teleop`** — Task-space keyboard jog teleop for the arm + gripper (see its own `src/m3pro_arm_teleop/README.md`). A single rclpy node (`scripts/arm_teleop_keyboard`) that builds a PyKDL kinematic chain from `/robot_description` (no SRDF/MoveIt), solves IK per keypress, and streams position targets to `/arm_controller/joint_trajectory` and `/gripper_controller/commands`. No launch file — run directly via `ros2 run` in its own terminal, same as the mecanum base's `teleop_twist_keyboard`. Has no dependency on the other two packages (only standard `ros2_control`/`robot_state_publisher` topics), so it's self-contained enough to be extracted into its own repo and reused against real hardware.

The data flow is: `scene_<scene>_<base>.xml` (MJCF + includes) → `mjcf_publisher` (resolves includes, including nested ones inside `<body>`/`<actuator>`/etc. — see `_resolve_element` and its `_MERGEABLE_SECTIONS` whitelist) → MuJoCo engine → `MujocoSystemInterface` → ros2_control controller manager → ROS topics.

## Base variants

The `base` launch argument (default `mecanum`) picks how the wheel/ground interaction is simulated, independent of the `scene` argument, by selecting `mjcf/scene_<scene>_<base>.xml`. All three variants expose the exact same ROS-facing joints/actuators/sensors (`lwheel1_joint`/`rwheel1_joint`/`lwheel2_joint`/`rwheel2_joint`, same `mecanum_drive_controller` wiring in `controllers.yaml`) — only what's simulating the wheel-ground physics underneath changes. See `docs/mecanum_drive_modeling_in_mujoco.md` for the general background on each approach.

- **`mecanum`** (default) — The passive-roller model described below. Highest fidelity, highest cost, the one the real physical rollers are meant to approximate.
- **`friction`** — Single anisotropic-friction capsule per wheel (`mjcf/bases/friction/`), approach 2 from the modeling doc. Cheaper and lower-`ncon` than the roller model, and straight-line driving is clean, but **strafe and rotate-in-place currently have real cross-coupling** (strafe induces ~10-30° of unwanted yaw; rotate-in-place has weak yaw authority and some lateral drift) even after re-deriving the capsule tilt from the already-validated roller axis direction instead of a naive 45° guess — see the "KNOWN LIMITATION" comment in `mjcf/bases/friction/contacts.xml` before relying on this for anything strafe-accuracy-sensitive. This is the same "easy to get subtly wrong" gotcha the modeling doc warns about for this approach.
- **`kinematic`** — No wheel-ground contact at all (`mjcf/bases/kinematic/`). `base_link` carries three explicit planar joints (`base_x_joint`/`base_y_joint`/`base_yaw_joint`: slide-x, slide-y, hinge-yaw, in that order) instead of a `freejoint`, held by fixed-tendon `<equality>` constraints to the exact forward-kinematics combination of the four wheel joints that `mecanum_drive_controller`'s own `odometry.cpp` computes (transcribed from that source, not re-derived). Wheel geoms are non-colliding (visual only). Validated via `scripts/verify_mecanum_wheels.py`: straight/strafe/rotate are all perfectly clean (zero cross-coupling) and jitter is exactly zero, since there's no contact solver in the propulsion path at all. The one wrinkle: raw MuJoCo qvel sign is physically inverted between left/right wheels (mirrored body quats — the same quirk documented for the roller model below), so the tendon coefficients for `rwheel1_joint`/`rwheel2_joint` carry a compensating sign flip relative to a literal transcription of the controller source; see the comment in `mjcf/bases/kinematic/planar_drive.xml`. Useful when base contact dynamics aren't what's being tested (e.g. arm work, nav stack logic) — no traction/slip modeling at all.

Run `python3 src/m3pro_description/scripts/verify_mecanum_wheels.py [--base mecanum|friction|kinematic|all]` after changing any base variant's geometry.

## Key Design Decisions

- **Mesh scale**: STL files from Yahboom are in meters (SI units). No scale factor is needed on mesh references.
- **Mecanum wheels (`base:=mecanum`, the default)**: Each wheel is a small hub cylinder plus 12 passive, free-spinning roller bodies (sphere geom + undriven hinge joint) around its rim, generated by `m3pro_description/scripts/generate_mecanum_rollers.py` into `mjcf/bases/mecanum/wheels.xml` — not the anisotropic-friction-cylinder approximation used previously (that approximation is now its own separate, explicitly-selected `base:=friction` variant — see "Base variants" above — not the default). This mirrors the approach in [JunHeonYoon/mujoco_mecanum](https://github.com/JunHeonYoon/mujoco_mecanum): physically modeled rollers, not friction tricks, are what make mecanum motion smooth in MuJoCo. Contact/friction on the wheel geoms is plain MuJoCo defaults; no `<contact><pair>` overrides or `cone="elliptic"` for this variant. Wheel actuators are driven by the ROS2 `mecanum_drive_controller`, same as before, but are written as `<general>` (not the `<velocity>` shortcut) so they can carry ramp dynamics — see below. These actuators/sensors live in `mjcf/common/wheel_actuators.xml` and `wheel_sensors.xml`, shared by all three base variants since the ROS-facing joint interface doesn't change.
  - **Roller count matters more than roller mass for smoothness.** An `N=8` attempt was still visibly jittery — not from mass/inertia (verified: sweeping roller-joint armature over 6 orders of magnitude had zero effect on jitter), but from a "faceted wheel" geometric effect: with few rollers, ground clearance oscillates measurably between roller hand-offs (computed ~2.85mm bob at N=8, vs ~0.34mm at N=24). Jitter metrics (base height std / angular velocity std/max during driving) tracked this almost exactly.
  - **Roller count vs. rotation/strafe authority is a real trade-off, not just a tunable gain.** Higher `N` (with this sphere-chord design) shrinks each roller's tilt angle (~25° at N=8 → ~10° at N=24), which measurably weakens in-place-rotation and strafe grip (rotation yaw for a fixed test wheel speed dropped ~10x from N=8 to N=24; strafe displacement dropped similarly). `N=12` (current) is a deliberate middle ground: ~1.3mm bob (vs 2.85mm at N=8), ~18.5° tilt, and validated straight/rotate/strafe all working cleanly (see `scripts/verify_mecanum_wheels.py`).
  - **A decoupled-tilt redesign was tried and reverted.** To get both low bob *and* good tilt simultaneously, an attempt was made to decouple roller tilt (fixed via capsule geoms at a chosen angle) from roller count (placement spacing) — mirroring how real mecanum rollers are angled barrels, not friction-derived chords. This measurably fixed rotation authority but broke strafing: densely-packed wide-chord capsules put multiple rollers per wheel simultaneously near-ground, producing over-constrained/chaotic contact dynamics (confirmed via an empirical per-wheel kinematic Jacobian — one wheel's contribution dominated the others by 6-20x, and negated commands produced non-mirrored responses, both signatures of contact chaos rather than a sign/geometry bug). Reverted in favor of the simpler, validated sphere-chord `N=12` design. Worth retrying with a lower roller count and/or narrower capsule footprint if future jitter/authority trade-offs need more headroom than `N` alone can give.
  - Known approximation: roller count/radius/tilt (`N=12`, radius 4.5mm, ~18.5° tilt) are a simulation convenience, not a reverse-engineered match to the real ROSMASTER M3 Pro's physical roller wheels. The pin-placement radius used for roller positions is solved (inflated by `1/cos(step/2)`, see `compute_geometry()`) so the effective rolling radius lands exactly on the nominal 0.04 m used in `controllers.yaml`'s kinematics, rather than falling ~3% short as an earlier version did. Roller handedness (which diagonal wheel pair shares which rim-tilt direction) was determined empirically via `scripts/verify_mecanum_wheels.py`, not derived analytically — see that script's sign-convention note if changing wheel geometry.
  - **The roller redesign was, for a time, wired to the wrong file.** All of the roller work above was originally applied only to `mjcf/m3pro.xml`, a file orphaned when the model was split into `m3pro_robot.xml` + `scene_*.xml` (`m3pro_robot.xml` is what scenes actually `<include>`). The running simulation kept using the old anisotropic-friction-cylinder wheels the whole time, which was the real cause of the reported jitter/fall-over during teleop — not roller count or contact tuning. `m3pro.xml` has since been deleted; `generate_mecanum_rollers.py`/`verify_mecanum_wheels.py` now target `mjcf/bases/mecanum/wheels.xml`/`mjcf/scene_empty_mecanum.xml` directly (the model was further split into per-base variants after this; see "Base variants" above) so this can't silently diverge again.
  - **Wheel velocity actuators ramp instead of stepping instantaneously to a new setpoint**, modeling the real hardware's motor/ESC acceleration limit. Implemented via `dyntype="filterexact" dynprm="0.15 0 0"` on each wheel's `<general>` actuator (`gaintype`/`biastype`/`gainprm`/`biasprm` reproduce the old `<velocity kv="5">` exactly; MuJoCo's `<velocity>` shortcut doesn't expose `dyntype`, hence the switch to `<general>`). The activation state ramps toward `ctrl` with time constant `dynprm[0]`, and that filtered value — not raw `ctrl` — drives the kv=5 velocity servo. No real M3 Pro acceleration spec is available yet; 0.15s is a placeholder pending hardware calibration. `filterexact` (vs `filter`) integrates the ramp analytically so it stays accurate regardless of `dynprm[0]` vs. the 0.002s timestep.
- **Body hierarchy**: Everything is under a single `base_link` body with a `freejoint`. The prior URDF-to-MJCF conversions in `urdf/` had wheels as siblings of the arm in `worldbody` (broken for mobile simulation) — the `src/` model fixes this.
- **Gripper**: Uses MuJoCo equality constraints to couple all finger joints to `rlink1_joint` (mimic joints from the original URDF).
- **Dual model files**: The MJCF and URDF must stay in sync for joint names. The MJCF is the physics source of truth; the URDF exists only because `robot_state_publisher` requires URDF for TF.

## Joint Name Mapping

Wheel joints: `lwheel1_joint` (FL), `rwheel1_joint` (FR), `lwheel2_joint` (RL), `rwheel2_joint` (RR)
Arm joints: `arm1_joint` through `arm5_joint`
Gripper: `rlink1_joint` (primary), `llink1_joint`, `rlink2_joint`, `llink2_joint`, `rlink3_joint`, `llink3_joint` (coupled via equality constraints)

These names must match across every `m3pro_robot_<base>.xml`, `m3pro.urdf`, and `controllers.yaml`.

`m3pro_arm_teleop`'s PyKDL chain runs `base_link` → `gripping_housing` (the fixed tool frame just past `arm5_joint`) — these are link names, not joint names, but are load-bearing for that package's IK.
