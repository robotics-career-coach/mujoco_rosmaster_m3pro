#!/usr/bin/env python3
"""
Headless MuJoCo verification for the mecanum base variants.

Loads scene_empty_<base>.xml (which <include>s m3pro_robot_<base>.xml -- the
file the real simulation actually loads for that base), settles it under
gravity, then drives it through straight, rotate-in-place, and strafe motion
primitives, reporting displacement, tilt, and ground-contact stability for
each. Strafe is the pass/fail check for correct mecanum kinematics -- if the
robot spins or moves diagonally instead of sideways:
  - mecanum base: flip the handedness signs in generate_mecanum_rollers.py's
    WHEELS table and regenerate.
  - friction base: swap the two tangential friction values (t1/t2) in
    bases/friction/contacts.xml, or flip the capsule euler tilt signs in
    bases/friction/wheels.xml -- see that file's header comment.
  - kinematic base: the FK/IK tendon coefficients in
    bases/kinematic/planar_drive.xml are a transcription of
    mecanum_drive_controller's own source, not an independently derived
    convention, so a strafe/rotate failure here most likely means a
    transcription error worth double-checking against that source, not a
    sign flip to try empirically.

Run: python3 src/m3pro_description/scripts/verify_mecanum_wheels.py [--base mecanum|friction|kinematic|all]
"""

import argparse
import math
from pathlib import Path

import mujoco
import numpy as np

MJCF_DIR = Path(__file__).parent.parent / "mjcf"
BASES = ["mecanum", "friction", "kinematic"]

WHEEL_ACTUATORS = ["fl_wheel_motor", "fr_wheel_motor", "rl_wheel_motor", "rr_wheel_motor"]


def tilt_deg(data, model):
    quat = data.xquat[model.body("base_link").id]
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, quat)
    r = r.reshape(3, 3)
    return math.degrees(math.acos(np.clip(r[2, 2], -1, 1)))


def settle(model, data, steps=500):
    for _ in range(steps):
        mujoco.mj_step(model, data)


def run_scenario(model, name, wheel_speeds, steps=1500):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    settle(model, data)

    start_xy = data.xpos[model.body("base_link").id][:2].copy()
    start_yaw = math.atan2(
        2 * (data.xquat[model.body("base_link").id][0] * data.xquat[model.body("base_link").id][3]),
        1 - 2 * (data.xquat[model.body("base_link").id][3] ** 2),
    )

    for name_, val in zip(WHEEL_ACTUATORS, wheel_speeds):
        data.ctrl[model.actuator(name_).id] = val

    max_tilt = 0.0
    ncons = []
    heights = []
    angvel_mags = []
    gyro_id = model.sensor("imu_gyro").id
    gyro_adr = model.sensor_adr[gyro_id]
    for _ in range(steps):
        mujoco.mj_step(model, data)
        max_tilt = max(max_tilt, tilt_deg(data, model))
        ncons.append(data.ncon)
        heights.append(data.xpos[model.body("base_link").id][2])
        angvel_mags.append(np.linalg.norm(data.sensordata[gyro_adr : gyro_adr + 3]))

    end_xy = data.xpos[model.body("base_link").id][:2].copy()
    end_yaw = math.atan2(
        2 * (data.xquat[model.body("base_link").id][0] * data.xquat[model.body("base_link").id][3]),
        1 - 2 * (data.xquat[model.body("base_link").id][3] ** 2),
    )
    disp = end_xy - start_xy
    yaw_change_deg = math.degrees(end_yaw - start_yaw)
    height_std = float(np.std(heights))
    angvel_std = float(np.std(angvel_mags))
    angvel_max = float(np.max(angvel_mags))

    print(f"\n=== {name} ===")
    print(f"  world XY displacement: dx={disp[0]:+.4f} dy={disp[1]:+.4f} (|d|={np.linalg.norm(disp):.4f} m)")
    print(f"  yaw change: {yaw_change_deg:+.1f} deg")
    print(f"  max tilt from vertical: {max_tilt:.2f} deg")
    print(f"  ncon: min={min(ncons)} max={max(ncons)} mean={np.mean(ncons):.1f}")
    print(f"  jitter: base height std={height_std * 1000:.3f} mm, gyro |w| std={angvel_std:.4f} max={angvel_max:.4f} rad/s")
    return disp, yaw_change_deg, max_tilt, height_std, angvel_std, angvel_max


def verify_base(base):
    mjcf_path = MJCF_DIR / f"scene_empty_{base}.xml"
    print(f"\n{'#' * 60}\n# base variant: {base}  ({mjcf_path.name})\n{'#' * 60}")
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))

    v = 6.0
    # order: fl, fr, rl, rr
    #
    # NOTE on sign convention: lwheel*/rwheel* bodies share one local joint axis
    # ("0 0 -1") but the l/r bodies use mirrored quats, so "same local ctrl sign
    # on all 4 wheels" is actually the ROTATE pattern, and "opposite sign
    # between left/right sides" is the STRAIGHT pattern -- confirmed empirically
    # against this model (this was true even before the roller migration; it's
    # a property of how these wheel joints/quats are defined, not a roller bug).
    # This sign convention is a property of the shared wheel body/joint
    # definitions, so it holds across all base variants, not just mecanum.
    run_scenario(model, "straight (left+, right-)", [v, -v, v, -v])
    run_scenario(model, "rotate in place (all wheels equal)", [v, v, v, v])
    run_scenario(
        model,
        "strafe (front-, rear+)",
        [-v, -v, v, v],
    )
    run_scenario(
        model,
        "strafe, opposite pattern (front+, rear-)",
        [v, v, -v, -v],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base", choices=BASES + ["all"], default="all",
        help="Which base variant's scene_empty_<base>.xml to verify (default: all)",
    )
    args = parser.parse_args()

    for base in BASES if args.base == "all" else [args.base]:
        verify_base(base)


if __name__ == "__main__":
    main()
