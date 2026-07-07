#!/usr/bin/env python3
"""
Headless MuJoCo verification for the passive-roller mecanum wheels.

Loads m3pro.xml, settles it under gravity, then drives it through straight,
rotate-in-place, and strafe motion primitives, reporting displacement, tilt,
and ground-contact stability for each. Strafe is the pass/fail check for
roller handedness (see generate_mecanum_rollers.py's WHEELS table) -- if the
robot spins or moves diagonally instead of sideways, flip the handedness
signs there and regenerate.

Run: python3 src/m3pro_description/scripts/verify_mecanum_wheels.py
"""

import math
from pathlib import Path

import mujoco
import numpy as np

MJCF_PATH = Path(__file__).parent.parent / "mjcf" / "m3pro.xml"

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
    for _ in range(steps):
        mujoco.mj_step(model, data)
        max_tilt = max(max_tilt, tilt_deg(data, model))
        ncons.append(data.ncon)

    end_xy = data.xpos[model.body("base_link").id][:2].copy()
    end_yaw = math.atan2(
        2 * (data.xquat[model.body("base_link").id][0] * data.xquat[model.body("base_link").id][3]),
        1 - 2 * (data.xquat[model.body("base_link").id][3] ** 2),
    )
    disp = end_xy - start_xy
    yaw_change_deg = math.degrees(end_yaw - start_yaw)

    print(f"\n=== {name} ===")
    print(f"  world XY displacement: dx={disp[0]:+.4f} dy={disp[1]:+.4f} (|d|={np.linalg.norm(disp):.4f} m)")
    print(f"  yaw change: {yaw_change_deg:+.1f} deg")
    print(f"  max tilt from vertical: {max_tilt:.2f} deg")
    print(f"  ncon: min={min(ncons)} max={max(ncons)} mean={np.mean(ncons):.1f}")
    return disp, yaw_change_deg, max_tilt


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

    v = 6.0
    # order: fl, fr, rl, rr
    #
    # NOTE on sign convention: lwheel*/rwheel* bodies share one local joint axis
    # ("0 0 -1") but the l/r bodies use mirrored quats, so "same local ctrl sign
    # on all 4 wheels" is actually the ROTATE pattern, and "opposite sign
    # between left/right sides" is the STRAIGHT pattern -- confirmed empirically
    # against this model (this was true even before the roller migration; it's
    # a property of how these wheel joints/quats are defined, not a roller bug).
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


if __name__ == "__main__":
    main()
