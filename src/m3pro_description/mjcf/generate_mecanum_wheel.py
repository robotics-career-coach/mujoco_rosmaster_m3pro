#!/usr/bin/env python3
"""Generate mecanum wheel roller bodies for m3pro_robot.xml.

Adapted from the roller-geometry construction in JunHeonYoon/mujoco_mecanum's
wheel_code_gen.py (see MECANUM_MODELING_NOTES.md), fixed to size roller
spheres by roller_r (not the full wheel radius r), and re-derived for this
model's wheel convention: the hub cylinder is oriented along local Z (not Y),
offset from the wheel body's origin by hub_center_z, with size="r h" meaning
radius r and half-length h.

Output is a list of <body> fragments for the rollers only -- splice them
inside the existing wheel <body>...</body> alongside the pre-existing hub
joint/geoms.
"""
from math import pi, sin, cos
import argparse

ROLLER_RATIO = 0.08 / 0.127  # confirmed ratio from the reference repo


def roller_bodies(link_name, r, h, hub_center_z, n_roller, wheel_type, roller_ratio=ROLLER_RATIO):
    roller_r = r * roller_ratio
    step = (2 * pi) / n_roller
    out = []

    def pin(i, z):
        return (
            (r - roller_r) * cos(step * i),
            (r - roller_r) * sin(step * i),
            z,
        )

    for i in range(n_roller):
        pin_1 = pin(i, hub_center_z - h / 2)

        if wheel_type == 0:
            j = 0 if i == n_roller - 1 else i + 1
        else:
            j = n_roller - 1 if i == 0 else i - 1
        pin_2 = pin(j, hub_center_z + h / 2)

        axis = tuple(b - a for a, b in zip(pin_1, pin_2))
        pos = tuple(a + d / 2 for a, d in zip(pin_1, axis))

        body_name = f"{link_name}_roller_{i}"
        joint_name = f"{link_name}_slipping_{i}_joint"

        out.append(f"""      <body name="{body_name}_link" pos="{pos[0]:.6g} {pos[1]:.6g} {pos[2]:.6g}">
        <joint name="{joint_name}" type="hinge" pos="0 0 0" axis="{axis[0]:.6g} {axis[1]:.6g} {axis[2]:.6g}"
               damping="0.1" limited="false" actuatorfrclimited="false"/>
        <inertial pos="0 0 0" mass="0.001" diaginertia="1e-5 1e-5 1e-5"/>
        <geom name="{body_name}_geom" type="sphere" size="{roller_r:.6g}"
              contype="1" conaffinity="0" group="3" rgba="0.15 0.15 0.15 1"/>
      </body>""")

    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--link_name", required=True, help="wheel body name, e.g. lwheel1")
    parser.add_argument("--n_roller", type=int, default=12)
    parser.add_argument("--r", type=float, default=0.04, help="wheel radius")
    parser.add_argument("--h", type=float, default=0.0275, help="hub cylinder half-length")
    parser.add_argument("--hub_center_z", type=float, default=0.0109,
                         help="z-offset of hub cylinder geom within the wheel body")
    parser.add_argument("--type", type=int, choices=[0, 1], required=True,
                         help="roller handedness: 0 or 1 (must alternate diagonally: FL+RR one type, FR+RL the other)")
    args = parser.parse_args()

    print(roller_bodies(args.link_name, args.r, args.h, args.hub_center_z, args.n_roller, args.type))


if __name__ == "__main__":
    main()
