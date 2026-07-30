#!/usr/bin/env python3
"""
Regenerates the passive-roller mecanum wheel geometry inside
mjcf/bases/mecanum/wheels.xml -- the fragment the "mecanum" base variant
<include>s for its four wheel bodies (see mjcf/m3pro_robot_mecanum.xml).

Replaces each wheel's hub-collision-geom + roller-bodies block with a freshly
computed one, following the approach demonstrated in
https://github.com/JunHeonYoon/mujoco_mecanum. Use this after changing
NUM_ROLLERS/ROLLER_RADIUS/etc. below, not for a one-off hand edit.

Run manually (dev-time codegen, not built/installed):
    python3 src/m3pro_description/scripts/generate_mecanum_rollers.py [--dry-run]
"""

import argparse
import itertools
import math
import re
import sys
from pathlib import Path

# Wheel body name -> roller handedness (+1/-1). Diagonal pairs (FL/RR, FR/RL)
# share handedness, mirroring how the model already handles FL/FR via body
# quat rather than per-wheel joint-axis changes. This is a best guess verified
# empirically by a pure-strafe test, not a derived fact -- flip signs here and
# regenerate if strafing spins the robot instead of translating it sideways.
WHEELS = [
    ("lwheel1", 1),  # front-left
    ("rwheel1", -1),  # front-right
    ("lwheel2", -1),  # rear-left
    ("rwheel2", 1),  # rear-right
]

# Geometry shared by all 4 wheels (must match the existing hub cylinder in wheels.xml).
WHEEL_RADIUS = 0.04  # matches controllers.yaml's kinematics.wheels_radius (nominal)
HALF_HEIGHT = 0.0275
HUB_Z = 0.0109
NUM_ROLLERS = 12
ROLLER_RADIUS = 0.0045
ROLLER_MASS = 0.005
ROLLER_DAMPING = 0.1
HUB_MARGIN = 0.8  # hub cylinder radius = HUB_MARGIN * roller outer reach
OVERLAP_MARGIN = 1.2  # required ratio of (adjacent roller center distance) / (2*roller_radius)

# Design note: each roller's tilt angle is directly determined by NUM_ROLLERS
# here (chord between adjacent roller placements) -- this is the same coupling
# the reference repo uses. A later attempt to decouple tilt from roller count
# (via elongated capsule rollers with an independently chosen tilt angle) was
# tried and reverted: densely packing wide-chord capsules caused multiple
# rollers per wheel to be simultaneously near-ground at once, producing
# over-constrained/chaotic contact dynamics during strafing (confirmed via an
# empirical per-wheel kinematic Jacobian showing one wheel's contribution
# dominating the others by 6-20x, and non-mirrored responses to command-negated
# inputs -- both signatures of contact chaos, not a geometry/sign bug). Sphere
# rollers with N tied to tilt is less smooth at high N and less grippy at low N,
# but behaves predictably. NUM_ROLLERS=12 here is a deliberate middle ground
# between N=8 (good grip, ~2.85mm bob -- visibly jittery) and N=24 (~0.34mm bob,
# but rotation/strafe authority dropped roughly 10x) -- see verify_mecanum_wheels.py
# results and CLAUDE.md for the measured trade-off at each N.


def compute_geometry():
    step = 2 * math.pi / NUM_ROLLERS
    # The roller *pin circle* radius `a` is not the same as the roller's effective
    # ground-contact radius: each roller body sits at the midpoint of a chord between
    # two pins on that circle, which pulls its center in to a*cos(step/2). Using
    # a = WHEEL_RADIUS - ROLLER_RADIUS directly (as before) therefore made the actual
    # reach fall short of WHEEL_RADIUS by a factor of cos(step/2) -- ~97% of nominal
    # at N=12, a real mismatch against controllers.yaml's kinematics.wheels_radius and
    # the wheel mesh's real 0.04 m radius. Inflating `a` by 1/cos(step/2) cancels that
    # shrinkage so reach lands exactly on WHEEL_RADIUS.
    a = (WHEEL_RADIUS - ROLLER_RADIUS) / math.cos(step / 2)
    reach = a * math.cos(step / 2) + ROLLER_RADIUS
    hub_radius = HUB_MARGIN * reach

    if hub_radius >= reach:
        raise SystemExit(f"hub_radius ({hub_radius:.4f}) must be smaller than roller reach ({reach:.4f})")

    return a, step, reach, hub_radius


def rollers_for_wheel(direction, a, step):
    """Returns list of (joint_pos_xyz, axis_xyz) in the wheel body's local frame.

    The reference repo (JunHeonYoon/mujoco_mecanum) parameterizes its rim circle
    as (X=cos, width=Y, Z=sin) because its wheel joint axis is Y. Our wheel joint
    axis is Z (axis="0 0 -1"), so width must map to Z here instead. A naive
    axis-swap (X=cos, Y=sin, width=Z) turned out to flip chirality (verified
    empirically: it made "straight" spin and "rotate" translate, independent of
    the `direction`/handedness parameter -- see the sign-convention note in
    WHEELS above). Using X=sin, Y=cos here is what was empirically confirmed
    (via src/m3pro_description/scripts/verify_mecanum_wheels.py) to reproduce
    correct mecanum kinematics once combined with the WHEELS handedness table.
    """
    rollers = []
    for i in range(NUM_ROLLERS):
        angle1 = step * i
        angle2 = step * (i + direction)

        pin1 = (a * math.sin(angle1), a * math.cos(angle1), HUB_Z - HALF_HEIGHT)
        pin2 = (a * math.sin(angle2), a * math.cos(angle2), HUB_Z + HALF_HEIGHT)

        joint_pos = tuple((p1 + p2) / 2 for p1, p2 in zip(pin1, pin2))
        axis_raw = tuple(p2 - p1 for p1, p2 in zip(pin1, pin2))
        mag = math.sqrt(sum(c * c for c in axis_raw))
        axis = tuple(c / mag for c in axis_raw)

        # Self-check: joint radial distance from the wheel axis must equal the
        # closed-form expected midpoint radius, or the placement math is wrong.
        expected_radius = a * math.cos(step / 2)
        actual_radius = math.sqrt(joint_pos[0] ** 2 + joint_pos[1] ** 2)
        if abs(actual_radius - expected_radius) > 1e-9:
            raise SystemExit(
                f"roller placement self-check failed: expected radius {expected_radius:.9f}, "
                f"got {actual_radius:.9f}"
            )

        rollers.append((joint_pos, axis))
    return rollers


def check_overlap(rollers):
    pts = [r[0] for r in rollers]
    min_dist = min(math.dist(p1, p2) for p1, p2 in itertools.combinations(pts, 2))
    margin = min_dist / (2 * ROLLER_RADIUS)
    if margin < OVERLAP_MARGIN:
        raise SystemExit(
            f"roller overlap margin too small: {margin:.2f}x (need >= {OVERLAP_MARGIN}x). "
            f"Reduce --num-rollers or --roller-radius."
        )
    return margin


def render_wheel_block(wheel_name, hub_radius, rollers):
    lines = []
    lines.append(
        f'    <geom name="{wheel_name}_collision" type="cylinder" size="{hub_radius:.4f} {HALF_HEIGHT}"'
    )
    lines.append(f'          pos="0 0 {HUB_Z}" class="wheel_contact"/>')
    for i, (pos, axis) in enumerate(rollers):
        lines.append(
            f'    <body name="{wheel_name}_roller{i}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}">'
        )
        lines.append(
            f'      <joint name="{wheel_name}_roller{i}_joint" type="hinge" '
            f'axis="{axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f}" '
            f'damping="{ROLLER_DAMPING}" armature="0" limited="false"/>'
        )
        lines.append(
            f'      <geom name="{wheel_name}_roller{i}_collision" type="sphere" '
            f'size="{ROLLER_RADIUS}" class="wheel_contact" mass="{ROLLER_MASS}"/>'
        )
        lines.append("    </body>")
    return "\n".join(lines)


def splice(xml_text, hub_radius, a, step):
    # Replace each wheel's hub-collision-geom + roller-bodies block. Anchored on
    # the hub geom (fixed name/type/pos, only its size varies) through the last
    # roller body's closing tag; roller bodies are flat (no nesting inside them),
    # so this doesn't need balanced-tag parsing, just a non-greedy repeat of the
    # 4-line roller-body pattern.
    for wheel_name, direction in WHEELS:
        rollers = rollers_for_wheel(direction, a, step)
        check_overlap(rollers)
        replacement = render_wheel_block(wheel_name, hub_radius, rollers)

        pattern = re.compile(
            r'    <geom name="' + re.escape(wheel_name) + r'_collision" type="cylinder" size="[0-9.]+ [0-9.]+"\n'
            r'          pos="0 0 [0-9.]+" class="wheel_contact"/>\n'
            r'(?:    <body name="' + re.escape(wheel_name) + r'_roller\d+" pos="[^"]+">\n'
            r'      <joint name="' + re.escape(wheel_name) + r'_roller\d+_joint"[^\n]*/>\n'
            r'      <geom name="' + re.escape(wheel_name) + r'_roller\d+_collision"[^\n]*/>\n'
            r"    </body>\n)+"
        )
        new_text, n = pattern.subn(replacement + "\n", xml_text, count=1)
        if n != 1:
            raise SystemExit(f"could not find the existing hub+roller block for {wheel_name}")
        xml_text = new_text

    return xml_text


def validate_with_mujoco(wheels_xml_path):
    """Load the real scene through the wheels.xml candidate via MuJoCo's own
    native (file-path-based) include resolution -- the same nested-include
    support the running simulation's mjcf_publisher replicates, but exercised
    directly against a real file so this doesn't need a temp directory shim."""
    import mujoco

    scene_path = wheels_xml_path.parent.parent.parent / "scene_empty_mecanum.xml"
    mujoco.MjModel.from_xml_path(str(scene_path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mjcf-path",
        default=str(Path(__file__).parent.parent / "mjcf" / "bases" / "mecanum" / "wheels.xml"),
        help="Path to bases/mecanum/wheels.xml (default: the package's own copy)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print result to stdout instead of writing the file")
    args = parser.parse_args()

    mjcf_path = Path(args.mjcf_path)
    xml_text = mjcf_path.read_text()

    a, step, reach, hub_radius = compute_geometry()
    print(
        f"wheel_radius={WHEEL_RADIUS} roller_radius={ROLLER_RADIUS} num_rollers={NUM_ROLLERS}\n"
        f"pin_radius(a)={a:.6f} step={math.degrees(step):.1f}deg "
        f"roller_reach={reach:.6f} hub_radius={hub_radius:.6f}\n"
        f"effective rolling radius vs nominal {WHEEL_RADIUS}: "
        f"{100 * reach / WHEEL_RADIUS:.1f}%",
        file=sys.stderr,
    )

    new_text = splice(xml_text, hub_radius, a, step)

    if args.dry_run:
        print(new_text)
        return

    mjcf_path.write_text(new_text)
    print(f"Wrote {mjcf_path}", file=sys.stderr)

    print("Validating via scene_empty_mecanum.xml with mujoco.MjModel.from_xml_path...", file=sys.stderr)
    validate_with_mujoco(mjcf_path)
    print("Validation OK.", file=sys.stderr)


if __name__ == "__main__":
    main()
