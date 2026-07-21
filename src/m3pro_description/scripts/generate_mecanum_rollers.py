#!/usr/bin/env python3
"""
Generates passive-roller mecanum wheel geometry and splices it into
m3pro_robot.xml -- the robot definition actually <include>d by scene_*.xml
and loaded by the running simulation.

Replaces the single-cylinder + anisotropic-friction-cone wheel approximation with
a wheel hub cylinder plus N free-spinning passive roller bodies around its rim,
following the approach demonstrated in https://github.com/JunHeonYoon/mujoco_mecanum.

Run manually (dev-time codegen, not built/installed):
    python3 src/m3pro_description/scripts/generate_mecanum_rollers.py [--dry-run]

Re-running against an already-migrated file: `git checkout -- <mjcf-path>` first
(check out a pre-migration commit of m3pro_robot.xml, i.e. one predating the
roller splice, not just the working-tree copy).
"""

import argparse
import itertools
import math
import re
import sys
import tempfile
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

# Geometry shared by all 4 wheels (must match the existing cylinder in m3pro_robot.xml).
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
        f'        <geom name="{wheel_name}_collision" type="cylinder" size="{hub_radius:.4f} {HALF_HEIGHT}"'
    )
    lines.append(f'              pos="0 0 {HUB_Z}" class="wheel_contact"/>')
    for i, (pos, axis) in enumerate(rollers):
        lines.append(
            f'        <body name="{wheel_name}_roller{i}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}">'
        )
        lines.append(
            f'          <joint name="{wheel_name}_roller{i}_joint" type="hinge" '
            f'axis="{axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f}" '
            f'damping="{ROLLER_DAMPING}" armature="0" limited="false"/>'
        )
        lines.append(
            f'          <geom name="{wheel_name}_roller{i}_collision" type="sphere" '
            f'size="{ROLLER_RADIUS}" class="wheel_contact" mass="{ROLLER_MASS}"/>'
        )
        lines.append("        </body>")
    return "\n".join(lines)


def splice(xml_text, hub_radius, a, step):
    # 1. Drop cone="elliptic" if present in this file -- it only existed to support
    #    the anisotropic <pair> friction overrides being removed below. m3pro_robot.xml
    #    itself has no <option> (that lives in the including scene_*.xml files), so this
    #    is a no-op there; it's cone="elliptic" in the scene files that should be dropped
    #    separately once the roller migration lands.
    xml_text = re.sub(r'\s*cone="elliptic"', "", xml_text, count=1)

    # 2. Add the wheel_contact default class after the existing collision class.
    collision_default = '    <default class="collision">\n      <geom group="3" rgba="0.5 0.5 0.5 0.3"/>\n    </default>'
    if collision_default not in xml_text:
        raise SystemExit("could not find the collision default block to anchor the new wheel_contact class")
    wheel_contact_default = (
        collision_default
        + "\n\n"
        + '    <default class="wheel_contact">\n'
        + '      <geom contype="1" conaffinity="0" group="3" rgba="0.5 0.5 0.5 0.3"/>\n'
        + "    </default>"
    )
    xml_text = xml_text.replace(collision_default, wheel_contact_default, 1)

    # 3. Replace each wheel's 3-line collision geom with the new hub geom + rollers.
    for wheel_name, direction in WHEELS:
        rollers = rollers_for_wheel(direction, a, step)
        check_overlap(rollers)
        replacement = render_wheel_block(wheel_name, hub_radius, rollers)

        pattern = re.compile(
            r'        <geom name="' + re.escape(wheel_name) + r'_collision" type="cylinder" size="0\.04 0\.0275"\n'
            r'              pos="0 0 0\.0109" class="collision"\n'
            r'              condim="4" friction="1\.0 0\.3 0\.001" priority="1"/>'
        )
        new_text, n = pattern.subn(replacement, xml_text, count=1)
        if n != 1:
            raise SystemExit(f"could not find the existing collision geom block for {wheel_name}")
        xml_text = new_text

    # 4. Remove the <contact> block of wheel-ground anisotropic friction pairs.
    #    No comment header precedes it in m3pro_robot.xml, so don't require one.
    contact_pattern = re.compile(
        r"\n  <contact>\n"
        r'(?:    <pair geom1="\w+" geom2="ground"\n'
        r'          condim="4" friction="[0-9. ]+"/>\n)+'
        r"  </contact>\n",
    )
    new_text, n = contact_pattern.subn("\n", xml_text, count=1)
    if n != 1:
        raise SystemExit("could not find the <contact> block of mecanum pair overrides to remove")
    xml_text = new_text

    # 5. Insert the wheel-layout doc comment describing the new approach + caveat,
    #    right before the first wheel body. Replaces the old comment if one is
    #    already present (e.g. re-running against a file this script previously
    #    migrated), otherwise inserts fresh -- pre-migration m3pro_robot.xml has no
    #    such comment block at all, just a bare "<!-- Front-left wheel -->".
    new_comment = (
        "      <!--\n"
        "        Wheel layout (top-down, +X is front):\n"
        "          lwheel1 (FL)  ___  rwheel1 (FR)\n"
        "                       |   |\n"
        "          lwheel2 (RL)  ===  rwheel2 (RR)\n"
        "\n"
        f"        Mecanum X-pattern via {NUM_ROLLERS} passive free-spinning roller bodies per wheel\n"
        "        (see scripts/generate_mecanum_rollers.py), not friction anisotropy. Roller count/\n"
        "        radius/tilt are a simulation approximation, not a reverse-engineered match to the\n"
        "        real ROSMASTER M3 Pro's physical roller wheels. The pin-placement radius is solved\n"
        f"        (not just WHEEL_RADIUS - ROLLER_RADIUS) so the effective rolling radius lands\n"
        f"        exactly on the nominal {WHEEL_RADIUS} m used in controllers.yaml's kinematics;\n"
        "        see the cos(step/2) note in compute_geometry() before changing roller count/radius.\n"
        "        NUM_ROLLERS is a deliberate smoothness/grip-authority trade-off; see the design note\n"
        "        in the generator script and CLAUDE.md before changing it.\n"
        "      -->\n\n"
    )
    wheel_anchor = '      <!-- Front-left wheel -->'
    old_comment_pattern = re.compile(r"      <!--\n(?:.*\n)*?      -->\n\n(?=      <!-- Front-left wheel -->)")
    if old_comment_pattern.search(xml_text):
        xml_text = old_comment_pattern.sub(new_comment, xml_text, count=1)
    elif wheel_anchor in xml_text:
        xml_text = xml_text.replace(wheel_anchor, new_comment + wheel_anchor, 1)
    else:
        raise SystemExit("could not find the Front-left wheel comment to anchor the wheel-layout doc comment")

    return xml_text


def validate_with_mujoco(xml_text):
    import mujoco

    with tempfile.TemporaryDirectory() as tmpdir:
        # meshdir="../meshes" is relative to the mjcf file's own directory, so
        # mirror the real mjcf/ + meshes/ sibling layout under the temp root.
        tmp_root = Path(tmpdir)
        mjcf_dir = tmp_root / "mjcf"
        mjcf_dir.mkdir()
        tmp_path = mjcf_dir / "m3pro_candidate.xml"
        tmp_path.write_text(xml_text)
        real_meshdir = (Path(__file__).parent.parent / "meshes").resolve()
        (tmp_root / "meshes").symlink_to(real_meshdir)
        mujoco.MjModel.from_xml_path(str(tmp_path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mjcf-path",
        default=str(Path(__file__).parent.parent / "mjcf" / "m3pro_robot.xml"),
        help="Path to m3pro_robot.xml (default: the package's own mjcf/m3pro_robot.xml)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print result to stdout instead of writing the file")
    args = parser.parse_args()

    mjcf_path = Path(args.mjcf_path)
    xml_text = mjcf_path.read_text()

    if "wheel_contact" in xml_text or "_roller0" in xml_text:
        raise SystemExit(
            f"{mjcf_path} already appears migrated (found 'wheel_contact' or '_roller0'). "
            f"Run `git checkout <pre-migration-commit> -- {mjcf_path}` first if you want to regenerate."
        )

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

    print("Validating candidate XML with mujoco.MjModel.from_xml_path...", file=sys.stderr)
    validate_with_mujoco(new_text)
    print("Validation OK.", file=sys.stderr)

    if args.dry_run:
        print(new_text)
    else:
        mjcf_path.write_text(new_text)
        print(f"Wrote {mjcf_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
