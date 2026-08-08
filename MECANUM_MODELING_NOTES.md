# How This Repo Gets Smooth Mecanum Motion in MuJoCo

Notes on the modeling and control choices behind the smooth mecanum behavior in this
repo, for reference when building a mecanum sim elsewhere.

## 1. The core trick: rollers are real bodies, not a friction hack

The wheel is **not** a single cylinder with a clever anisotropic friction coefficient.
Each wheel is a hub (cylinder) plus N small sphere "rollers" (default 12,
`wheel_code_gen.py`), each attached to the hub by its own `hinge` joint whose axis is
tilted ~45° (tangent to the roller's pin-to-pin line on the wheel).

```
<body name="..._wheel_intermediate_link">
    <joint name="..._wheel_rolling_joint" axis="0 1 0" />      <!-- driven axis -->
    <geom type="cylinder" .../>                                 <!-- hub, visual/backbone -->

    <body name="..._roller_i_link" pos="...">
        <joint type="hinge" axis="<tilted roller axis>" damping="0.1"
               limited="false" actuatorfrclimited="false"/>     <!-- free-spinning -->
        <inertial mass="0.001" diaginertia="1e-5 1e-5 1e-5" />  <!-- near-massless -->
        <geom type="sphere" size="..." contype="1" conaffinity="0"/>
    </body>
</body>
```

When the hub is driven, whichever roller is in contact with the ground touches down as
a free-spinning sphere. The contact solver naturally resolves rolling in the driven
direction and free slip along the roller's tilted axis — the mecanum kinematics fall
out of ordinary rigid-body contact, no anisotropic-friction plugin or special contact
model required. This is why it holds up well under MuJoCo's default solver.

**Roller axis geometry** (`wheel_code_gen.py:32-47`): for each roller `i`, two "pin"
points are computed on the two wheel faces at radius `r - roller_r`, one at angular
step `i` and the other at the *adjacent* step (`i+1` or `i-1` depending on the `type`
flag, which controls whether rollers angle "/" or "\" — i.e. left- vs right-handed
wheels). The roller body sits at the midpoint of those two pins, and its hinge axis is
the vector between them. This is the standard construction for a mecanum roller axis
and is worth copying directly if you're deriving your own.

## 2. Collision filtering to avoid self-collision

Both the hub cylinder and every roller sphere use `contype="1" conaffinity="0"`, while
the ground plane uses `contype="1" conaffinity="1"`. Under MuJoCo's rule
`(contype1 & conaffinity2) || (contype2 & conaffinity1)`:

- wheel part vs. ground → collides (`1 & 1`)
- wheel part vs. any other wheel part (hub or roller) → **does not** collide (`1 & 0`)

This matters because the rollers are packed tightly around the hub at basically the
same radius as the hub surface — without this filtering they'd constantly
self-intersect with the hub and each other, injecting spurious contact forces and
jitter. If your sim is vibrating, check this first.

## 3. Roller sizing — a subtlety (and a bug to avoid)

`roller_ratio = 0.08 / 0.127` is a fixed ratio hard-coded in `wheel_code_gen.py`, used
to size each roller sphere relative to the wheel radius (`roller_r = r * roller_ratio`,
~63% of wheel radius here — deliberately large rollers, not the small rollers you may
picture from real hardware).

Two things worth knowing:

- **Large rollers relative to wheel radius, plus a high roller count (12), keep the
  ground-contact point nearly continuous as the wheel turns.** Real mecanum wheels
  with few/small rollers produce a visible "bump-bump-bump" in sim as contact hands
  off from one roller to the next. Bigger rollers with more of them minimize that
  handoff discontinuity — likely a bigger contributor to the smoothness you're seeing
  than any solver setting.
- **The standalone `wheel_code_gen.py` output has an actual bug**: the roller sphere's
  `size` is written as `r` (full wheel radius) instead of `roller_r`
  (`wheel_code_gen.py:53`), so a freshly generated `wheel.xml` has oversized roller
  spheres equal to the wheel's own radius. The *actual* SUMMIT-XL model
  (`summit_xls.urdf.xml`) does **not** have this bug — its roller spheres are
  hand-set to `size="0.08"` (correctly small). If you adapt this generator, fix that
  line (`size="{str(roller_r)}"`) before trusting its output.

## 4. Roller joints are damped but effectively free

Roller hinge joints use `damping="0.1"`, `limited="false"`, `actuatorfrclimited="false"`
— free rotation, not driven, but with a small viscous damping term. That damping isn't
there to resist the slip motion (0.1 is tiny); it's there to dissipate high-frequency
numerical chatter and keep the free DOF from ringing. Combined with the near-massless
roller inertial properties (`mass="0.001"`, `diaginertia="1e-5 1e-5 1e-5"`), the rollers
behave as lightweight kinematic contact aids rather than bodies with real momentum — 
they don't meaningfully perturb the vehicle's dynamics, but they're not a totally rigid
zero-damping joint either. If your rollers are undamped, that's a plausible source of
jitter.

## 5. No exotic solver/contact tuning

Notably, nothing in this repo overrides MuJoCo's defaults: no `<option>` block, no
custom `condim`, `friction`, `solref`/`solimp`, or `noslip` iterations anywhere in the
model files. It runs on the default Newton solver, default `condim="3"` contacts, and
default friction `"1 0.005 0.0001"`. **The smooth behavior comes entirely from the
geometric/kinematic modeling in sections 1–4, not from solver parameter tuning.** If
you've been reaching for solver tweaks to fix mecanum jitter, it's worth first checking
whether your roller geometry, collision filtering, and actuator type (next section) are
right — that's where this repo's smoothness actually comes from.

## 6. Actuation: force/torque motors + your own velocity controller, never `<velocity>`

The README calls this out explicitly and it matters:

```xml
<!-- Wrong for this wheel model -->
<velocity name="mecanum_wheel_joint" .../>

<!-- Right -->
<motor name="mecanum_wheel_joint" .../>
```

Reasoning: MuJoCo's `<velocity>` actuator is an implicit PD servo baked into the
actuator itself, tuned assuming the joint it drives directly determines the body's
ground-contact velocity. With this wheel model, the hub's ground velocity is a
combination of the driven hinge *and* whatever the currently-contacting roller is doing
passively — the actuator doesn't have full authority over the DOF it thinks it's
servoing. That mismatch fights the roller physics and produces instability/oscillation.

Instead, `summit_xls_actuator.xml` declares plain force actuators:

```xml
<motor name="front_right_wheel_rolling_joint" joint="front_right_wheel_rolling_joint" ctrlrange="-10 10"/>
```

and `summit_test.py` implements velocity control *outside* MuJoCo, in Python, as an
explicit P-controller on measured joint velocity:

```python
mobile_dot[i] = d.qvel[<index of wheel i's rolling joint>]   # measured wheel speed
command = (target_vel - mobile_dot) * mobile_kv               # mobile_kv = 200.0
d.ctrl[i] = command[...]                                       # force command
```

Key details:
- `ctrlrange="-10 10"` on the motor acts as a hard torque limit (±10 N·m at gear=1),
  which keeps a fairly aggressive P gain (200) from producing violent transients when
  the velocity error is large.
- The control loop starts with `target_vel = 0` for the first 1000 steps
  (`delay = 1000`) purely to let the robot settle onto the ground under gravity before
  commanding any motion — avoids an initial-contact impact transient corrupting the
  first commanded move.
- The script manually permutes actuator/measurement indices
  (`d.ctrl[0] = command[1]`, etc., `summit_test.py:51-54`) because the actuator
  declaration order (FR, FL, BR, BL) doesn't match the `qvel` index order used for
  measurement. Worth double-checking this kind of ordering in your own model — a silent
  mismatch here would look like "weird slightly-wrong motion," not a crash.

## Summary checklist if you're chasing smoothness

- [ ] Rollers modeled as real spheres with tilted free hinge joints (not a friction hack)
- [ ] Roller axis constructed from adjacent pin points on the two wheel faces (§1)
- [ ] `conaffinity="0"` on hub + rollers so they don't self-collide, but still `contype`
      matches the ground's `conaffinity` so they collide with the world
- [ ] Rollers sized generously relative to wheel radius, and enough of them (10-12+) to
      minimize contact handoff bumps — don't undersize them the way the buggy
      `wheel_code_gen.py` output does
- [ ] Small nonzero damping (~0.1) on roller joints, near-massless roller inertial
      properties
- [ ] No need to reach for custom solver/contact params — defaults are fine if the
      geometry above is right
- [ ] `<motor>` actuators only, never `<velocity>`; do closed-loop velocity control
      yourself in Python against measured `qvel`
- [ ] Let the robot settle under gravity before commanding motion
- [ ] Double-check actuator-order vs. qvel-index-order mapping
