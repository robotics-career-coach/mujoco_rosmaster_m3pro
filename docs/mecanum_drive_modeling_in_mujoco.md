# Modeling Mecanum Drives in MuJoCo

This document summarizes the available approaches for simulating mecanum-wheeled
bases in MuJoCo, their tradeoffs, and guidance on which to use. It's a general
reference for anyone modeling a mecanum-wheeled base in MuJoCo, not tied to any
particular project's implementation.

**Contents**
- [Background: why mecanum wheels are hard to simulate](#background-why-mecanum-wheels-are-hard-to-simulate)
- [Modeling approaches](#modeling-approaches)
  1. [Full roller geometry](#1-full-roller-geometry-high-fidelity-route)
  2. [Anisotropic friction on a simplified shape](#2-anisotropic-friction-on-a-simplified-shape-smart--compromise-route)
  3. [Rigid roller shapes with anisotropic friction](#3-rigid-roller-shapes-with-anisotropic-friction-hybrid-route)
  4. [Plain wheel + isotropic friction](#4-plain-wheel--isotropic-friction-not-recommended)
- [Comparison summary](#comparison-summary)
- [Practical checklist (approach 2)](#practical-checklist-approach-2)
- [References](#references)

## Background: why mecanum wheels are hard to simulate

A mecanum wheel is a normal wheel with a ring of small passive rollers mounted
around its circumference at **45°** to the wheel's rotation axis. When the wheel
spins, the rollers let the contact patch slide freely along their own axis while
still gripping perpendicular to it. Combine four wheels mounted in an "X"
pattern (front-left/rear-right share one roller handedness, front-right/rear-left
share the other) and you get full holonomic motion from four single-axis motors.

Physics engines don't have a built-in "mecanum wheel" primitive. The 45° roller
behavior has to be built out of more basic constructs, and the fidelity/cost
tradeoff you pick has real consequences for how well the sim matches the
kinematics your motion controller assumes.

An important distinction, because it's easy to conflate the two: **omni-wheels**
have rollers mounted at 90° (perpendicular) to the axle, giving simple
fore-aft-grip / lateral-slip behavior. **Mecanum wheels** have rollers at 45°,
giving a *diagonal* free-slip axis. Techniques that work well for omni-wheels
don't automatically transfer to mecanum wheels — the friction axes have to be
tilted correctly, not just split into "rolling" vs "axle" directions.

## Modeling approaches

### 1. Full roller geometry ("high-fidelity" route)

Model every individual roller as its own body with its own free-spinning joint,
mounted at 45° around the wheel hub. The main wheel becomes an assembly of
~8-12 small cylinder/capsule bodies rather than a single geom.

**Advantages**
- Most physically accurate — you're not approximating anything, the roller
  geometry and free rotation are simulated directly.
- Handles complex/uneven terrain correctly, since real roller-ground contact
  and momentary single-roller support are actually modeled.
- No need to hand-derive friction coefficients or worry about MuJoCo's
  contact-frame-orientation quirks (see below) — the physics falls out of the
  geometry.

**Disadvantages**
- Computationally expensive: 4 wheels × ~8-12 rollers = many extra bodies,
  joints, and contacts, which increases sim step cost and can hurt real-time
  performance.
- More prone to solver/contact instability — more simultaneous, small,
  frequently-changing contacts to resolve each step.
- More XML to write/maintain (though this can be templated/macro'd — see
  [JunHeonYoon/mujoco_mecanum](https://github.com/JunHeonYoon/mujoco_mecanum),
  which generates roller assemblies from a parametrized macro).
- Reference implementations recommend using **force-based (motor) actuators
  on the roller/wheel joints rather than velocity actuators**, since velocity
  servoing on many small interacting joints tends to fight the roller
  dynamics.

**When to use it**: when simulation accuracy matters more than real-time
performance — e.g. validating controller behavior on rough terrain, generating
training data, or any case where you specifically care about wheel-terrain
interaction fidelity.

### 2. Anisotropic friction on a simplified shape ("smart" / compromise route)

Model each wheel as a single simplified collision shape (not the visual mesh)
and use MuJoCo's per-direction friction coefficients to fake the roller
behavior: high friction in the grip direction, near-zero friction along the
roller's free-slip axis. No extra bodies or joints — the wheel's own drive
joint is the only DOF.

**Advantages**
- Cheap: one collision geom per wheel, no extra contacts or joints beyond
  what a normal wheeled robot already has.
- Fast and numerically stable compared to the full-roller approach.
- Good enough on flat/typical indoor floors, which covers most robotics use
  cases (nav2 testing, arm-on-mobile-base tasks, etc).

**Disadvantages / gotchas** (this is where it's easy to get subtly wrong)
- **Geometry choice matters a lot.** MuJoCo's anisotropic friction is applied
  in the *contact frame*, and that frame's orientation is derived from the
  colliding geom's shape. For **capsule** geoms, the contact tangent frame
  reliably rotates with the capsule's own longitudinal axis. For **cylinder,
  box, and sphere** geoms, this is not guaranteed — the tangent frame can stay
  effectively locked to the world frame regardless of how the geom itself is
  oriented. This is a documented MuJoCo behavior
  ([google-deepmind/mujoco#67](https://github.com/google-deepmind/mujoco/issues/67)).
  **Use capsules for the collision geometry, not cylinders,** if you're relying
  on anisotropic friction to encode a direction.
- **The friction axis has to actually be diagonal (45°) for mecanum, not
  90°.** It's easy to copy an omni-wheel recipe (grip fore-aft, slip along the
  axle) and get something that drives forward fine but produces wrong forces
  under strafe/rotate commands, because the kinematic mixing your motion
  controller uses assumes the true 45° roller axis. The capsule's long axis
  needs to be oriented at 45° from the rolling direction, mirrored per corner
  to match the mecanum "X" pattern (FL/RR one handedness, FR/RL the other).
- Requires `condim="4"` (or `"6"`) and `cone="elliptic"` at the `<option>`
  level — the default pyramidal cone approximates anisotropic friction poorly.
- Slip-direction friction should be quite low (roughly `0.01`–`0.05`) to
  properly emulate a freely-spinning roller; leaving it too high (e.g. `0.3`)
  reintroduces resistance that shouldn't be there and can look like stick-slip
  jitter.
- Still an approximation: doesn't capture roller-scale contact effects (e.g.
  the small periodic bumps from the wheel not being perfectly round due to
  discrete rollers).

**When to use it**: the default choice for most robot bases on typical flat
ground. It's the right tradeoff of accuracy vs. performance for ROS2
development/testing where the mecanum base is a means to an end, not the
object of study. Requires getting the capsule orientation and friction values
right per the notes above.

### 3. Rigid roller shapes with anisotropic friction (hybrid route)

A middle ground described in academic literature (e.g. "Fast and Accurate
Simulation of Mecanum Wheels with Passive Rollers Emulated by Fixed Joints and
Anisotropic Friction"): model roller-shaped geoms around the wheel circumference,
but attach them rigidly (no additional joint/DOF) rather than giving each one a
free-spinning joint, and let anisotropic friction stand in for the roller's
free rotation.

**Advantages**
- More faithful to the real roller layout than a single anisotropic-friction
  capsule (approach 2), since it reflects the actual discrete roller
  positions around the wheel.
- Cheaper than approach 1 — no extra joints, since the rollers don't need
  their own DOF (the anisotropic friction does the work a free joint would).

**Disadvantages**
- More complex to build than approach 2, without the "no approximation"
  guarantee of approach 1 — you're still relying on the friction model being
  correctly oriented per roller.
- Less commonly implemented / fewer off-the-shelf references than approaches
  1 and 2.

**When to use it**: if approach 2's single-shape approximation isn't accurate
enough for your use case (e.g. you care about the small periodic disturbances
real mecanum wheels produce) but approach 1's per-roller joints are too
expensive.

### 4. Plain wheel + isotropic friction (not recommended)

Just a cylinder with standard uniform friction. The base will physically move
when driven, but lateral commands will be resisted by ordinary friction
instead of sliding, so the resulting motion won't match real mecanum
kinematics at all. Only useful for the coarsest smoke tests (e.g. "does
anything move when I send a velocity command") — never for actually
validating holonomic behavior.

## Comparison summary

| # | Approach | Fidelity | Cost | Best for |
|---|---|---|---|---|
| 1 | Full roller bodies | Highest | Highest | Terrain interaction studies, training data, high-accuracy validation |
| 2 | Anisotropic friction, capsule | Good (flat ground) | Low | Default choice for ROS2 dev/test on typical floors |
| 3 | Rigid roller shapes + anisotropic friction | Better than single-capsule, less than full rollers | Medium | When approach 2 isn't accurate enough but approach 1 is too expensive |
| 4 | Plain wheel, isotropic friction | Wrong | Lowest | Never, except a rough "does it move" smoke test |

## Practical checklist (approach 2)

Approach 2 is singled out here because its failure modes are silent: a
misoriented friction axis still drives forward correctly and only breaks
under strafe/rotate, which makes it the easiest of the four approaches to get
subtly wrong without noticing. Approaches 1 and 3 don't share this specific
gotcha, since their behavior falls out of geometry rather than a
hand-oriented friction axis.

- [ ] Collision geoms are `type="capsule"`, not `cylinder`.
- [ ] Capsule long axis is oriented along the wheel's true 45° roller
      direction, mirrored correctly per corner (FL/RR vs FR/RL).
- [ ] `<option cone="elliptic">` is set.
- [ ] Contact/geom `condim` is `4` or `6` (not `3`, which can't represent
      asymmetric tangential friction).
- [ ] Grip-direction friction ~1.0–1.5, slip-direction friction ~0.01–0.05.
- [ ] Verified in `simulate` with contact frame visualization that the
      tangent axes rotate with the chassis as expected, not the world frame.
- [ ] Verified forward, strafe, and rotate-in-place commands all produce
      smooth motion with the actual motion controller (e.g. a ROS2 driver
      node), not just isolated joint-slider tests.

## References

- Ekumen, ["Modeling LeKiwi's omni-base in MuJoCo"](https://ekumenlabs.com/blog/posts/modeling-lekiwis-omni-base-in-mujoco/) — the most detailed public walkthrough of the anisotropic-friction approach and the cylinder/capsule contact-frame gotcha.
- [google-deepmind/mujoco#67](https://github.com/google-deepmind/mujoco/issues/67) — GitHub issue documenting that anisotropic friction contact-frame orientation depends on the collider type.
- [JunHeonYoon/mujoco_mecanum](https://github.com/JunHeonYoon/mujoco_mecanum) — reference implementation of the full-roller-geometry approach with a macro generator.
- "Fast and Accurate Simulation of Mecanum Wheels with Passive Rollers Emulated by Fixed Joints and Anisotropic Friction" — academic description of the rigid-roller-shape hybrid approach.
- MuJoCo documentation: [Modeling — friction and contact dimensionality](https://mujoco.readthedocs.io/en/latest/modeling.html), [Computation — friction cones](https://mujoco.readthedocs.io/en/stable/computation/index.html#friction-cones).
